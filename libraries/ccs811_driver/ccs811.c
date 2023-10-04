#include "string.h"
#include <stdio.h>
#include <stdint.h>

#include "ccs811.h"

#define DEBUG_LOG_MODULE_NAME "CCS811_DRIVER"
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG
#include "debug_log.h"

#define __SYSTEM_CLOCK_64M (64000000UL)
uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_64M;

#include "nrf_delay.h"

#include "app_scheduler.h"

/** Maximum I2C write transfer */
#define MAX_WRITE_SIZE 16

/** Maximum I2C read transfer */
#define MAX_READ_SIZE 16

#define GAS_SENSOR_PWR_ON_DELAY_MS (30) ///< ms for sensor to become active after pwr on. NB: May be up tp 70 ms for first boot after gas sensor firmware download. 20 ms from datasheet + margin.

static i2c_conf_t m_i2c_conf = {
    .clock = 100000,
    .pullup = false,
};

/**
    @brief      Write with I2C (function required by STMicroelectronics lib).
    @param[in]  reg First register to write to.
    @param[out] bufp Pointer in RAM to the data to be written.
    @param[in]  len Number of registers (of 1 byte) to write.
*/
static int32_t ccs811_writeI2C(
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{
    i2c_res_e res;
    uint8_t tx[MAX_WRITE_SIZE + 1];

    nrf_delay_us(20);
    if ((res = sx1509_set_pin_level(BANK_A, CCS_WAKE, 0)) != 0)
    {
        LOG(LVL_ERROR, "Error while clearing the wake pin of the ccs881 sensor");
        return res;
    }
    nrf_delay_us(50);

    tx[0] = reg;
    memcpy(&tx[1], bufp, len);
    res = I2C_init(&m_i2c_conf);
    if (res == I2C_RES_OK || res == I2C_RES_ALREADY_INITIALIZED)
    {
        i2c_xfer_t xfer_tx = {
            .address = CCS_811_ADDRESS,
            .write_ptr = tx,
            .write_size = len + 1,
            .read_ptr = NULL,
            .read_size = 0};

        res = I2C_transfer(&xfer_tx, NULL);
        I2C_close();
    }

    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write ERROR res:%u reg: 0x%x len: %u", res, reg, len);
        return res;
    }
    if ((res = sx1509_set_pin_level(BANK_A, CCS_WAKE, 1)) != 0)
    {
        LOG(LVL_ERROR, "Error while clearing the wake pin of the ccs881 sensor");
    }

    return res;
}

/**
    @brief     Read from I2C (function required by STMicroelectronics lib).
    @param[in] reg First register to read.
    @param[in] bufp Pointer to store read registers.
    @param[in] len Number of registers (of 1 byte) to read.
*/
static int32_t ccs811_readI2C(
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{

    i2c_res_e res;
    uint8_t rx[MAX_READ_SIZE + 1];
    uint8_t tx[MAX_WRITE_SIZE + 1];

    nrf_delay_us(20);
    if ((res = sx1509_set_pin_level(BANK_A, CCS_WAKE, 0)) != 0)
    {
        LOG(LVL_ERROR, "Error while clearing the wake pin of the ccs881 sensor");
        return res;
    }
    nrf_delay_us(50); // Allow the CCS811 to power up.

    tx[0] = reg; // Directly use the register address without setting the MSB

    res = I2C_init(&m_i2c_conf);

    if (res == I2C_RES_OK || res == I2C_RES_ALREADY_INITIALIZED)
    {
        i2c_xfer_t xfer_rx = {
            .address = CCS_811_ADDRESS,
            .write_ptr = tx,
            .write_size = 1,
            .read_ptr = rx,
            .read_size = len,
            .custom = 0};

        res = I2C_transfer(&xfer_rx, NULL);
        memcpy(bufp, &rx[0], len);
    }
    I2C_close();

    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C read ERROR res:%u reg: 0x%x len: %u", res, reg, len);
        return res;
    }
    if ((res = sx1509_set_pin_level(BANK_A, CCS_WAKE, 1)) != 0)
    {
        LOG(LVL_ERROR, "Error while clearing the wake pin of the ccs881 sensor");
    }
    return res;
}

uint8_t ccs811_verify(void)
{
    uint8_t reg_val = 0;
    uint8_t err;
    if ((err = ccs811_readI2C(HW_ID_REG, &reg_val, 1)) != 0)
    {
        return err;
    }
    LOG(LVL_DEBUG, "HW_ID_REG_VAL: 0x%x", reg_val);

    return (reg_val == HW_ID_REG_VAL) ? 0 : -1;
}

uint8_t ccs811_status(void)
{
    uint8_t reg_val = 0;
    uint8_t err;
    if ((err = ccs811_readI2C(CCS_STATUS_REG, &reg_val, 1)) != 0)
    {
        LOG(LVL_DEBUG, "ccs811_status: 0x%x", reg_val);
        return err;
    }
    LOG(LVL_DEBUG, "ccs811_status: 0x%x", reg_val);

    return reg_val;
}

uint8_t ccs811_read_alg_result(ccs811_alg_result_t *result)
{
    uint8_t err;
    uint8_t buf[2]; // ALG_RESULT_DATA consists of 4 bytes: 2 for eCO2

    if (!result)
    {
        return -1; // Error: Null pointer provided
    }

    // Read ALG_RESULT_DATA register
    if ((err = ccs811_readI2C(ALG_RESULT_DATA, buf, 2)) != 0)
    {
        LOG(LVL_ERROR, "Failed to read the ALG_RESULT_DATA register of the ccs811 sensor");
        return err; // Error during I2C read
    }

    // Convert the read buffer to eCO2 and TVOC values
    result->eCO2 = (uint16_t)(buf[0] << 8) | buf[1];

    return err; // Success
}

uint8_t ccs811_read_error_id(uint8_t *error_id)
{
    uint8_t err;

    if (!error_id)
    {
        return -1; // Error: Null pointer provided
    }

    // Read ERROR_ID register
    if ((err = ccs811_readI2C(ERROR_ID_REG, error_id, 1)) != 0)
    {
        LOG(LVL_ERROR, "Error while reading the ERROR-ID register of the ccs811 sensor");
        return err; // Error during I2C read
    }

    return err; // Success
}

uint32_t ccs811_pre_init(void)
{
    if ((sx1509_set_pin_as_output(BANK_A, CCS_PWR_CTRL)) != 0)
    {
        LOG(LVL_ERROR, "Error while configuring the power control pin of the ccs811 sensor");
    }
    if ((sx1509_set_pin_as_output(BANK_A, CCS_WAKE)) != 0)
    {
        LOG(LVL_ERROR, "Error while configuring the wake pin of the ccs811 sensor");
    }
    if ((sx1509_set_pin_as_output(BANK_A, CCS_RESET)) != 0)
    {
        LOG(LVL_ERROR, "Error while configuring the reset pin of the ccs811 sensor");
    }

    if ((sx1509_set_pin_level(BANK_A, CCS_PWR_CTRL, 1)) != 0)
    {
        LOG(LVL_ERROR, "Error while setting the power control pin of the ccs811 sensor");
    }

    if ((sx1509_set_pin_level(BANK_A, CCS_RESET, 1)) != 0)
    {
        LOG(LVL_ERROR, "Error while setting the reset pin of the ccs811 sensor");
    }

    return APP_SCHEDULER_STOP_TASK;
}

uint8_t ccs811_init(void)
{
    uint8_t err;
    uint8_t buf[1];

    if ((err = ccs811_verify()) != 0)
    {
        LOG(LVL_ERROR, "Failed to verify ccs811 sensor: %u", err);
        return err;
    }

    buf[0] = 0;
    LOG(LVL_DEBUG, "Set the sensor into App mode");
    if ((err = ccs811_writeI2C(APP_START_REG, buf, 0)) != 0)
    {
        LOG(LVL_ERROR, "Failed to set the ccs811 sensor into APP mode: %u", err);
        return err;
    }
    nrf_delay_us(30);
    return err;
}

uint8_t ccs811_start()
{
    uint8_t err;
    uint8_t buf[1];
    // Set measurement mode to 60 sec
    buf[0] = DRIVE_MODE_60SEC | INTERRUPT_DRIVEN;
    if ((err = ccs811_writeI2C(MEAS_MODE_REG, buf, 1)) != 0)
    {
        LOG(LVL_ERROR, "Failed to set ccs811 sensor measurement mode: %u", err);
        return err;
    }
    LOG(LVL_DEBUG, "Gas sensor has been started;");
    return err;
}

uint8_t ccs811_set_env_data(uint16_t humid, float temp)
{
    uint8_t err;
    uint8_t env_data[4];

    uint16_t rh_ppth = humid * 10;
    int32_t temp_mdeg = (int32_t)(temp * 1000.0f);

    LOG(LVL_ERROR, "Calibrating gas sensor: humid out %d [ppt], temp out: %d [mdeg C]\r\n", rh_ppth, temp_mdeg);

    // Check that supplied values are within bounds.
    if ((rh_ppth > M_RH_PPTH_MAX) || (temp_mdeg < M_TEMP_MILLIDEG_MIN) || (temp_mdeg > M_TEMP_MILLIDEG_MAX))
    {
        LOG(LVL_ERROR, "Values are not within the range!!!");
        return -1;
    }

    /* The least significant bit of the most significant byte represents a humidity and temperature fraction (1/2).
    This section sets the Environement Data Register values. See the CCS811 datasheet for more information. */

    if ((rh_ppth % 10) > 7)
    {
        env_data[0] = ((rh_ppth / 10) + 1) << 1;
    }
    else
    {
        env_data[0] = ((rh_ppth / 10)) << 1;
    }

    if (((rh_ppth % 10) > 2) && ((rh_ppth % 10) < 8))
    {
        env_data[0] |= 1; // Set least significant bit in the most significant byte.
    }
    else
    {
        // Do nothing, decimal value = 0.
    }

    env_data[1] = 0; // According to the datsasheet, only the most significant byte is used.

    temp_mdeg += M_TEMP_OFFSET; // Add offset according to datasheet.

    if ((temp_mdeg % 1000) >= 750)
    {
        env_data[2] = ((temp_mdeg / 1000) + 1) << 1;
    }
    else
    {
        env_data[2] = (temp_mdeg / 1000) << 1;
    }

    if (((temp_mdeg % 1000) >= 250) && ((temp_mdeg % 1000) < 750))
    {
        env_data[2] |= 1; // Set least significant bit in the most significant byte.
    }
    else
    {
        // Do nothing, decimal value = 0.
    }

    env_data[3] = 0; // According to the datasheet, only the most significant byte is used.

    // Write the environmental data to the CCS811
    if ((err = ccs811_writeI2C(ENV_DATA, env_data, 4)) != 0)
    {
        LOG(LVL_ERROR, "Failed to set ccs811 sensor env_data: %u", err);
        return err;
    }

    return err;
}
