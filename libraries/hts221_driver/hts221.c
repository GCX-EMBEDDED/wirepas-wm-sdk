#include "string.h"
#include <stdio.h>
#include <stdint.h>

#include "hts221.h"

#define DEBUG_LOG_MODULE_NAME "HTC221_DRIVER"
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG
#include "debug_log.h"

/** Maximum I2C write transfer */
#define MAX_WRITE_SIZE 16

/** Maximum I2C read transfer */
#define MAX_READ_SIZE 16

#define HTS221_ADDR 0x5F

#define AV_CONF_RESERVED_BITS (0xC0)   ///< Reserved bits in the AV_CONF register.
#define CTRL_REG1_RESERVED_BITS (0x78) ///< Reserved bits in the CTRL_REG1 register.
#define CTRL_REG2_RESERVED_BITS (0x7C) ///< Reserved bits in the CTRL_REG2 register.
#define CTRL_REG3_RESERVED_BITS (0x3B) ///< Reserved bits in the CTRL_REG3 register.

static i2c_conf_t m_i2c_conf = {
    .clock = 100000,
    .pullup = false,
};

/**@brief Configuration struct.
 */
typedef struct
{
    hts221_evt_handler_t evt_handler; ///< Event handler, called when data is ready.
    bool enabled;                     ///< Driver enabled flag.
    hts221_calib_t calib;             ///< Calibration struct.
} htc221_t;

/**@brief Stored configuration.
 */
static htc221_t htc221_sensor;

/**
    @brief      Write with I2C (function required by STMicroelectronics lib).
    @param[in]  reg First register to write to.
    @param[out] bufp Pointer in RAM to the data to be written.
    @param[in]  len Number of registers (of 1 byte) to write.
*/
static int32_t htc221_writeI2C(
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{

    i2c_res_e res;
    uint8_t tx[MAX_WRITE_SIZE + 1];

    tx[0] = reg;
    memcpy(&tx[1], bufp, len);
    res = I2C_init(&m_i2c_conf);
    if (res == I2C_RES_OK || res == I2C_RES_ALREADY_INITIALIZED)
    {
        i2c_xfer_t xfer_tx = {
            .address = HTS221_ADDR,
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
    }

    return res;
}

/**
    @brief     Read from I2C (function required by STMicroelectronics lib).
    @param[in] reg First register to read.
    @param[in] bufp Pointer to store read registers.
    @param[in] len Number of registers (of 1 byte) to read.
*/
static int32_t htc221_readI2C(
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{

    i2c_res_e res;
    uint8_t rx[MAX_READ_SIZE + 1];
    uint8_t tx[MAX_WRITE_SIZE + 1];
    tx[0] = (len > 1) ? (reg | 0x80) : reg; // set the MSB for multi-byte reads

    res = I2C_init(&m_i2c_conf);

    if (res == I2C_RES_OK || res == I2C_RES_ALREADY_INITIALIZED)
    {
        i2c_xfer_t xfer_rx = {
            .address = HTS221_ADDR,
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
    }

    return res;
}

uint8_t hts221_verify(void)
{
    uint8_t reg_val;
    uint8_t err;
    if ((err = htc221_readI2C(WHO_AM_I_REG, &reg_val, 1)) != 0)
    {
        return err;
    }
    LOG(LVL_DEBUG, "WHO_AM_I_VAL: 0x%x", reg_val);

    return (reg_val == WHO_AM_I_VAL) ? 0 : -1;
}

int16_t hts221_humidity_get(void)
{
    int16_t humidity;
    uint8_t humid_h;
    uint8_t humid_l;
    uint8_t err;

    int16_t _humidity;
    float hum = 0.0f;
    float h_temp = 0.0f;
    static const int16_t HUMID_MAX = 100;
    static const int16_t HUMID_MIN = 0;

    if ((err = htc221_readI2C(HUMIDITY_OUT_L_REG, &humid_l, 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C read error. err: %u", err);
        return err;
    }
    if ((err = htc221_readI2C(HUMIDITY_OUT_H_REG, &humid_h, 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C read error. err: %u", err);
        return err;
    }
    humidity = ((uint16_t)humid_h << 8) + humid_l;

    // Decode Humidity.
    hum = ((int16_t)(htc221_sensor.calib.H1_rH_x2) - (int16_t)(htc221_sensor.calib.H0_rH_x2)) / 2.0f; // Remove x2 multiple.

    // Calculate humidity in decimal of grade centigrades i.e. 15.0 = 150.
    h_temp = (((int16_t)humidity - (int16_t)htc221_sensor.calib.H0_T0_OUT) * hum) / ((int16_t)htc221_sensor.calib.H1_T0_OUT - (int16_t)htc221_sensor.calib.H0_T0_OUT);
    hum = ((int16_t)htc221_sensor.calib.H0_rH_x2) / 2.0; // Remove x2 multiple.
    _humidity = (int16_t)((hum + h_temp));               // Provide signed % measurement unit.

    // Limit output range according to HTS221 datasheet.
    if (_humidity >= HUMID_MAX)
    {
        _humidity = HUMID_MAX;
    }
    else if (_humidity <= HUMID_MIN)
    {
        _humidity = HUMID_MIN;
    }

    return (_humidity);
}

float hts221_temperature_get(void)
{
    int16_t t_out;
    uint8_t temp_h;
    uint8_t temp_l;
    uint8_t err;

    float deg = 0.0f;
    float t_temp = 0.0f;
    float _temperature = 0.0f;

    if ((err = htc221_readI2C(TEMP_OUT_L_REG, &temp_l, 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C read error. err: %u", err);
        return err;
    }
    if ((err = htc221_readI2C(TEMP_OUT_H_REG, &temp_h, 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C read error. err: %u", err);
        return err;
    }

    t_out = ((uint16_t)temp_h << 8) + temp_l;
    // T0_degC_x8.
    deg = ((int16_t)(htc221_sensor.calib.T1_degC_x8) - (int16_t)(htc221_sensor.calib.T0_degC_x8)) / 8.0f; // remove x8 multiple
    // Calculate Temperature in decimal of grade centigrades i.e. 15.0 = 150.
    t_temp = (((int16_t)t_out - (int16_t)htc221_sensor.calib.T0_OUT) * deg) / ((int16_t)htc221_sensor.calib.T1_OUT - (int16_t)htc221_sensor.calib.T0_OUT);
    deg = ((int16_t)htc221_sensor.calib.T0_degC_x8) / 8.0; // Remove x8 multiple.
    _temperature = deg + t_temp;                           // Provide signed celsius measurement unit.

    return _temperature;
}

uint8_t hts221_calib_get(hts221_calib_t *p_calib)
{
    uint8_t calib_raw[CALIBRATION_REGS_NUM];
    uint8_t err;

    err = htc221_readI2C(CALIBRATION_REGS, calib_raw, CALIBRATION_REGS_NUM);

    p_calib->H0_rH_x2 = calib_raw[0];
    p_calib->H1_rH_x2 = calib_raw[1];
    p_calib->T0_degC_x8 = (uint16_t)calib_raw[2] + ((uint16_t)(calib_raw[5] & 0x03) << 8);
    p_calib->T1_degC_x8 = (uint16_t)calib_raw[3] + ((uint16_t)((calib_raw[5] >> 2) & 0x03) << 8);
    p_calib->H0_T0_OUT = (int16_t)calib_raw[6] + ((int16_t)calib_raw[7] << 8);
    p_calib->H1_T0_OUT = (int16_t)calib_raw[10] + ((int16_t)calib_raw[11] << 8);
    p_calib->T0_OUT = (int16_t)calib_raw[12] + ((int16_t)calib_raw[13] << 8);
    p_calib->T1_OUT = (int16_t)calib_raw[14] + ((int16_t)calib_raw[15] << 8);

    return err;
}

uint8_t hts221_one_shot(void)
{
    uint8_t reg_val;
    uint8_t err;

    if ((err = htc221_readI2C(CTRL_REG2, &reg_val, 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C read error. err: %u", err);
        return err;
    }

    if (reg_val & CTRL_REG2_ONE_SHOT_Msk)
    {
        return 0;
    }

    reg_val |= CTRL_REG2_ONE_SHOT_Msk;

    err = htc221_writeI2C(CTRL_REG2, &reg_val, 1);

    return err;
}

uint8_t hts221_cfg_set(hts221_cfg_t *p_cfg)
{
    uint8_t err;

    if ((p_cfg->av_conf & AV_CONF_RESERVED_BITS) ||
        (p_cfg->ctrl_reg1 & CTRL_REG1_RESERVED_BITS) ||
        (p_cfg->ctrl_reg2 & CTRL_REG2_RESERVED_BITS) ||
        (p_cfg->ctrl_reg3 & CTRL_REG3_RESERVED_BITS))
    {
        return -1;
    }

    if ((err = htc221_writeI2C(AV_CONF_REG, &(p_cfg->av_conf), 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C write error. err: %u", err);
        return err;
    }
    if ((err = htc221_writeI2C(CTRL_REG1, &(p_cfg->ctrl_reg1), 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C write error. err: %u", err);
        return err;
    }
    if ((err = htc221_writeI2C(CTRL_REG2, &(p_cfg->ctrl_reg2), 1)) != 0)
    {
        LOG(LVL_ERROR, "I2C write error. err: %u", err);
        return err;
    }
    err = htc221_writeI2C(CTRL_REG3, &(p_cfg->ctrl_reg3), 1);
    return err;
}

uint8_t hts221_init(void)
{
    uint8_t err;

    if ((err = hts221_verify()) != 0)
    {
        LOG(LVL_ERROR, "Failed to verify hts221 sensor: %u", err);
        return err;
    }

    if ((err = hts221_calib_get(&htc221_sensor.calib)) != 0)
    {
        LOG(LVL_ERROR, "Failed to get caliberation data form hts221 sensor: %u", err);
        return err;
    }

    static hts221_cfg_t cfg = {
        .av_conf = AV_CONF_REG_DEFAULT,
        .ctrl_reg1 = CTRL_REG1_DEFAULT,
        .ctrl_reg2 = CTRL_REG2_DEFAULT,
        .ctrl_reg3 = CTRL_REG3_DEFAULT};

    err = hts221_cfg_set(&cfg);

    return err;
}

uint8_t hts221_enable(void)
{
    uint8_t err;

    if (htc221_sensor.enabled)
    {
        return 0;
    }

    htc221_sensor.enabled = true;

    static hts221_cfg_t cfg = {
        .av_conf = AV_CONF_REG_DEFAULT,
        .ctrl_reg1 = (CTRL_REG1_DEFAULT | (CTRL_REG1_PD_Active << CTRL_REG1_PD_Pos)),
        .ctrl_reg2 = CTRL_REG2_DEFAULT,
        .ctrl_reg3 = (CTRL_REG3_DEFAULT | (CTRL_REG3_DRDY_Enable << CTRL_REG3_DRDY_Pos) | (CTRL_REG3_DRDY_H_L_ActiveLow << CTRL_REG3_DRDY_H_L_Pos))};

    err = hts221_cfg_set(&cfg);

    return err;
}

uint8_t hts221_disable(void)
{
    uint8_t err;

    if (htc221_sensor.enabled == false)
    {
        return 0;
    }

    htc221_sensor.enabled = false;

    static hts221_cfg_t cfg = {
        .av_conf = AV_CONF_REG_DEFAULT,
        .ctrl_reg1 = CTRL_REG1_DEFAULT,
        .ctrl_reg2 = CTRL_REG2_DEFAULT,
        .ctrl_reg3 = CTRL_REG3_DEFAULT};

    err = hts221_cfg_set(&cfg);

    return err;
}