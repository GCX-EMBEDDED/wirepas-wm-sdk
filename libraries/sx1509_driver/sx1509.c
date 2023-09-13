#include "i2c.h"
#include "board.h"
#include "string.h"

#include "sx1509.h"

#define DEBUG_LOG_MODULE_NAME "SX1509_DRIVER"
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG
#include "debug_log.h"

/** Maximum I2C write transfer */
#define MAX_WRITE_SIZE 16

/** Maximum I2C read transfer */
#define MAX_READ_SIZE 16

#define SX1509_ADDR 0x3E

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
int32_t sx1509_writeI2C(
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
            .address = SX1509_ADDR,
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
int32_t sx1509_readI2C(
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
            .address = SX1509_ADDR,
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

uint8_t sx1509_set_pin_as_output(uint8_t bank, uint8_t pin)
{
    uint8_t regAddr;
    uint8_t regValue;
    uint8_t res;

    if (bank == BANK_A)
    {
        regAddr = REG_DIR_A;
    }
    else if (bank == BANK_B)
    {
        regAddr = REG_DIR_B;
    }
    else
    {
        return -1; // Invalid bank
    }

    // Read current direction register value
    res = sx1509_readI2C(regAddr, &regValue, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C read error err: %u", res);
        return res;
    }

    // Clear the corresponding bit to set the pin as an output
    regValue &= ~(1 << pin);

    // Write the updated register value back to the SX1509
    res = sx1509_writeI2C(regAddr, &regValue, 1);
    return res;
}

uint8_t sx1509_set_pin_level(uint8_t bank, uint8_t pin, uint8_t level)
{
    uint8_t regAddr;
    uint8_t regValue;
    uint8_t res;

    if (bank == BANK_A)
    {
        regAddr = REG_DATA_A;
    }
    else if (bank == BANK_B)
    {
        regAddr = REG_DATA_B;
    }
    else
    {
        return -1; // Invalid bank
    }

    // Read current data register value
    res = sx1509_readI2C(regAddr, &regValue, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C read error err: %u", res);
        return res;
    }

    // Set or clear the corresponding bit based on the desired level
    if (level)
    {
        regValue |= (1 << pin);
    }
    else
    {
        regValue &= ~(1 << pin);
    }

    // Write the updated register value back to the SX1509
    res = sx1509_writeI2C(regAddr, &regValue, 1);
    return res;
}

uint8_t sx1509_set_all_pins_high(void)
{
    uint8_t res;
    uint8_t allOutput = 0x00; // All pins as output
    uint8_t allHigh = 0xFF;   // All pins high

    // Set all pins of Bank A as output
    res = sx1509_writeI2C(REG_DIR_A, &allOutput, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Set all pins of Bank B as output
    res = sx1509_writeI2C(REG_DIR_B, &allOutput, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Drive all pins of Bank A high
    res = sx1509_writeI2C(REG_DATA_A, &allHigh, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Drive all pins of Bank B high
    res = sx1509_writeI2C(REG_DATA_B, &allHigh, 1);
    return res;
}

uint8_t sx1509_set_all_pins_low(void)
{
    uint8_t res;
    uint8_t allOutput = 0x00; // All pins as output
    uint8_t allLow = 0x00;    // All pins low

    // Set all pins of Bank A as output
    res = sx1509_writeI2C(REG_DIR_A, &allOutput, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Set all pins of Bank B as output
    res = sx1509_writeI2C(REG_DIR_B, &allOutput, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Drive all pins of Bank A low
    res = sx1509_writeI2C(REG_DATA_A, &allLow, 1);
    if (res != I2C_RES_OK)
    {
        LOG(LVL_ERROR, "I2C write error err: %u", res);
        return res;
    }

    // Drive all pins of Bank B low
    res = sx1509_writeI2C(REG_DATA_B, &allLow, 1);
    return res;
}
