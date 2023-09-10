#ifndef DRV_SX1509_H__
#define DRV_SX1509_H__

#include <stdio.h>
#include <stdint.h>

#define VDD_PWR_CTRL 30

// SX1509 Registers
#define REG_DIR_B 0x0F  // Pin direction register - bank B
#define REG_DIR_A 0x0E  // Pin direction register - bank A
#define REG_DATA_B 0x11 // Data register - bank B
#define REG_DATA_A 0x10 // Data register - bank A

// SX1509 Pins
#define BANK_A 0
#define BANK_B 1

int32_t sx1509_setPinLevel(uint8_t bank, uint8_t pin, uint8_t level);

int32_t sx1509_setPinAsOutput(uint8_t bank, uint8_t pin);

int32_t sx1509_setAllPinsHigh();

int32_t sx1509_setAllPinsLow();

#endif // DRV_SX1509_H__