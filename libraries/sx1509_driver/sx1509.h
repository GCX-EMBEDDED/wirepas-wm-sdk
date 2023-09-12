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

#define GREEN_LED 5 // Green color of the lightwell LEDs SXIO5
#define BLUE_LED 6  // Blue color of the lightwell LEDs  SXIO6
#define RED_LED 7   // Red color of the lightwell LEDs   SXIO7

/**@brief Function to set a pin as output.
 *
 * @return 0 If the call was successful
 */
uint8_t sx1509_set_pin_as_output(uint8_t bank, uint8_t pin);

/**@brief Function to set the level (High or Low) of an output pin.
 *
 * @return 0 If the call was successful
 */
uint8_t sx1509_set_pin_level(uint8_t bank, uint8_t pin, uint8_t level);

/**@brief Function to set all output pins of the io-extender to high
 *
 * @return 0 If the call was successful
 */
uint8_t sx1509_set_all_pins_high(void);

/**@brief Function to set all output pins of the io-extender to low
 *
 * @return 0 If the call was successful
 */
uint8_t sx1509_set_all_pins_low(void);

#endif // DRV_SX1509_H__