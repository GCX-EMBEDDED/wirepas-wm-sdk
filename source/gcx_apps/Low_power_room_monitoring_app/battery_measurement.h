#ifndef APP_BATTERY_MEASREMENT_H__
#define APP_BATTERY_MEASREMENT_H__

#include <stdint.h>

/**
 * @brief   Initialize voltage measurement mechanisms.
 *
 * @return 0 If the call was successful
 */
uint8_t battery_measurement_init(void);

/**
 * @brief   Measure battery voltage.
 *
 * @return  Voltage in millivolts or an error code
 */
uint16_t battery_measurement_get(void);

#endif // APP_BATTERY_MEASREMENT_H__