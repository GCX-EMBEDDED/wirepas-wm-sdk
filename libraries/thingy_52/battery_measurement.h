
#ifndef APP_BATTERY_MEASREMENT_H__
#define APP_BATTERY_MEASREMENT_H__

#include <stdint.h>

/**
 * @brief   Initialize voltage measurement mechanisms.
 */
void battery_measurement_init(void);

/**
 * @brief   Measure battery voltage. 
 *
 * @return  Voltage in millivolts
 *
 */
uint16_t battery_measurement_get(void);


#endif // APP_BATTERY_MEASREMENT_H__