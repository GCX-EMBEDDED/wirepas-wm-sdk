#ifndef __DRIVER_CCS881_H__
#define __DRIVER_CCS881_H__

#include "i2c.h"
#include "board.h"
#include "sx1509.h"

#define CCS_STATUS_REG 0x00
#define MEAS_MODE_REG 0x01
#define ALG_RESULT_DATA 0x02
#define ENV_DATA 0x05
#define THRESHOLDS 0x10
#define BASELINE 0x11
#define HW_ID_REG 0x20
#define HW_ID_REG_VAL 0x81
#define ERROR_ID_REG 0xE0
#define APP_START_REG 0xF4
#define SW_RESET 0xFF
#define CCS_811_ADDRESS 0x5A // CCS811 ADDR pin is logic zero otherwise address would be 0x5B
#define GPIO_WAKE 0x5
#define DRIVE_MODE_IDLE 0x0
#define DRIVE_MODE_1SEC 0x10
#define DRIVE_MODE_10SEC 0x20
#define DRIVE_MODE_60SEC 0x30
#define INTERRUPT_DRIVEN 0x8
#define THRESHOLDS_ENABLED 0x4

#define M_TEMP_OFFSET (25000)        ///< Temperature offset in millidegrees Centigrade.
#define M_RH_PPTH_MAX (1000)         ///< Maximum relative humidity in parts per thousand.
#define M_TEMP_MILLIDEG_MAX (50000)  ///< Maximum temperature in millidegrees Centigrade.
#define M_TEMP_MILLIDEG_MIN (-25000) ///< Minimum temperature in millidegrees Centigrade.

typedef struct
{
    uint16_t eCO2;
    uint16_t TVOC;
} ccs811_alg_result_t;

uint32_t ccs811_pre_init(void);

/**@brief Function to init the ccs811 sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_init(void);

/**@brief Function for verifying the HTS221 WHO_AM_I register.
 *
 * @return 0 If the call was successful*/
uint8_t ccs811_verify(void);

/**@brief Function to get the status of the ccs811 sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_status(void);

/**@brief Function to read the error register of the ccs811 sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_read_error_id(uint8_t *error_id);

/**@brief Function to read the read the eCO2 and TVOC values from the ccs811 sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_read_alg_result(ccs811_alg_result_t *result);

/**@brief Function to activate the interrupt and to trigger the gas sensor measurement once each 1 min.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_start();

/**@brief Function to write the temperature and humidity values to the env_data register of the ccs811 sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t ccs811_set_env_data(uint16_t humid, float temp);

#endif //__DRIVER_CCS881_H__