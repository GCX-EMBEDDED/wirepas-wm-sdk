#ifndef __DRIVER_HTS221_H__
#define __DRIVER_HTS221_H__

#include "i2c.h"
#include "board.h"

/**@brief Device identification register. */
#define WHO_AM_I_REG 0x0F
#define WHO_AM_I_VAL 0xBC

/**@brief Humidity and temperature resolution/average mode register. */
#define AV_CONF_REG 0x10
#define AV_CONF_REG_DEFAULT 0x1B

/**@brief Bitmasks for AVGT. */
#define AV_CONF_REG_AVGT_Pos 3
#define AV_CONF_REG_AVGT_Msk (7 << AV_CONF_REG_AVGT_Pos)
#define AV_CONF_REG_AVGT_2 0
#define AV_CONF_REG_AVGT_4 1
#define AV_CONF_REG_AVGT_8 2
#define AV_CONF_REG_AVGT_16 3
#define AV_CONF_REG_AVGT_32 4
#define AV_CONF_REG_AVGT_64 5
#define AV_CONF_REG_AVGT_128 6
#define AV_CONF_REG_AVGT_256 7

/**@brief Bitmasks for AVGH. */
#define AV_CONF_REG_AVGH_Pos 0
#define AV_CONF_REG_AVGH_Msk (7 << AV_CONF_REG_AVGH_Pos)
#define AV_CONF_REG_AVGH_4 0
#define AV_CONF_REG_AVGH_8 1
#define AV_CONF_REG_AVGH_16 2
#define AV_CONF_REG_AVGH_32 3
#define AV_CONF_REG_AVGH_64 4
#define AV_CONF_REG_AVGH_128 5
#define AV_CONF_REG_AVGH_256 6
#define AV_CONF_REG_AVGH_512 7

/**@brief Control register 1. */
#define CTRL_REG1 0x20
#define CTRL_REG1_DEFAULT 0x00

/**@brief Bitmasks for PD. */
#define CTRL_REG1_PD_Pos 7
#define CTRL_REG1_PD_Msk (1 << CTRL_REG1_PD_Pos)
#define CTRL_REG1_PD_PowerDown 0
#define CTRL_REG1_PD_Active 1

/**@brief Bitmasks for BDU. */
#define CTRL_REG1_BDU_Pos 2
#define CTRL_REG1_BDU_Msk (1 << CTRL_REG1_BDU_Pos)
#define CTRL_REG1_BDU_Continuous 0
#define CTRL_REG1_BDU_Set 1

/**@brief Bitmasks for ODR. */
#define CTRL_REG1_ODR_Pos 0
#define CTRL_REG1_ODR_Msk (3 << CTRL_REG1_ODR_Pos)
#define CTRL_REG1_ODR_OneShot 0
#define CTRL_REG1_ODR_1Hz 1
#define CTRL_REG1_ODR_7Hz 2
#define CTRL_REG1_ODR_12_5Hz 3

/**@brief Control register 2. */
#define CTRL_REG2 0x21
#define CTRL_REG2_DEFAULT 0x00

/**@brief Bitmasks for BOOT. */
#define CTRL_REG2_BOOT_Pos 7
#define CTRL_REG2_BOOT_Msk (1 << CTRL_REG2_BOOT_Pos)
#define CTRL_REG2_BOOT_Normal 0
#define CTRL_REG2_BOOT_Reboot 1

/**@brief Bitmasks for HEATER. */
#define CTRL_REG2_HEATER_Pos 1
#define CTRL_REG2_HEATER_Msk (1 << CTRL_REG2_HEATER_Pos)
#define CTRL_REG2_HEATER_Disable 0
#define CTRL_REG2_HEATER_Enable 1

/**@brief Bitmasks for ONE_SHOT. */
#define CTRL_REG2_ONE_SHOT_Pos 0
#define CTRL_REG2_ONE_SHOT_Msk (1 << CTRL_REG2_ONE_SHOT_Pos)
#define CTRL_REG2_ONE_SHOT_WaitingForStart 0
#define CTRL_REG2_ONE_SHOT_Start 1

/**@brief Control register 3. */
#define CTRL_REG3 0x22
#define CTRL_REG3_DEFAULT 0x00

/**@brief Bitmasks for DRDY_H_L. */
#define CTRL_REG3_DRDY_H_L_Pos 7
#define CTRL_REG3_DRDY_H_L_Msk (1 << CTRL_REG3_DRDY_H_L_Pos)
#define CTRL_REG3_DRDY_H_L_ActiveHigh 0
#define CTRL_REG3_DRDY_H_L_ActiveLow 1

/**@brief Bitmasks for PP_OD. */
#define CTRL_REG3_PP_OD_Pos 6
#define CTRL_REG3_PP_OD_Msk (1 << CTRL_REG3_PP_OD_Pos)
#define CTRL_REG3_PP_OD_PushPull 0
#define CTRL_REG3_PP_OD_OpenDrain 1

/**@brief Bitmasks for DRDY. */
#define CTRL_REG3_DRDY_Pos 2
#define CTRL_REG3_DRDY_Msk (1 << CTRL_REG3_DRDY_Pos)
#define CTRL_REG3_DRDY_Disable 0
#define CTRL_REG3_DRDY_Enable 1

/**@brief Status register. */
#define STATUS_REG 0x27

/**@brief Bitmasks for H_DA. */
#define STATUS_REG_H_DA_Pos 1
#define STATUS_REG_H_DA_Msk (1 << STATUS_REG_H_DA_Pos)
#define STATUS_REG_H_DA_NotAvail 0
#define STATUS_REG_H_DA_Avail 1

/**@brief Bitmasks for T_DA. */
#define STATUS_REG_T_DA_Pos 0
#define STATUS_REG_T_DA_Msk (1 << STATUS_REG_T_DA_Pos)
#define STATUS_REG_T_DA_NotAvail 0
#define STATUS_REG_T_DA_Avail 1

/**@brief Relative humidity data (LSB). */
#define HUMIDITY_OUT_L_REG 0x28

/**@brief Relative humidity data (MSB). */
#define HUMIDITY_OUT_H_REG 0x29

/**@brief Temperature data (LSB). */
#define TEMP_OUT_L_REG 0x2A

/**@brief Temperature data (MSB). */
#define TEMP_OUT_H_REG 0x2B

#define CALIBRATION_REGS 0x30
#define CALIBRATION_REGS_NUM 16

/**@brief Configuration struct for HTS221 humidity sensor.
 */
typedef struct
{
    uint8_t av_conf;
    uint8_t ctrl_reg1;
    uint8_t ctrl_reg2;
    uint8_t ctrl_reg3;
} hts221_cfg_t;

/**@brief Calibration struct.
 */
typedef struct
{
    uint8_t H0_rH_x2;
    uint8_t H1_rH_x2;
    uint16_t T0_degC_x8;
    uint16_t T1_degC_x8;
    int16_t H0_T0_OUT;
    int16_t H1_T0_OUT;
    int16_t T0_OUT;
    int16_t T1_OUT;
} hts221_calib_t;

/**@brief Hts221 driver event types.
 */
typedef enum
{
    HTS221_EVT_DATA, /**<Converted value ready to be read.*/
    HTS221_EVT_ERROR /**<Hardware error on the communication bus.*/
} hts221_evt_t;

/**@brief Hts221 driver event handler callback type.
 */
typedef void (*hts221_evt_handler_t)(hts221_evt_t evt);

/**@brief Function for initializing the hts221 driver.
 *
 * @return 0 If the call was successful
 */
uint8_t hts221_init(void);

/**@brief Function for enabling the sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t hts221_enable(void);

/**@brief Function for disabling the sensor.
 *
 * @return 0 If the call was successful
 */
uint8_t hts221_disable(void);

/**@brief Function for verifying the HTS221 WHO_AM_I register.
 *
 * @return 0 If the call was successful*/
uint8_t hts221_verify(void);

/**@brief Function for getting the sensor calibration data.
 *
 * @param[out]   p_calib   Calibration values.
 *
 * @return 0 If the call was successful*/
uint8_t hts221_calib_get(hts221_calib_t *p_calib);

/**@brief Function for configuring the HTS221 sensor according to the specified configuration.
 *
 * @param[in]   p_cfg   Pointer to the sensor configuration.
 *
 * @return 0 If the call was successful
 */
uint8_t hts221_cfg_set(hts221_cfg_t *const p_cfg);

/**@brief Function for getting the humidity data.
 *
 * @return Humidity value fetched from humidity registers
 */
int16_t hts221_humidity_get(void);

/**@brief Function for getting the temperature data.
 *
 * @return Temperature value fetched from temperature registers
 **/
float hts221_temperature_get(void);

/**@brief Function for starting one shot conversion.
 *
 * @return  0 If the call was successful
 */
uint8_t hts221_one_shot(void);

#endif //__DRIVER_HTS221_H__