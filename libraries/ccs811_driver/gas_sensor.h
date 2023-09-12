/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file Gas sensor
 *
 * @defgroup gas_sensor Gas sensor
 * @{
 * @ingroup drivers
 * @brief Digital gas sensor API.
 *
 */

#ifndef __DRV_GAS_SENSOR_H__
#define __DRV_GAS_SENSOR_H__

#include "mcu.h"
#include "ccs811.h"
#include "ccs811_bitfields.h"
#include <stdbool.h>


#define SX_CCS_RESET                        11
#define IOEXT_PIN11_SYSTEM_DEFAULT_CFG      SX_PIN_INPUT_PULLDOWN

#define SX_CCS_WAKE                         12
#define IOEXT_PIN12_SYSTEM_DEFAULT_CFG      SX_PIN_INPUT_PULLDOWN

/* Bits 3..2 : Pull configuration */
#define GPIO_PIN_CNF_PULL_Pos (2UL) /*!< Position of PULL field. */
#define GPIO_PIN_CNF_PULL_Msk (0x3UL << GPIO_PIN_CNF_PULL_Pos) /*!< Bit mask of PULL field. */
#define GPIO_PIN_CNF_PULL_Disabled (0UL) /*!< No pull */
#define GPIO_PIN_CNF_PULL_Pulldown (1UL) /*!< Pull down on pin */
#define GPIO_PIN_CNF_PULL_Pullup (3UL) /*!< Pull up on pin */

/* Bits 17..16 : When In task mode: Operation to be performed on output when OUT[n] task is triggered. When In event mode: Operation on input that shall trigger IN[n] event. */
#define GPIOTE_CONFIG_POLARITY_Pos (16UL) /*!< Position of POLARITY field. */
#define GPIOTE_CONFIG_POLARITY_Msk (0x3UL << GPIOTE_CONFIG_POLARITY_Pos) /*!< Bit mask of POLARITY field. */
#define GPIOTE_CONFIG_POLARITY_None (0UL) /*!< Task mode: No effect on pin from OUT[n] task. Event mode: no IN[n] event generated on pin activity. */
#define GPIOTE_CONFIG_POLARITY_LoToHi (1UL) /*!< Task mode: Set pin from OUT[n] task. Event mode: Generate IN[n] event when rising edge on pin. */
#define GPIOTE_CONFIG_POLARITY_HiToLo (2UL) /*!< Task mode: Clear pin from OUT[n] task. Event mode: Generate IN[n] event when falling edge on pin. */
#define GPIOTE_CONFIG_POLARITY_Toggle (3UL) /*!< Task mode: Toggle pin from OUT[n]. Event mode: Generate IN[n] when any change on pin. */
typedef uint32_t ret_code_t;
/**@brief Struct for holding the measurement results.
 */
typedef drv_ccs811_alg_result_t drv_gas_sensor_data_t;

#define CCS_INT                             22


/**@brief Gas sensor measurement intervals.
 */
typedef enum
{
    DRV_GAS_SENSOR_MODE_250MS,
    DRV_GAS_SENSOR_MODE_1S,
    DRV_GAS_SENSOR_MODE_10S,
    DRV_GAS_SENSOR_MODE_60S
} drv_gas_sensor_mode_t;

// typedef enum
// {
//   NRF_GPIOTE_POLARITY_LOTOHI = GPIOTE_CONFIG_POLARITY_LoToHi,       ///<  Low to high.
//   NRF_GPIOTE_POLARITY_HITOLO = GPIOTE_CONFIG_POLARITY_HiToLo,       ///<  High to low.
//   NRF_GPIOTE_POLARITY_TOGGLE = GPIOTE_CONFIG_POLARITY_Toggle        ///<  Toggle.
// } nrf_gpiote_polarity_t;
/**@brief Input pin configuration. */
typedef struct
{
    nrf_gpiote_polarity_t sense;      /**< Transition that triggers interrupt. */
    nrf_gpio_pin_pull_t   pull;       /**< Pulling mode. */
    bool                  is_watcher; /**< True when the input pin is tracking an output pin. */
    bool                  hi_accuracy;/**< True when high accuracy (IN_EVENT) is used. */
} nrf_drv_gpiote_in_config_t;


/**@brief Gas sensor init struct.
 */
// typedef struct
// {
//     nrf_drv_twi_t         const * p_twi_instance;   ///< The TWI instance.
//     nrf_drv_twi_config_t  const * p_twi_cfg;        ///< TWI configuraion.
//     uint8_t                       twi_addr;         ///< TWI address on bus.
//     drv_gas_sensor_data_handler_t data_handler;     ///< Handler to be called when data capture has finished.
// } drv_gas_init_t;

/**@brief Gas sensor driver event handler callback type.
 */
typedef void (*drv_gas_sensor_data_handler_t)(drv_gas_sensor_data_t const * p_evt);

typedef uint32_t nrf_drv_gpiote_pin_t;


/**@brief Gas sensor init struct.
 */
typedef struct
{
   //  nrf_drv_twi_t         const * p_twi_instance;   ///< The TWI instance.
   //  nrf_drv_twi_config_t  const * p_twi_cfg;        ///< TWI configuraion.
   //  uint8_t                       twi_addr;         ///< TWI address on bus.
    drv_gas_sensor_data_handler_t data_handler;     ///< Handler to be called when data capture has finished.
} drv_gas_init_t;


/**@brief Function for calibrating the gas sensor based on the ambient humidity and temperature.
 *
 * @param[in] rh_ppth       Relative humidity in parts per thousand (ppt).
 * @param[in] temp_mdeg     Temperature in millidegrees Centigrade.
 *
 * @return NRF_SUCCESS  If the call was successful.
 * @return Other codes from the underlying drivers.
 */
ret_code_t drv_gas_sensor_calibrate_humid_temp(uint16_t rh_ppth, int32_t temp_mdeg);


/**@brief Function for starting data acquisition from the the gas sensor.
 *
 * @param[in] mode   The given mode (frequency) of sensor reads.
 *
 * @return NRF_SUCCESS  If the call was successful.
 * @return Other codes from the underlying drivers.
 */
ret_code_t drv_gas_sensor_start(drv_gas_sensor_mode_t mode);

/**@brief Function for stopping data acquisition from the the gas sensor.
 *
 * @return NRF_SUCCESS              If the call was successful.
 * @return NRF_ERROR_NOT_SUPPORTED  The mode is currently not supported.
 * @return NRF_ERROR_INVALID_PARAM  Invalid parameters supplied.
 * @return NRF_ERROR_INVALID_STATE  The sensor is in an invalid state.
 * @return Other codes from the underlying drivers.
 */
ret_code_t drv_gas_sensor_stop(void);

/**@brief Function for initializing the gas sensor.
 *
 * @param[in] p_init    Pointer with configuration parameters.
 *
 * @return NRF_SUCCESS  If the call was successful.
 * @return Other codes from the underlying drivers.
 */
//ret_code_t drv_gas_sensor_init(drv_gas_init_t * p_init);

uint32_t gas_sensor_init();
uint32_t calibrate_gas_sensor(uint16_t humid, float temp);
uint32_t gas_start(void);

/**@brief Function for getting the results of eCO2 and TVOC from the gas sensor.
 * @return drv_ccs811_alg_result_descr_t struct that contains eCO2 and TVOC values.
 */
drv_ccs811_alg_result_t get_gas_sensor_values();

uint32_t gas_stop(void);

#endif

/** @} */
