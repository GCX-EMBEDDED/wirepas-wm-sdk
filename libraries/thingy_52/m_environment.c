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

#include "m_environment.h"
#include <string.h>
#include <inttypes.h>   // For PRIu32, PRIu8
#include "app_util_platform.h"
#include "drv_humidity_temperature.h"
#include "drv_gas_sensor.h"
#include "drv_ccs811.h"
#include "app_timer.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "macros_common.h"
#define DEBUG_LOG_MODULE_NAME "m_environment"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"


/**
 * \brief   Value to return from task to remove it from the scheduler
 */
#define APP_SCHEDULER_STOP_TASK     ((uint32_t)(-1))



env_config config_env_intervals;
temperature_t temp;
humidity_t humid;

temperature_t get_temperature(void)
{
   return temp;
}

uint8_t get_humidity(void)
{
   return humid;
}

/**@brief Function for converting the temperature sample.
 */
static void temperature_conv_data(float in_temp, temperature_t * p_out_temp)
{  
    float f_decimal;
    p_out_temp->integer = (int8_t)in_temp;
    f_decimal = in_temp - p_out_temp->integer;
    p_out_temp->decimal = (uint8_t)(f_decimal * 100.0f);
    LOG(LVL_DEBUG,"Temperature: %d.%d C", p_out_temp->integer, p_out_temp->decimal);
}


/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, humidity_t * p_out_humid)
{
   *p_out_humid = (uint8_t)humid;
   LOG(LVL_DEBUG,"Relative Humidty: %d%%", humid);
}


/**@brief Humidity sensor event handler.
 */
static void drv_humidity_evt_handler(drv_humidity_temperature_evt_t event)
{  
   LOG(LVL_DEBUG,"drv_humidity_evt_handler has been called");
   float temperature = drv_humidity_temp_get();
   uint16_t humidity = drv_humidity_get();
   calibrate_gas_sensor(humidity, temperature);
}

/**@brief Gas sensor data handler.
 */
/**@brief Gas sensor data handler.
 */
static void drv_gas_data_handler(drv_gas_sensor_data_t const * p_data)
{  
   LOG(LVL_DEBUG,"gas_data_handler: Pointer is NULL");
    if (p_data != NULL)
    {
        LOG(LVL_DEBUG,"gas_data_handler eCO2:, %d, - TVOC:, %d,\r\n", p_data->ec02_ppm,
                                                                      p_data->tvoc_ppb);
    }
}

uint32_t update_temperature(void)
{

   drv_humidity_temperature_enable();
   float temperature = drv_humidity_temp_get();
   temperature_conv_data(temperature, &temp);
   drv_humidity_temperature_sample();
   //drv_humidity_temperature_disable();
   return config_env_intervals.temperature_interval_ms;
}

/**@brief Function for starting humidity sampling.
 */
uint32_t update_humidity(void)
{
    drv_humidity_temperature_enable();
    drv_humidity_temperature_sample();
    uint16_t humidity = drv_humidity_get();
    humidity_conv_data(humidity, &humid);
    drv_humidity_temperature_disable();
   return config_env_intervals.humidity_interval_ms;
}

/**@brief Function for initializing the humidity/temperature sensor
 */
static uint32_t humidity_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{    
    ret_code_t               err_code = NRF_SUCCESS;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_temperature_init_t    init_params =
    {
        .twi_addr            = HTS221_ADDR,
        .pin_int             = HTS_INT,
        .p_twi_instance      = p_twi_instance,
        .p_twi_cfg           = &twi_config,
        .evt_handler         = drv_humidity_evt_handler
    };

    err_code = drv_humidity_temperature_init(&init_params);

    return err_code;
}

static uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = CCS811_ADDR;
    init_params.data_handler   = drv_gas_data_handler;

    err_code = drv_gas_sensor_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

 uint32_t gas_start(void)
{
   uint32_t err_code;
   drv_gas_sensor_mode_t mode;
   mode = DRV_GAS_SENSOR_MODE_60S;
   err_code = drv_gas_sensor_start(mode);
   RETURN_IF_ERROR(err_code);
}

 uint32_t gas_stop(void)
{
    uint32_t err_code;

        //err_code = humidity_temp_stop_for_gas_calibration();
        //RETURN_IF_ERROR(err_code);
    return drv_gas_sensor_stop();
}

/**@brief Sends the sampled humidity and temperature to the gas sensor for calibration.
 *
 * @note Not currently used.
 */
uint32_t calibrate_gas_sensor(uint16_t humid, float temp)
{
    uint32_t err_code;
        uint16_t rh_ppt    = humid * 10;
        int32_t temp_mdeg = (int32_t)(temp * 1000.0f);

        LOG(LVL_DEBUG,"Calibrating gas sensor: humid out %d [ppt], temp out: %d [mdeg C]\r\n", rh_ppt, temp_mdeg);

        err_code = drv_gas_sensor_calibrate_humid_temp(rh_ppt, temp_mdeg);
        LOG(LVL_DEBUG,"drv_gas_sensor_calibrate_humid_temp %" PRIu32 "\n",err_code);
        //RETURN_IF_ERROR(err_code);

        return NRF_SUCCESS;
}

uint32_t m_environment_init(m_environment_init_t * p_params)
{  
    uint32_t err_code;
    NULL_PARAM_CHECK(p_params);

    config_env_intervals.humidity_interval_ms = 2000u;
    config_env_intervals.temperature_interval_ms = 3000u;

    /**@brief Init drivers */
    err_code = humidity_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);
    err_code = gas_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}
