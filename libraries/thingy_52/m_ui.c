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

#include "m_ui.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "drv_ext_light.h"
#include "nrf_drv_gpiote.h"
#include "pca20020.h"
#include <stdlib.h>
#include <string.h>

#define NRF_LOG_MODULE_NAME "m_ui          "
#include "macros_common.h"
#include "nrf_log.h"

#define DEBUG_LOG_MODULE_NAME "EVAL_APP"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"

uint32_t m_ui_init(m_ui_init_t *p_params)
{

    static drv_sx1509_cfg_t sx1509_cfg;
    drv_ext_light_init_t led_init;
    static const drv_ext_light_conf_t led_conf[DRV_EXT_LIGHT_NUM] = DRV_EXT_LIGHT_CFG;

    static const nrf_drv_twi_config_t twi_config = {
        .scl = TWI_SCL, .sda = TWI_SDA, .frequency = NRF_TWI_FREQ_100K, .interrupt_priority = APP_IRQ_PRIORITY_LOW};

    sx1509_cfg.twi_addr = SX1509_ADDR;
    sx1509_cfg.p_twi_instance = p_params->p_twi_instance;
    sx1509_cfg.p_twi_cfg = &twi_config;

    led_init.p_light_conf = led_conf;
    led_init.num_lights = DRV_EXT_LIGHT_NUM;
    led_init.clkx_div = DRV_EXT_LIGHT_CLKX_DIV_8;
    led_init.p_twi_conf = &sx1509_cfg;
    led_init.resync_pin = SX_RESET;

    drv_ext_light_init(&led_init, false);

    //(void)drv_ext_light_on(DRV_EXT_RGB_LED_SENSE);
    //(void)drv_ext_light_on(DRV_EXT_RGB_LED_LIGHTWELL);

    drv_ext_light_rgb_intensity_t color;

    color.r = 255;
    color.g = 150;
    color.b = 50;
    (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_SENSE, &color);
    (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_LIGHTWELL, &color);

    return NRF_SUCCESS;
}

// Create a function to set the leds
