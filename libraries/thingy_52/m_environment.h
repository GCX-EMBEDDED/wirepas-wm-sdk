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

/** @file Environment module
 *
 * @defgroup m_environment Environment
 * @{
 * @ingroup modules
 * @brief Environment module API.
 *
 */

#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "nrf_drv_twi.h"
#include <stdbool.h>
#include <stdint.h>

#define PACKED(TYPE) TYPE __attribute__((packed))

/**@brief Initialization parameters. */
typedef struct
{
    const nrf_drv_twi_t *p_twi_instance;
} m_environment_init_t;

typedef PACKED(struct {
    uint32_t temperature_interval_ms;
    uint32_t humidity_interval_ms;
}) env_config;

typedef PACKED(struct {
    int8_t integer;
    uint8_t decimal;
}) temperature_t;

typedef uint8_t humidity_t;

uint32_t update_temperature(void);

uint32_t update_humidity(void);

temperature_t get_temperature(void);

uint8_t get_humidity(void);

/**@brief Function for initializing the environment module.
 *
 * @param[in] p_handle  Pointer to the location to store the service handle.
 * @param[in] p_params  Pointer to the init parameters.
 *
 * @retval NRF_SUCCESS  If initialization was successful.
 */
uint32_t m_environment_init(m_environment_init_t *p_params);

uint32_t calibrate_gas_sensor(uint16_t humid, float temp);

uint32_t gas_start(void);

uint32_t gas_stop(void);

#endif

/** @} */
