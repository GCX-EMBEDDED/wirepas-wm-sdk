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

/** @file UI module
 *
 * @defgroup m_ui User interface
 * @{
 * @ingroup modules
 * @brief User interface module API.
 *
 */

#ifndef __M_UI_H__
#define __M_UI_H__

#include "nrf_drv_twi.h"

/** @brief TWI configuraion.
 */
typedef struct
{
    nrf_drv_twi_t const *p_twi_instance;
} m_ui_init_t;

/**@brief Function for initializing all UI components (Buttons and LEDs).
 *
 * @param[in] p_params  Initialization parameters.
 *
 * @retval NRF_SUCCESS      Operation was successful.
 * @retval NRF_ERROR_NULL   NULL pointer supplied.
 * @retval Other codes from the underlying drivers.
 */
ret_code_t m_ui_init(m_ui_init_t *p_params);

/**@brief Function for setting the RGB value of an LED.
 *
 * @param[in]   r   Red intensity (0 to 255).
 * @param[in]   g   Green intensity (0 to 255).
 * @param[in]   b   Blue intensity (0 to 255).
 *
 * @note In Breathe or One-shot mode, the intensity will be set via a separate intensity variable.
 * The values entered in these two modes will be treated as binary (Boolean) for each color.
 *
 * @retval NRF_SUCCESS      Operation was successful.
 * @retval Other codes from the underlying drivers.
 */
ret_code_t m_ui_set_color_led_lightwell(uint8_t red, uint8_t green, uint8_t blue);

#endif /*__THINGY_UI_H__*/

/** @} */
