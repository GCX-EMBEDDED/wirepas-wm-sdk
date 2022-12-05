/* Copyright 2021 Wirepas Ltd. All Rights Reserved.
 *
 * See file LICENSE.txt for full license details.
 *
 */

/**
 * @file
 *
 * Board definition for the
 * <a href="https://www.silabs.com/development-tools/thunderboard/thunderboard-bg22-kit">Silabs Thunderboard BG22</a>
 */
#ifndef BOARD_SILABSBRD4184A_BOARD_H_
#define BOARD_SILABSBRD4184A_BOARD_H_

// VCOM port only supports 115200 baudrate
// This speed will be used independently of UART_BAUDRATE flag value
#define BOARD_USART_FORCE_BAUDRATE      115200

// List of GPIO pins
#define BOARD_GPIO_PIN_LIST            {{GPIO_PORTB, 0}, /* PB00 */\
                                        {GPIO_PORTB, 1}, /* PB01 */\
                                        {GPIO_PORTA, 5}, /* PA05. usart tx pin */\
                                        {GPIO_PORTA, 6}} /* PA06. usart rx pin */

// User friendly name for GPIOs (IDs mapped to the BOARD_GPIO_PIN_LIST table)
#define BOARD_GPIO_ID_LED0              0  // mapped to pin PB00
#define BOARD_GPIO_ID_BUTTON0           1  // mapped to pin PB01
#define BOARD_GPIO_ID_USART_TX          2  // mapped to pin PA05
#define BOARD_GPIO_ID_USART_RX          3  // mapped to pin PA06

// List of LED IDs
#define BOARD_LED_ID_LIST              {BOARD_GPIO_ID_LED0}

// Active high polarity for LEDs
#define BOARD_LED_ACTIVE_LOW            false

// List of button IDs
#define BOARD_BUTTON_ID_LIST           {BOARD_GPIO_ID_BUTTON0}

// Active low polarity for buttons
#define BOARD_BUTTON_ACTIVE_LOW         true

// Board has external pull-up for buttons
#define BOARD_BUTTON_INTERNAL_PULL      false


#endif /* BOARD_SILABSBRD4184A_BOARD_H_ */
