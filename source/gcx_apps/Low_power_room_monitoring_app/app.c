/* Copyright (c) 2023 grandcentrix GmbH
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * \file    app.c
 * \brief   Meeting room monitor application
 */

#include <stdlib.h>

#include "api.h"
#include "node_configuration.h"
#include "shared_data.h"
#include "app_scheduler.h"

#include "board.h"
#include "hts221.h"
#include "gpio.h"

#define DEBUG_LOG_MODULE_NAME "MEETING_ROOM_MONITOR_APP"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"

/** Period to send data */
#define DEFAULT_PERIOD_S 60
#define DEFAULT_PERIOD_MS (DEFAULT_PERIOD_S * 1000)

/** Time needed to execute the periodic work, in us */
#define EXECUTION_TIME_US 500

/** Endpoint to change the sending period value */
#define SET_PERIOD_EP 10

#define DATA_EP 1

#define MSG_ID_READING 0

/** Period to send measurements, in ms */
static uint32_t period_ms;

typedef struct __attribute__((packed))
{
    /** periodic message interval in milliseconds */
    uint32_t period_ms;
} payload_periodic_t;

struct env_data
{
    bool temp_to_send;
    int8_t temp_integer;
    uint8_t temp_decimal;
    bool humi_to_send;
    int16_t humidity;
};

static struct env_data environmental_data;

/**
 * @brief   Task to send periodic message.
 * @return  next period
 */

static uint32_t send_data_task(void)
{
    static uint32_t id = 0; // Value to send
    static uint8_t buffer[6];

    buffer[0] = MSG_ID_READING;
    buffer[1] = id;
    buffer[2] = environmental_data.temp_integer;
    buffer[3] = environmental_data.temp_decimal;
    buffer[4] = environmental_data.humidity;
    buffer[5] = (environmental_data.humidity >> 8);

    // Create a data packet to send
    app_lib_data_to_send_t data_to_send;
    data_to_send.bytes = (const uint8_t *)buffer;
    data_to_send.num_bytes = sizeof(buffer);
    data_to_send.dest_address = APP_ADDR_ANYSINK;
    data_to_send.src_endpoint = DATA_EP;
    data_to_send.dest_endpoint = DATA_EP;
    data_to_send.qos = APP_LIB_DATA_QOS_NORMAL;
    data_to_send.delay = 0;
    data_to_send.flags = APP_LIB_DATA_SEND_FLAG_NONE;
    data_to_send.tracking_id = APP_LIB_DATA_NO_TRACKING_ID;

    // Send the data packet
    Shared_Data_sendData(&data_to_send, NULL);

    // Increment value to send
    id++;

    environmental_data.temp_to_send = false;
    environmental_data.humi_to_send = false;
    // Inform the stack that this function should be called again in
    // period_ms microseconds. By returning APP_SCHEDULER_STOP_TASK,
    // the stack won't call this function again.
    return period_ms;
}

/**
 * @brief   Task to send periodic message.
 * @return  next period
 */

static uint32_t trigger_sensors_task(void)
{
    hts221_one_shot();
    return 10000;
}

void hts221_interrupt_handler(uint8_t pin, gpio_event_e event)
{
    float f_decimal;
    float in_temp = hts221_temperature_get();
    environmental_data.temp_integer = (int8_t)in_temp;
    f_decimal = in_temp - environmental_data.temp_integer;
    environmental_data.temp_decimal = (uint8_t)(f_decimal * 100.0f);
    LOG(LVL_DEBUG, "Temperature: %d.%d C", environmental_data.temp_integer, environmental_data.temp_decimal);
    int16_t humidity = hts221_humidity_get();
    LOG(LVL_DEBUG, "Relative Humidty: %d%%", humidity);
    environmental_data.temp_to_send = true;
    environmental_data.humi_to_send = true;
}

static uint32_t init_sensors(void)
{
    hts221_init();
    hts221_enable();
    GPIO_register_for_event(24, GPIO_NOPULL, GPIO_EVENT_HL, 100, hts221_interrupt_handler);
    return APP_SCHEDULER_STOP_TASK;
}

/**
 * \brief   Initialization callback for application
 *
 * This function is called after hardware has been initialized but the
 * stack is not yet running.
 *
 */
void App_init(const app_global_functions_t *functions)
{
    LOG_INIT();
    LOG(LVL_INFO, "App_init");

    // Basic configuration of the node with a unique node address
    if (configureNodeFromBuildParameters() != APP_RES_OK)
    {
        // Could not configure the node
        // It should not happen except if one of the config value is invalid
        return;
    }

    /*
     * Set node operating mode (i.e low-energy or low-latency with autorole)
     * Default is low-energy.
     */
#ifdef ENABLE_LOW_LATENCY_MODE
    lib_settings->setNodeRole(APP_LIB_SETTINGS_ROLE_AUTOROLE_LL);
#endif

    // Set a periodic callback to be called after DEFAULT_PERIOD_MS
    period_ms = DEFAULT_PERIOD_MS;
    App_Scheduler_addTask_execTime(send_data_task,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   EXECUTION_TIME_US);

    App_Scheduler_addTask_execTime(trigger_sensors_task,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   EXECUTION_TIME_US);

    // Enable power for sensors
    nrf_gpio_pin_dir_set(VDD_PWD_CTRL, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(VDD_PWD_CTRL);
    nrf_gpio_pin_set(VDD_PWD_CTRL); // consumes about 70 µA

    App_Scheduler_addTask_execTime(init_sensors, 1000, 50 * 1000);

    // Start the stack
    lib_state->startStack();
}
