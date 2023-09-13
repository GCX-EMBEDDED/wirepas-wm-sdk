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
#include "battery_measurement.h"
#include "sx1509.h"

#define DEBUG_LOG_MODULE_NAME "MEETING_ROOM_MONITOR_APP"

#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"

/** Period to trigger sensors */
#define DEFAULT_MEASURE_PERIOD_S 60
#define DEFAULT_MEASURE_PERIOD_MS (DEFAULT_MEASURE_PERIOD_S * 1000)

/** Max time needed to execute the periodic work, in us */
#define EXECUTION_TIME_US 2500

/** Delay time needed before executing some tasks, in ms */
#define TASK_DELAY_TIME_MS 1000

/** Data end-point */
#define DATA_EP 1

#define MSG_ID_READING 0

enum measurement_state
{
    SM_MEAS_TRIGGER_TEMPERATURE_CONV,
    SM_MEAS_WAIT_FOR_TEMPERATURE_CONV,
    SM_MEAS_WAIT_FOR_GAS_SENSOR_CONV,
};

static enum measurement_state state;

struct env_data
{
    int8_t temp_integer;
    uint8_t temp_decimal;
    int16_t humidity;
    uint8_t movement_counter;
    uint16_t voltage;
};

static struct env_data environmental_data;

/**
 * @brief   Task to send periodic message.
 * @return  Next period
 */
static uint32_t send_data_task(void)
{
    static uint32_t id = 0; // Value to send
    static uint8_t buffer[11];

    buffer[0] = MSG_ID_READING;
    buffer[1] = id;
    buffer[2] = environmental_data.voltage;
    buffer[3] = (environmental_data.voltage >> 8);
    buffer[4] = environmental_data.temp_integer;
    buffer[5] = environmental_data.temp_decimal;
    buffer[6] = 255;        // Const value for now
    buffer[7] = (100 >> 8); // Const value for now
    buffer[8] = environmental_data.humidity;
    buffer[9] = (environmental_data.humidity >> 8);
    buffer[10] = environmental_data.movement_counter;

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
    // Clear motion sensor count
    environmental_data.movement_counter = 0;

    LOG(LVL_DEBUG, "Message has been sent!");

    return APP_SCHEDULER_STOP_TASK;
}

static void read_temperature_and_humidity(void)
{
    float f_decimal;
    float in_temp = hts221_temperature_get();
    environmental_data.temp_integer = (int8_t)in_temp;
    f_decimal = in_temp - environmental_data.temp_integer;
    environmental_data.temp_decimal = (uint8_t)(f_decimal * 100.0f);
    LOG(LVL_DEBUG, "Temperature: %d.%d C", environmental_data.temp_integer, environmental_data.temp_decimal);
    environmental_data.humidity = hts221_humidity_get();
    LOG(LVL_DEBUG, "Relative Humidty: %d%%", environmental_data.humidity);
}

static uint32_t measurement_sm_task(void)
{
    uint32_t next_period = APP_SCHEDULER_STOP_TASK;

    switch (state)
    {
    case SM_MEAS_TRIGGER_TEMPERATURE_CONV:
        // Trigger temperature and humidity sensor conversion
        state = SM_MEAS_WAIT_FOR_TEMPERATURE_CONV;
        if (hts221_one_shot() != 0)
        {
            LOG(LVL_ERROR, "Failed to preform one shot measurement on the hts221 sensor");
        }
        break;

    case SM_MEAS_WAIT_FOR_TEMPERATURE_CONV:
        // Read temperature and humidity sensor
        read_temperature_and_humidity();
        // Write temperature/humidity to gas sensor

        // Trigger gas sensor

        state = SM_MEAS_WAIT_FOR_GAS_SENSOR_CONV;
        // -> in gas sensor interrupt handler, schedule this task again

        // Because the interrupt handler is not implemented yet, reschedule the task here again
        App_Scheduler_addTask_execTime(measurement_sm_task,
                                       APP_SCHEDULER_SCHEDULE_ASAP,
                                       EXECUTION_TIME_US);
        break;

    case SM_MEAS_WAIT_FOR_GAS_SENSOR_CONV:
        // Read gas sensor

        // Read battery voltage
        environmental_data.voltage = battery_measurement_get();
        LOG(LVL_DEBUG, "Battery voltage %lu mV", environmental_data.voltage);

        // Send data
        App_Scheduler_addTask_execTime(send_data_task, APP_SCHEDULER_SCHEDULE_ASAP, EXECUTION_TIME_US);
        state = SM_MEAS_TRIGGER_TEMPERATURE_CONV;

        // Schedule this task in DEFAULT_MEASURE_PERIOD_MS again
        App_Scheduler_addTask_execTime(measurement_sm_task,
                                       DEFAULT_MEASURE_PERIOD_MS,
                                       EXECUTION_TIME_US);
        break;
    default:
        LOG(LVL_ERROR, "Unknown state %d", state);
        break;
    }

    return next_period;
}

void hts221_interrupt_handler(uint8_t pin, gpio_event_e event)
{
    if (state == SM_MEAS_WAIT_FOR_TEMPERATURE_CONV)
    {
        App_Scheduler_addTask_execTime(measurement_sm_task,
                                       APP_SCHEDULER_SCHEDULE_ASAP,
                                       EXECUTION_TIME_US);
    }
}

void hw416_interrupt_handler(uint8_t pin, gpio_event_e event)
{
    ++environmental_data.movement_counter;
    LOG(LVL_DEBUG, "Movement has been detected!, Movement_counter=%u", environmental_data.movement_counter);
}

static uint32_t switch_off_led_task(void)
{
    sx1509_set_pin_level(BANK_B, GREEN_LED, 1);
    return APP_SCHEDULER_STOP_TASK;
}

static uint32_t init_sensors(void)
{
    if (hts221_init() == 0)
    {
        if (hts221_enable() != 0)
        {
            LOG(LVL_ERROR, "Failed to enable the hts221 sensor");
        }
    }
    else
    {
        LOG(LVL_ERROR, "Failed to init the hts221 sensor");
    }
    if (GPIO_register_for_event(24, GPIO_NOPULL, GPIO_EVENT_HL, 10, hts221_interrupt_handler) != GPIO_RES_OK)
    {
        LOG(LVL_ERROR, "Failed to register event handler for hts221");
    }
    if (GPIO_register_for_event(4, GPIO_NOPULL, GPIO_EVENT_LH, 10, hw416_interrupt_handler) != GPIO_RES_OK)
    {
        LOG(LVL_ERROR, "Failed to register event handler for hw416");
    }
    if (battery_measurement_init() != 0)
    {
        LOG(LVL_ERROR, "Failed to init battery measurement module");
    }
    // Read the hts221 sensor to clear the flag of data-ready and to activate the interrupt handler by the next one-shot measurement
    read_temperature_and_humidity();
    state = SM_MEAS_TRIGGER_TEMPERATURE_CONV;
    App_Scheduler_addTask_execTime(measurement_sm_task,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   EXECUTION_TIME_US);

    // Configure the green led pin as output
    if (sx1509_set_pin_as_output(BANK_B, GREEN_LED) != 0)
    {
        LOG(LVL_ERROR, "Error while configuring the pin of the green led");
    }
    // Turn on the green led by setting the led pin to low
    if (sx1509_set_pin_level(BANK_B, GREEN_LED, 0) != 0)
    {
        LOG(LVL_ERROR, "Error while setting the pin of green led to low");
    }
    // Call a task to switch off the led after one second
    App_Scheduler_addTask_execTime(switch_off_led_task, TASK_DELAY_TIME_MS, EXECUTION_TIME_US);

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
    LOG(LVL_INFO, "MEETING_ROOM_MONITOR_APP INIT");

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

    // Enable power for sensors
    nrf_gpio_pin_dir_set(VDD_PWD_CTRL, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_cfg_output(VDD_PWD_CTRL);
    nrf_gpio_pin_set(VDD_PWD_CTRL); // Consumes about 70 µA

    App_Scheduler_addTask_execTime(init_sensors, TASK_DELAY_TIME_MS, EXECUTION_TIME_US);

    // Start the stack
    lib_state->startStack();
}
