/* Copyright (c) 2023 grandcentrix GmbH
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>

#include "api.h"
#include "node_configuration.h"
#include "shared_data.h"
#include "app_scheduler.h"
#include "hal_api.h"

#include "app_util_platform.h"
#include "nrf_delay.h"
#include "twi_manager.h"
#include "pca20020.h"

#include "m_environment.h"

#define DEBUG_LOG_MODULE_NAME "MEETING_ROOM_MONITOR_APP"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"

#define MSG_ID_READING 0

/** Period to send data */
#define DEFAULT_PERIOD_S 60
#define DEFAULT_PERIOD_MS (DEFAULT_PERIOD_S * 1000)

/** Time needed to execute the periodic work, in us */
#define EXECUTION_TIME_US 2500

#define DATA_EP 1

/** Period to send measurements, in ms */
static uint32_t period_ms;

typedef struct __attribute__((packed))
{
    /** periodic message interval in milliseconds */
    uint32_t period_ms;
} payload_periodic_t;

static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
static void board_init(void);
bool calibrated_gas_sensor = false;

/**
 * @brief   Task to send periodic message.
 * @return  next period
 */
static uint32_t send_data_task(void)
{
    static uint8_t id = 0; // Value to send
    static uint8_t buffer[6];

    uint16_t voltage = Mcu_voltageGet();
    LOG(LVL_DEBUG, "Battery voltage %lu mV", voltage);

    update_temperature();
    temperature_t temperature =  get_temperature();

    if(calibrated_gas_sensor == false)
    {
    //float temperature = drv_humidity_temp_get();
    //uint16_t humidity = drv_humidity_get();
    //calibrate_gas_sensor(humidity, 25);
    gas_start();
    calibrated_gas_sensor = true;
    }
    get_gas_sensor_values();
    //get_gas_sensor_data();

    buffer[0] = MSG_ID_READING;
    buffer[1] = id;
    buffer[2] = voltage;
    buffer[3] = (voltage >> 8);
    buffer[4] = temperature.integer;
    buffer[5] = temperature.decimal;

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

    // Inform the stack that this function should be called again in
    // period_ms microseconds. By returning APP_SCHEDULER_STOP_TASK,
    // the stack won't call this function again.
    return period_ms;
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
    board_init();
    // APP_IRQ_PRIORITY_THREAD = 15
    twi_manager_init(APP_IRQ_PRIORITY_THREAD);
    m_environment_init_t     env_params;
    env_params.p_twi_instance = &m_twi_sensors;
    m_environment_init(&env_params);
    /* Initialize voltage measurement. */
    Mcu_voltageInit();

    // Set a periodic callback to be called after DEFAULT_PERIOD_MS
    period_ms = DEFAULT_PERIOD_MS;
    App_Scheduler_addTask_execTime(send_data_task,
                                   APP_SCHEDULER_SCHEDULE_ASAP,
                                   EXECUTION_TIME_US);

    // Start the stack
    lib_state->startStack();
}


static void board_init(void)
{
    drv_ext_gpio_init_t ext_gpio_init;
    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = 3
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg      = &twi_config
    };

    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    support_func_configure_io_startup(&ext_gpio_init);


    nrf_delay_ms(100);
}