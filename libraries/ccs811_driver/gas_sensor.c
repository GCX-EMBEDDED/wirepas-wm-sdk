
#include "gas_sensor.h"
//#include "drv_ext_gpio.h"
//#include "nrf_drv_gpiote.h"
// #include "nrf_delay.h"
// #include "app_scheduler.h"
//#include "sdk_errors.h"
#include "ccs811.h"
#include "ccs811_bitfields.h"
#include "sx1509.h"
//#define  NRF_LOG_MODULE_NAME "drv_gas_sensor"
//#include "nrf_log.h"
//#include "macros_common.h"
#include <inttypes.h>   // For PRIu32, PRIu8
#include <stdint.h>

#define DEBUG_LOG_MODULE_NAME "drv_gas_sensor"
/** To activate logs, configure the following line with "LVL_INFO". */
#define DEBUG_LOG_MAX_LEVEL LVL_DEBUG

#include "debug_log.h"


#define RETURN_IF_ERROR(err_code)           \
if ((err_code) != SUCCESS)                  \
{                                           \
    LOG(LVL_DEBUG, "return_if_error");      \
                                            \
    return (err_code);                      \
}

#define SUCCESS 0
#define ERROR 1
#define ERROR_NOT_FOUND   5
#define ERROR_NOT_SUPPORTED 6
#define ERROR_INVALID_PARAM 7
#define ERROR_INVALID_STATE 8

#define GPIO_PIN_PULLUP 3
#define GPIOTE_POLARITY_TOGGLE 3

#define GAS_SENSOR_ID                       (0x81)   ///< HW ID of the gas sensor.
#define GAS_SENSOR_TDWAKE_ENABLE_DELAY_US     (20)   ///< minimum time between sensor I2C accesses.
#define GAS_SENSOR_TAWAKE_ENABLE_DELAY_US     (50)   ///< us for sensor to become active after nWAKE signal.
#define GAS_SENSOR_PWR_ON_DELAY_MS            (30)   ///< ms for sensor to become active after pwr on. NB: May be up tp 70 ms for first boot after gas sensor firmware download. 20 ms from datasheet + margin.

#define SX_CCS_PWR_CTRL                 10
#define IOEXT_PIN10_SYSTEM_DEFAULT_CFG  SX_PIN_OUTPUT_SET

#define CCS_INT                             22

/**@brief Reads the ERROR_ID register from the gas sensor.
 */
#define CCS811_PRINT_IF_SENSOR_ERROR                                                           \
{                                                                                              \
    uint8_t err_id;                                                                            \
    if (drv_ccs811_err_id_get(&err_id))                                                        \
    {                                                                                          \
        LOG(LVL_DEBUG,"Error from CCS811, could not read error ID. line: %d \r\n",  __LINE__); \
    }                                                                                          \
                                                                                               \
    if (err_id)                                                                                \
    {                                                                                          \
        LOG(LVL_DEBUG,"Error from CCS811, code: 0x%x, line: %d \r\n", err_id, __LINE__);       \
    }                                                                                          \
}

static __inline ret_code_t ccs811_open(void);
static __inline ret_code_t ccs811_close(void);
static ret_code_t hw_id_verify(void);
static ret_code_t valid_app_verify(void);
static ret_code_t io_pins_init(void);
static ret_code_t io_pins_init(void);
static drv_ccs811_cfg_t              m_drv_ccs811_cfg = {0}; ///< Driver configuration.
static bool                          m_started = false;      ///< Indicates if the gas sensor is started.

uint32_t drv_ext_gpio_pin_set(uint32_t pin_number);
uint32_t drv_ext_gpio_pin_clear(uint32_t pin_number);
uint32_t drv_ext_gpio_cfg_output(uint32_t pin_number);
ret_code_t drv_gas_sensor_init();
ret_code_t drv_gas_sensor_stop(void);
ret_code_t drv_gas_sensor_start(drv_gas_sensor_mode_t mode);
ret_code_t drv_gas_sensor_calibrate_humid_temp(uint16_t rh_ppth, int32_t temp_mdeg);

uint32_t gas_sensor_init();
uint32_t calibrate_gas_sensor(uint16_t humid, float temp);
uint32_t gas_start(void);
drv_ccs811_alg_result_t get_gas_sensor_values();
uint32_t gas_stop(void);

volatile uint32_t counter;

#define CYCLES_PER_US ((uint32_t) 100)

static void nrf_delay_ms(uint32_t number_of_ms)
{
    for (counter = 0; counter <= number_of_ms * CYCLES_PER_US * 1000; counter++);
}

static void nrf_delay_us(uint32_t number_of_us)
{
    for (counter = 0; counter <= number_of_us * CYCLES_PER_US; counter++);
}


/**@brief Open gas sensor TWI interface.
 */
 static __inline ret_code_t ccs811_open(void)
{
    ret_code_t err_code;
    
    nrf_delay_us(GAS_SENSOR_TDWAKE_ENABLE_DELAY_US);

    err_code = drv_ext_gpio_pin_clear(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    nrf_delay_us(GAS_SENSOR_TAWAKE_ENABLE_DELAY_US);  // Delay for gas sensor to become active according to datasheet.

    return drv_ccs811_open(&m_drv_ccs811_cfg);

 }

/**@brief Close gas sensor TWI interface.
 */
static __inline ret_code_t ccs811_close(void)
{
    ret_code_t err_code;

    err_code = drv_ccs811_close();
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_pin_set(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}


/**@brief Verfies the hardware ID of the gas sensor.
 */
static ret_code_t hw_id_verify(void)
{
    uint8_t  hw_id = 0;
    bool     hw_id_correct = false;

    if(ccs811_open() == SUCCESS)
    {
        if( drv_ccs811_hw_id_get(&hw_id) == SUCCESS )
        {
            CCS811_PRINT_IF_SENSOR_ERROR;
            hw_id_correct = ( hw_id == GAS_SENSOR_ID ) ? true : false;
        }
    }

    (void)ccs811_close();

    return hw_id_correct == true ? SUCCESS : ERROR_NOT_FOUND;
}


/**@brief Verifes that a valid app version is running on the gas sensor.
 */
static ret_code_t valid_app_verify(void)
{
    ret_code_t  err_code;
    uint8_t     status;

    err_code = ccs811_open();
    RETURN_IF_ERROR(err_code);

    err_code = drv_ccs811_status_get(&status);
    RETURN_IF_ERROR(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    err_code = ccs811_close();
    RETURN_IF_ERROR(err_code);

    if (status & DRV_CCS811_STATUS_APP_VALID_Msk)
    {
        return SUCCESS;
    }
    return ERROR_NOT_FOUND;
}

#ifdef GAS_SENSOR_DEBUG
    /**@brief Gets all hardware and firmware version information from the gas sensor.
    */
    static ret_code_t gas_sensor_info_get(uint8_t * p_hw_id, uint8_t * p_hw_version, uint16_t * p_fw_boot_version, uint16_t * p_fw_app_version)
    {
        ret_code_t  err_code;

        err_code = ccs811_open();
        RETURN_IF_ERROR(err_code);

        err_code = drv_ccs811_hw_id_get(p_hw_id);
        RETURN_IF_ERROR(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;

        err_code = drv_ccs811_hw_version_get(p_hw_version);
        RETURN_IF_ERROR(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;

        err_code = drv_ccs811_fw_boot_version_get(p_fw_boot_version);
        RETURN_IF_ERROR(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;

        err_code = drv_ccs811_fw_app_version_get(p_fw_app_version);
        RETURN_IF_ERROR(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;

        err_code = ccs811_close();
        RETURN_IF_ERROR(err_code);

        return SUCCESS;
    }
#endif


/**@brief Get result from gas sensor "For external use".
 */
drv_ccs811_alg_result_t get_gas_sensor_values()
{
    ret_code_t                      err_code;
    drv_ccs811_alg_result_descr_t   result_descr = DRV_CCS811_ALG_RESULT_DESCR_ALL;
    static drv_ccs811_alg_result_t  s_result;
    err_code = ccs811_open();
    //APP_ERROR_CHECK(err_code);

    err_code = drv_ccs811_alg_result_data_get(result_descr, &s_result);
    LOG(LVL_DEBUG,"gas_data_handler eCO2:, %d, - TVOC:, %d,\r\n", s_result.ec02_ppm,s_result.tvoc_ppb);
    //APP_ERROR_CHECK(err_code);
    err_code = ccs811_close();
    //APP_ERROR_CHECK(err_code);
    return s_result;
}



// /**@brief GPIOTE event handler for gas sensor. Called when the gas sensor signals a pin interrupt (sample finished).
//  */

// static void gpio_evt_handler(uint8_t pin, gpio_event_e event)
// {

//     if ((pin == CCS_INT) && (nrf_gpio_pin_read(CCS_INT) == 0)) // Check that pin is low (interrupt has occured);
//     {  
//         //maybe app schedueler has to be used
//         gpio_evt_scheduled();
//         //APP_ERROR_CHECK(err_code);
//     }
// }

// /**@brief Initlialize the GPIOTE for capturing gas sensor pin interrupts.
//  */
// static ret_code_t gpiote_init(uint32_t pin)
// {   

//     gpio_res_e err_code = GPIO_register_for_event(pin, GPIO_PULLUP, GPIO_EVENT_ALL, 0, gpio_evt_handler );


//     if(err_code == GPIO_RES_OK)
//     { 
//         return SUCCESS;
//     }
//     else
//     { 
//         return ERROR;
//     }
  
// }

// /**@brief Uninitialize the GPIOTE system.
//  */
// static ret_code_t gpiote_uninit(uint32_t pin)
// {
//     nrf_drv_gpiote_in_uninit(pin);

//     return SUCCESS;
// }


uint32_t drv_ext_gpio_pin_set(uint32_t pin_number)
{
    
    return sx1509_set_pin_level(BANK_A, pin_number, 1);
}

uint32_t drv_ext_gpio_pin_clear(uint32_t pin_number)
{
    
    return sx1509_set_pin_level(BANK_A, pin_number, 0);
}


uint32_t drv_ext_gpio_cfg_output(uint32_t pin_number)
{

    return sx1509_set_pin_as_output(BANK_A, pin_number);
    uint32_t err_code;

    // VALID_PIN_CHECK(pin_number);

    //err_code = drv_sx1509_open(m_drv_ext_gpio.p_cfg);
    // RETURN_IF_ERROR_EXT_GPIO_CLOSE(err_code);

    //err_code = drv_sx1509_dir_modify(0, (1UL << pin_number));
    // RETURN_IF_ERROR_EXT_GPIO_CLOSE(err_code);

    //err_code = drv_sx1509_close();
    // RETURN_IF_ERROR_EXT_GPIO_CLOSE(err_code);

    // return 0;
}

/**@brief Configures the IO pins of the host controller.
 */
static ret_code_t io_pins_init(void)
{
    ret_code_t    err_code;

    err_code = drv_ext_gpio_cfg_output(SX_CCS_PWR_CTRL);
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_cfg_output(SX_CCS_RESET);
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_cfg_output(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    #if defined(THINGY_HW_v0_7_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #elif defined(THINGY_HW_v0_8_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #elif defined(THINGY_HW_v0_9_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #else
        while (drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL)   != SUCCESS);
    #endif

    err_code = drv_ext_gpio_pin_set(SX_CCS_RESET);
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_pin_clear(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    nrf_delay_ms(GAS_SENSOR_PWR_ON_DELAY_MS); // Allow the CCS811 to power up.

    return SUCCESS;
}


ret_code_t drv_gas_sensor_calibrate_humid_temp(uint16_t rh_ppth, int32_t temp_mdeg)
{
    ret_code_t err_code;

    err_code = ccs811_open();
    RETURN_IF_ERROR(err_code);

    err_code = drv_ccs811_env_data_set(rh_ppth, temp_mdeg);
    
    RETURN_IF_ERROR(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    err_code = ccs811_close();
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}





ret_code_t drv_gas_sensor_start(drv_gas_sensor_mode_t mode)
{   
    ret_code_t  err_code;
    uint8_t     status;
    uint8_t     meas_mode_reg;
    uint8_t     drive_mode = DRV_CCS811_MEAS_MODE_DRIVE_MODE_Idle;

    if (m_started)
    {
        return SUCCESS;
    }

    m_started = true;

    switch (mode)
    {
        case DRV_GAS_SENSOR_MODE_250MS:
            return ERROR_NOT_SUPPORTED;
        case DRV_GAS_SENSOR_MODE_1S:
            drive_mode = DRV_CCS811_MEAS_MODE_DRIVE_MODE_ConstPwr;
            break;
        case DRV_GAS_SENSOR_MODE_10S:
            drive_mode = DRV_CCS811_MEAS_MODE_DRIVE_MODE_PulseHeat;
            break;
        case DRV_GAS_SENSOR_MODE_60S:
            drive_mode = DRV_CCS811_MEAS_MODE_DRIVE_MODE_LowPwrPulseHeat;
            break;
        default:
            return ERROR_INVALID_PARAM;
    }
    
    // err_code = gpiote_init(CCS_INT);
    //APP_ERROR_CHECK(err_code);

    // Power on gas sensor.
    #if defined(THINGY_HW_v0_7_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #elif defined(THINGY_HW_v0_8_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #elif defined(THINGY_HW_v0_9_0)
        while (drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL) != SUCCESS);
    #else
        while (drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL)   != SUCCESS);
    #endif

    err_code = drv_ext_gpio_pin_clear(SX_CCS_WAKE);
    //APP_ERROR_CHECK(err_code);

    err_code = drv_ext_gpio_pin_set(SX_CCS_RESET);
    //APP_ERROR_CHECK(err_code);

    nrf_delay_ms(GAS_SENSOR_PWR_ON_DELAY_MS); // Allow the CCS811 to power up.
    
    // nrf_drv_gpiote_in_event_enable(CCS_INT, true);

    err_code = ccs811_open();
    //APP_ERROR_CHECK(err_code);

    err_code = drv_ccs811_app_start();
    //APP_ERROR_CHECK(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    err_code = drv_ccs811_status_get(&status);
    //APP_ERROR_CHECK(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    if (!(status & DRV_CCS811_STATUS_FW_MODE_Msk))
    {   
        return ERROR_INVALID_STATE;
    }

    if (status & DRV_CCS811_STATUS_DATA_READY_Msk)
    {
        // Dummy readout
        drv_ccs811_alg_result_descr_t result_descr = DRV_CCS811_ALG_RESULT_DESCR_ALL;
        drv_ccs811_alg_result_t       result;

        err_code = drv_ccs811_alg_result_data_get(result_descr, &result);
        //APP_ERROR_CHECK(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;
    }

    err_code = drv_ccs811_meas_mode_get(&meas_mode_reg);
    RETURN_IF_ERROR(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    meas_mode_reg |= (drive_mode << DRV_CCS811_MEAS_MODE_DRIVE_MODE_Pos) |
                     (DRV_CCS811_MEAS_MODE_INTERRUPT_Enable << DRV_CCS811_MEAS_MODE_INTERRUPT_Pos);

    err_code = drv_ccs811_meas_mode_modify(meas_mode_reg, 0);
    RETURN_IF_ERROR(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    err_code = ccs811_close();
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}

ret_code_t drv_gas_sensor_stop(void)
{
    ret_code_t  err_code;
    uint8_t     status;

    if (m_started == false)
    {
        return SUCCESS;
    }

    m_started = false;

    // nrf_drv_gpiote_in_event_disable(CCS_INT);
    // err_code = gpiote_uninit(CCS_INT);
    // RETURN_IF_ERROR(err_code);

    err_code = ccs811_open();
    RETURN_IF_ERROR(err_code);

    err_code = drv_ccs811_status_get(&status);
    RETURN_IF_ERROR(err_code);
    CCS811_PRINT_IF_SENSOR_ERROR;

    if (status & DRV_CCS811_STATUS_FW_MODE_Msk) // Chech that the device is in "application mode" and not "boot mode".
    {
        // Disable interrupt and set idle mode.
        err_code = drv_ccs811_meas_mode_modify(0, DRV_CCS811_MEAS_MODE_DRIVE_MODE_Msk | DRV_CCS811_MEAS_MODE_INTERRUPT_Msk);
        RETURN_IF_ERROR(err_code);
        CCS811_PRINT_IF_SENSOR_ERROR;
    }

    err_code = ccs811_close();
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_pin_clear(SX_CCS_RESET);
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_pin_clear(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    // Power off gas sensor.
    #if defined(THINGY_HW_v0_7_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #elif defined(THINGY_HW_v0_8_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #elif defined(THINGY_HW_v0_9_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #else
        err_code = drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL);
    #endif
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}

ret_code_t drv_gas_sensor_init()
{
    ret_code_t err_code;

    //NULL_PARAM_CHECK(p_init);
    //NULL_PARAM_CHECK(p_init->p_twi_instance);
    //NULL_PARAM_CHECK(p_init->p_twi_cfg);
    //NULL_PARAM_CHECK(p_init->data_handler);

    //m_drv_ccs811_cfg.p_twi_cfg      = p_init->p_twi_cfg;
    //m_drv_ccs811_cfg.p_twi_instance = p_init->p_twi_instance;
    //m_drv_ccs811_cfg.twi_addr       = p_init->twi_addr;
    //  m_data_handler                  = p_init->data_handler;

    drv_ccs811_init();

    err_code = io_pins_init();
    RETURN_IF_ERROR(err_code);

    err_code = hw_id_verify();
    RETURN_IF_ERROR(err_code);

    err_code = valid_app_verify();
    RETURN_IF_ERROR(err_code);

    #ifdef GAS_SENSOR_DEBUG // Print all info from the gas sensor.
        uint8_t  hw_id;
        uint8_t  hw_version;
        uint16_t fw_boot_version;
        uint16_t fw_app_version;

        err_code = gas_sensor_info_get(&hw_id, &hw_version, &fw_boot_version, &fw_app_version);
        RETURN_IF_ERROR(err_code);

        NRF_LOG_DEBUG(" HW ID: 0x%x, HW version: 0x%x, FW boot version: 0x%x, FW app version: 0x%x \r\n", hw_id, hw_version, fw_boot_version, fw_app_version);
    #endif

    err_code = drv_ext_gpio_pin_clear(SX_CCS_RESET);
    RETURN_IF_ERROR(err_code);

    err_code = drv_ext_gpio_pin_clear(SX_CCS_WAKE);
    RETURN_IF_ERROR(err_code);

    #if defined(THINGY_HW_v0_7_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #elif defined(THINGY_HW_v0_8_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #elif defined(THINGY_HW_v0_9_0)
        err_code = drv_ext_gpio_pin_set(SX_CCS_PWR_CTRL);
    #else
        err_code = drv_ext_gpio_pin_clear(SX_CCS_PWR_CTRL);
    #endif
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}


uint32_t gas_sensor_init()
{
    uint32_t       err_code;

    err_code = drv_gas_sensor_init();
    RETURN_IF_ERROR(err_code);

    return SUCCESS;
}

uint32_t calibrate_gas_sensor(uint16_t humid, float temp)
{
    uint32_t err_code;
        uint16_t rh_ppt    = humid * 10;
        int32_t temp_mdeg = (int32_t)(temp * 1000.0f);

        LOG(LVL_DEBUG,"Calibrating gas sensor: humid out %d [ppt], temp out: %d [mdeg C]\r\n", rh_ppt, temp_mdeg);

        err_code = drv_gas_sensor_calibrate_humid_temp(rh_ppt, temp_mdeg);
        LOG(LVL_DEBUG,"drv_gas_sensor_calibrate_humid_temp %" PRIu32 "\n",err_code);
        //RETURN_IF_ERROR(err_code);

        return SUCCESS;
}

 uint32_t gas_start(void)
{
   drv_gas_sensor_mode_t mode;
   mode = DRV_GAS_SENSOR_MODE_10S;
   return drv_gas_sensor_start(mode);
}

uint32_t gas_stop(void)
{
    uint32_t err_code;

        //err_code = humidity_temp_stop_for_gas_calibration();
        //RETURN_IF_ERROR(err_code);
    return drv_gas_sensor_stop();
}
