
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "uicr_config.h"
const uint32_t uicrreg = 0xFFFFFFF9;//2.1V

#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"   
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_lpcomp.h"

#include "nrf_drv_gpiote.h"

#include "nrf_power.h"
#include "bsp.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Xiecon v1"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (3 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define DEVICE_ADDR                     20

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  360  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   5    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_timer_id);                                          								/**< Battery timer. */

//#define BATTERY_LEVEL_MEAS_INTERVAL      	APP_TIMER_TICKS(3600*1000)                          /**< Battery level measurement interval (ticks). */
#define BATTERY_LEVEL_MEAS_INTERVAL      		APP_TIMER_TICKS(1000)                          /**< Battery level measurement interval (ticks). */

//static nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);
static uint32_t 	devid=0;
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


#define BLE_SEND_BUF_CNT            10
#define BLE_SEND_MSG_MAX_LEN        30

static uint8_t    m_nCurBufIdx = 0;
static uint8_t    m_nSentBufIdx = 0;
static uint8_t    m_ble_send_buf[BLE_SEND_BUF_CNT][BLE_SEND_MSG_MAX_LEN];

static uint16_t         m_nLedBlink;
#define FLASH_COUNT     5

static nrf_saadc_value_t adc_buf;                   //!< Buffer used for storing ADC value.
static uint16_t          m_batt_lvl_in_milli_volts; //!< Current battery level.

#define SI7210_CONFIG_POLARITY_OMNI             0

#define STATUS_DISCONNECTED             0
#define STATUS_ADVERTISING              1
#define STATUS_CONNECTED                2
#define STATUS_DISCONNECTING            4

uint8_t          m_nConnStatus;
static uint8_t   m_nMutex;

void battery_voltage_get(uint16_t * p_vbatt);
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
/*
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }
*/
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */

static void advertising_start(void)
{
    m_nConnStatus = STATUS_ADVERTISING;
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    // err_code = bsp_btn_ble_sleep_mode_prepare();
    // APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            // m_nConnStatus = STATUS_DISCONNECTED;
             advertising_start();
            // sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_nLedBlink = FLASH_COUNT;
            
             
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            m_nConnStatus = STATUS_CONNECTED;
            m_nLedBlink = 1000;
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_nConnStatus = STATUS_DISCONNECTED;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@snippet [UART Initialization] */

void twi_init (void)
{
//    ret_code_t err_code;
//
//    const nrf_drv_twi_config_t twi_config = {
//       .scl                = SCL_PIN_NUMBER,
//       .sda                = SDA_PIN_NUMBER,
//       .frequency          = NRF_DRV_TWI_FREQ_100K,
//       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//       .clear_bus_init     = false
//    };
//
//    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_twi_enable(&m_twi);
}


void push_msg_to_buffer(uint8_t* pData, uint8_t len) {
    //__disable_irq();
    
    len = len < BLE_SEND_MSG_MAX_LEN - 1 ? len : BLE_SEND_MSG_MAX_LEN - 1;

    m_ble_send_buf[ m_nCurBufIdx ][ 0 ] = len;
    memcpy( m_ble_send_buf[ m_nCurBufIdx ] + 1, pData, len );    
        
    m_nCurBufIdx = (m_nCurBufIdx + 1) % BLE_SEND_BUF_CNT;
    if( m_nCurBufIdx == m_nSentBufIdx ) {               // if buffer is full, last thing is deleted
        m_nSentBufIdx = (m_nSentBufIdx + 1) % BLE_SEND_BUF_CNT;
    }    
    
    //__enable_irq();
}

uint8_t check_msg_buffer() {
    if( m_nCurBufIdx == m_nSentBufIdx )
        return 0;
    
    return 1;
}

uint8_t pop_msg_from_buffer(uint8_t* pData) {
    if( pData == NULL )
        return 0;
    if( m_nCurBufIdx == m_nSentBufIdx )
        return 0;
    
    sd_nvic_critical_region_enter(&m_nMutex);
    //__disable_irq();
    
    uint8_t nOrgIdx = m_nSentBufIdx;
    memcpy( pData, m_ble_send_buf[ m_nSentBufIdx ] + 1, m_ble_send_buf[ m_nSentBufIdx ][0] );
    m_nSentBufIdx = (m_nSentBufIdx + 1) % BLE_SEND_BUF_CNT;
     
    sd_nvic_critical_region_exit(m_nMutex);
    //__enable_irq();
    return m_ble_send_buf[ nOrgIdx ][0];
}

void my_comm_send_voltage(uint16_t voltage) {
    uint8_t sendData[20] = {"Voltage"};
    m_nLedBlink = FLASH_COUNT;
    
    sprintf((char*)sendData, "Voltage %d.%d V", voltage/100, voltage%100);
   
    uint16_t length = strlen( (const char*)sendData );
    
    push_msg_to_buffer( sendData, length );
}

void my_comm_send(uint8_t* pData, uint16_t len) {
    uint32_t err_code;
    
    do
    {
        err_code = ble_nus_data_send(&m_nus, pData, &len, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}


void saadc_callback0(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;

        adc_result = p_event->data.done.p_buffer[0];

        m_batt_lvl_in_milli_volts =
            ADC_RESULT_IN_MILLI_VOLTS(adc_result); //+ DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        
        // my_comm_send( BLE_CMD_VOLTAGE, m_batt_lvl_in_milli_volts );
        my_comm_send_voltage( m_batt_lvl_in_milli_volts );
        NRF_LOG_INFO("battery : %d\n", m_batt_lvl_in_milli_volts);
    }
}


void saadc_init0(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
    //err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
    //APP_ERROR_CHECK(err_code);
    
    //err_code = nrf_drv_saadc_sample();
    //APP_ERROR_CHECK(err_code);
}


void battery_voltage_get(uint16_t * p_vbatt)
{
    VERIFY_PARAM_NOT_NULL_VOID(p_vbatt);

    *p_vbatt = m_batt_lvl_in_milli_volts;
    if (!nrf_drv_saadc_is_busy())
    {
        ret_code_t err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t    err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

/*  
    init.config.ble_adv_primary_phy     = BLE_GAP_PHY_CODED;
    init.config.ble_adv_extended_enabled = true;
*/
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    
    if(NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_2V1)
    {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen<<NVMC_CONFIG_WEN_Pos;
      while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
      NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_2V1;
      
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
      while(NRF_NVMC->READY == NVMC_READY_READY_Busy){}
    }
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);    
    
    NRF_POWER->DCDCEN = 1;
    NRF_POWER->DCDCEN0 = 1;
    NRF_USBD->ENABLE = 0;
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */

static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

void timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint16_t  result;
    battery_voltage_get( &result );
}

void init_ports()
{
  for(int i = 0; i < 31; i++)
  {
    nrf_gpio_pin_dir_set(i, NRF_GPIO_PIN_DIR_INPUT);
  }
}

/**
 * @brief Initialize LPCOMP driver.
*/

/**/
static void lpcomp_handler(nrf_lpcomp_event_t event)
//void pir_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//  if( pin == PIR_H ) {
//    if(nrf_gpio_pin_read(PIR_L) == 0)
    if (event == NRF_LPCOMP_EVENT_CROSS)
    {
      my_comm_send_pir();
    }
//  }else if(pin == PIR_L)
//  {
//    if(nrf_gpio_pin_read(PIR_H) == 0)
//      my_comm_send_pir();
//  }
}

void lpcomp_init(void)
{
    uint32_t                err_code;

    nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;
    //config.input = NRF_LPCOMP_INPUT_5;

    // initialize LPCOMP driver, from this point LPCOMP will be active and provided
    // event handler will be executed when defined action is detected
    err_code = nrf_drv_lpcomp_init(&config, lpcomp_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_lpcomp_enable();
}

uint32_t getDevId()
{
	uint32_t ret = 0;
  ret = NRF_FICR->DEVICEID[1] << 8;
  ret += NRF_FICR->DEVICEID[0];
	
	return ret;
}

void put_powerdown()
{
    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);
    nrf_pwr_mgmt_feed();
//    m_is_ready = true;
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_CONTINUE);

}
/**@brief Application main function.
 */
  int main(void)
{    
// Initialize.
// uart_init();
    log_init();
    NRF_LOG_INFO("===========START Xiecon 840===========\n");
		
    uint8_t  data_send[BLE_SEND_MSG_MAX_LEN];
    uint16_t data_len;
    
    m_nConnStatus = STATUS_DISCONNECTED;
    init_ports();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    
    nrf_drv_gpiote_init();
//    saadc_init0();
//    app_timer_init();
//    app_timer_create(&m_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                timeout_handler);
//    app_timer_start(m_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);  
//    timeout_handler(NULL);
    
    advertising_start();
    
     // Enter main loop.
    for (;;)
    {
        if( check_msg_buffer() ) {
          if( m_nConnStatus == STATUS_CONNECTED ) {                              
            do {
              data_len = pop_msg_from_buffer(data_send);
              if( data_len > 0 ) {
                  my_comm_send(data_send, data_len);
              }
            } while( data_len > 0 );
          }
          idle_state_handle();
        } else {          
          idle_state_handle();
        }
    }
}


/**
 * @}
 */
