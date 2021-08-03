
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
//#include "uicr_config.h"
////#pragma location="UICR_REG0"
////const uint32_t uicrreg = 0xFFFFFFF9;//2.1V
//const uint32_t uicrreg = 0xFFFFFFFC;//3.0V

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

#include "nrf_drv_saadc.h"

#include "nrf_drv_gpiote.h"

#include "nrf_power.h"
#include "bsp.h"
#include "my_memory.h"
#include "millis.h"

#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "global.h"
#include "my_board.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                1600                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (3 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  360  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   5    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint8_t ad_vals[FSDB_TRACKSIZE];
static uint8_t ad_buf[FSDB_TRACKSIZE];
static uint8_t cur_channel = 0;
static uint16_t res_val = 0;

#define SAADC_SAMPLES_IN_BUFFER         4

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

//static nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);
uint32_t 	devid=0;
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
static uint8_t    sampling_finished = 0;
static uint16_t          m_batt_lvl_in_milli_volts; //!< Current battery level.
static nrf_saadc_value_t adc_buf;    

#define STATUS_DISCONNECTED             0
#define STATUS_ADVERTISING              1
#define STATUS_CONNECTED                2
#define STATUS_DISCONNECTING            4

uint8_t          m_nConnStatus;
static uint8_t   m_nMutex;
static uint32_t  m_overtime = TIMEOVER_1HZ;
uint8_t g_app_mode = APP_MODE_INIT;

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
    static char devname[128];
    
    sprintf(devname, "%s_%X", DEVICE_NAME, devid);

    NRF_LOG_INFO("device name : %s", devname);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) devname,
                                          strlen(devname));
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
             
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            m_nConnStatus = STATUS_CONNECTED;
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

void my_comm_send_voltage(uint16_t voltage) {
    uint8_t sendData[20] = {"Voltage"};
    
    sprintf((char*)sendData, "Voltage %d.%d V", voltage/100, voltage%100);
   
    uint16_t length = strlen( (const char*)sendData );
    
    push_msg_to_buffer( sendData, length );
}

void my_comm_send_adc_result(uint8_t* sendData, uint32_t length) {
    push_msg_to_buffer( sendData, length );
}
//
void init_gpio()
{
  //nrf_drv_gpiote_init();
  nrf_gpio_cfg_output(FORCE_ON);
  nrf_gpio_cfg_output(FORCE_OFF);
}

void RS232_TurnOn(bool bOn)
{
  if(bOn)
  {
    //auto power-down disabled
    nrf_gpio_pin_set(FORCE_ON);
//    //auto power-down enabled
//    nrf_gpio_pin_clear(FORCE_ON);

    nrf_gpio_pin_set(FORCE_OFF);
  }else
  {
    nrf_gpio_pin_set(FORCE_ON);
    nrf_gpio_pin_clear(FORCE_OFF);
  }
}
//
//uint8_t select_channel(uint8_t ch)
//{
//    if(ch > 18)
//      return 1;
////    g_channel_num = ch;
//    if(ch < 16){
//      nrf_gpio_pin_set(ENABLE1);
//      nrf_gpio_pin_clear(ENABLE2);
//
//      if(ch & 1) nrf_gpio_pin_set(ADDR0); else nrf_gpio_pin_clear(ADDR0);
//      if(ch & 2) nrf_gpio_pin_set(ADDR1); else nrf_gpio_pin_clear(ADDR1);
//      if(ch & 4) nrf_gpio_pin_set(ADDR2); else nrf_gpio_pin_clear(ADDR2);
//      if(ch & 8) nrf_gpio_pin_set(ADDR3); else nrf_gpio_pin_clear(ADDR3);
//    
//      nrf_gpio_pin_clear(ADDR4); 
//      nrf_gpio_pin_clear(ADDR5);
//    
//    }else{
//      nrf_gpio_pin_clear(ENABLE1);
//      nrf_gpio_pin_set(ENABLE2);
//      nrf_gpio_pin_clear(ADDR0); 
//      nrf_gpio_pin_clear(ADDR1);
//      nrf_gpio_pin_clear(ADDR2);
//      nrf_gpio_pin_clear(ADDR3);
//      
//      if((ch+1) & 0x1) nrf_gpio_pin_set(ADDR4); else nrf_gpio_pin_clear(ADDR4);
//      if((ch+1) & 0x2) nrf_gpio_pin_set(ADDR5); else nrf_gpio_pin_clear(ADDR5);
//    }
//
//    return 0;
//}
//
////select channels
//void disable_selection()
//{
//    nrf_gpio_pin_clear(ENABLE1);
//    nrf_gpio_pin_clear(ENABLE2);
//}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snipp

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_RX_PIN,
        .tx_pin_no    = UART_TX_PIN,
        .rts_pin_no   = 14,
        .cts_pin_no   = 11,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

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

//    battery_voltage_get( &result );
}

uint32_t getDevId()
{
    uint32_t ret = 0;
    ret = NRF_FICR->DEVICEID[1] << 8;
    ret += NRF_FICR->DEVICEID[0];
  
    return ret;
}

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_config.frequency = NRF_TIMER_FREQ_62500Hz;
    err_code = nrf_drv_timer_init(&m_timer, &timer_config, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer,50);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_disable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint16_t adc_value;
        uint16_t bytes_to_send;
     
        // set buffers
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);
						
//        NRF_LOG_INFO("%d(%d)", p_event->data.done.p_buffer[0], m_adc_evt_counter);

        adc_value = p_event->data.done.p_buffer[0];
        ad_vals[cur_channel*2] = (uint8_t)(adc_value & 0xff);
        ad_vals[cur_channel*2+1] = (uint8_t)(adc_value >> 8) & 0xff;
        
        cur_channel++;
        select_channel(cur_channel);

       if(cur_channel >= CHANNELS-1)
       {
          saadc_sampling_event_disable();
          nrfx_saadc_uninit();
          sampling_finished = 1;
       }
       cur_channel %= CHANNELS;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;
	
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    channel_0_config.gain = NRF_SAADC_GAIN1_4;
    channel_0_config.reference = NRF_SAADC_REFERENCE_VDD4;
//	
    nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    channel_1_config.gain = NRF_SAADC_GAIN1_4;
    channel_1_config.reference = NRF_SAADC_REFERENCE_VDD4;
			
	
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
//    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);   
//    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);
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
        
        ad_vals[FSDB_TRACKSIZE-2] = (uint8_t)(m_batt_lvl_in_milli_volts & 0xff);
        ad_vals[FSDB_TRACKSIZE-1] = (uint8_t)(m_batt_lvl_in_milli_volts >> 8) & 0xff;

        nrfx_saadc_uninit();
        NRF_LOG_INFO("battery : %d\n", m_batt_lvl_in_milli_volts);
    }
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

void put_powerdown()
{
    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);
    nrf_pwr_mgmt_feed();
//    m_is_ready = true;
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_CONTINUE);

}

static uint8_t txbuf[255] = {0};
void print_res_data(uint8_t* pData, uint32_t len)
{
    ret_code_t err_code = NRF_SUCCESS;
    memcpy(txbuf, pData, len); txbuf[len] = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        do
        {
            err_code = app_uart_put(txbuf[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_INFO("Failed to put uart data. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
}

void set_ble_tx_power()
{    //rf tx power set 8DB. 
    uint32_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8); 
    APP_ERROR_CHECK(err_code);
}

void app_proc()
{
    static char strbuf[255];
    static uint32_t tm = 0, tm1 = 0;
    if(millis() - tm > m_overtime)
    {
        switch(g_app_mode)
        {
          case APP_MODE_10HZ:
            NRF_LOG_INFO("===========10 hz mode============");
          break;
          case APP_MODE_1HZ:
            NRF_LOG_INFO("===========1 hz mode============");
          break;
          case APP_MODE_SLEEP:
            NRF_LOG_INFO("===========0.1 hz mode============");
          break;
        }
        
        for(int i = 0; i < CHANNELS-1; i++)
        {
          res_val = (ad_vals[i*2+1]*0x100) + ad_vals[i*2];
          NRF_LOG_INFO("adc(%d):%d", i+1, res_val); 
          sprintf(strbuf, "C%d:%d\r\n", i+1, res_val);
#ifdef UART_ENABLE 
            print_res_data(strbuf, strlen(strbuf));
#endif              
        }
        memcpy(ad_buf, ad_vals, FSDB_TRACKSIZE);
        my_comm_send_adc_result(ad_buf, FSDB_TRACKSIZE);

        NRF_LOG_INFO("\n");
        
        fstorage_write(FSDB_BODY_ADDR + g_page_addr, (uint8_t*)ad_vals, FSDB_TRACKSIZE);
        print_fsdb_body(g_page_addr, FSDB_TRACKSIZE*2);
        
        NRF_LOG_INFO("db pointer: %d", g_page_addr);
        fstorage_erase(FSDB_BASE_ADDR, 1);
        fstorage_write(FSDB_BASE_ADDR, (uint8_t*)&g_page_addr, sizeof(g_page_addr));
        print_fsdb_header();

        g_page_addr += FSDB_TRACKSIZE;

        if(g_page_addr % FSDB_PAGESIZE == 0)
        {
          NRF_LOG_INFO("[FSDB] erasing body [%d]", FSDB_BODY_ADDR + g_page_addr);
          fstorage_erase(FSDB_BODY_ADDR + g_page_addr, 1);
        }

        if(g_page_addr > FSDB_PAGESIZE * FSDB_PAGECNT)
          g_page_addr = 0;            


        cur_channel = 0;
        select_channel(cur_channel);
        
        saadc_init();
        saadc_sampling_event_enable();
        tm = millis();
        NRF_LOG_FLUSH();
    }
    if(millis() - tm1 > TIMEOVER_GET_BATT && sampling_finished)
    {
      uint16_t  result;
      saadc_init0();
      battery_voltage_get( &result );
      sampling_finished = 0;
      tm1 = millis();
    }
}

void ble_proc()
{
    static uint8_t  data_send[BLE_SEND_MSG_MAX_LEN];
    static uint16_t data_len;

    if( check_msg_buffer() ) {
      if( m_nConnStatus == STATUS_CONNECTED ) {                              
        do {
          data_len = pop_msg_from_buffer(data_send);
          if( data_len > 0 ) {
              my_comm_send(data_send, data_len);
#ifdef UART_ENABLE 
              for(int i = 0; i < data_len; i++)
                app_uart_put(data_send[i]);
#endif 
          }
        } while( data_len > 0 );
      }
    }
}

void my_app_proc()
{
  switch(g_app_mode)
  {
    case APP_MODE_INIT:
      
      memset(ad_vals, 0, sizeof(ad_vals));
      g_page_addr = 0;
      m_nConnStatus = STATUS_DISCONNECTED;
      select_channel(0);

//    reset_fsdb_contents();
      read_fsdb_harder();
      print_fsdb_header();
      print_fsdb_body(g_page_addr, FSDB_TRACKSIZE);
      
      saadc_init();
      saadc_sampling_event_enable();
      start_millis_timer();

      g_app_mode = DEFULAT_MODE;

    break;
    case APP_MODE_10HZ:
      m_overtime = TIMEOVER_10HZ;
      app_proc();
    break;
    case APP_MODE_1HZ:
      m_overtime = TIMEOVER_1HZ;
      app_proc();
    break;
    case APP_MODE_SLEEP:
      m_overtime = TIMEOVER_SLEEP;
      app_proc();
    break;
  }
}

/**@brief Application main function.
 */
int main(void)
{    
    ret_code_t rc;
    
    power_management_init();
    devid = getDevId();
        
    log_init();    
    
    init_gpio();
    RS232_TurnOn(true);

    timers_init();
    init_millis_timer();
    
    init_fstorage();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

//#ifdef UART_ENABLE    
//    uart_init();
//#endif

    NRF_LOG_INFO("=========== START ScaleCon ===========\r\n");
    advertising_start();
//  set_ble_tx_power();
    NRF_LOG_FLUSH();
    
//    RS232_TurnOn(false);
    // Enter main loop.
    for (;;)
    {
//        my_app_proc();
//        ble_proc();
//        app_proc();

        idle_state_handle();
        NRF_LOG_FLUSH();
    }
}


/**
 * @}
 */
