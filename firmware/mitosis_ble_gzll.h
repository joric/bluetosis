#include <stdarg.h>
#include "nrf_gzll.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_soc.h"

#define FILTER_OUT_UART_PIN_KEY // uncomment on unwanted HID reports

#ifdef COMPILE_LEFT
#pragma message "COMPILE_LEFT"
#define RGB_DI_PIN 19
#define DEBUG_PIN 20
#endif

#ifdef COMPILE_RIGHT
#pragma message "COMPILE_RIGHT"
#define RGB_DI_PIN 21
#define DEBUG_PIN 20
#endif

#define RGBLED_NUM 22

#define KEY_RF S17
#define KEY_ADJUST S16
#define KEY_FN S18
#define KEY_PINKY S22

#include "mitosis_keymap.h"

#define RGBLIGHT_ENABLE

//#define DISPLAY_ENABLE

#ifdef DISPLAY_ENABLE
#include "display.h"
#endif

int rgb_mode = 0;
#ifdef RGBLIGHT_ENABLE
#include "rgb_modes.h"
#endif

// external receiver support
typedef enum
{
    BLE,
    GAZELL
} radio_mode_t;

radio_mode_t running_mode = BLE;

#if !(NRF_LOG_ENABLED)
#define SIMPLE_DEBUG
#endif

#ifdef SIMPLE_DEBUG
#include "app_uart.h"

#undef NRF_LOG_INFO
#undef NRF_LOG_DEBUG
#undef NRF_LOG_PROCESS
#define NRF_LOG_INFO printf
#define NRF_LOG_DEBUG printf
#define NRF_LOG_PROCESS() false

#undef NRF_LOG_INIT
#define NRF_LOG_INIT debug_init

#undef RX_PIN_NUMBER
#undef TX_PIN_NUMBER
#undef CTS_PIN_NUMBER
#undef RTS_PIN_NUMBER
#undef HWFC

#define RX_PIN_NUMBER  -1
#define TX_PIN_NUMBER  DEBUG_PIN
#define CTS_PIN_NUMBER -1
#define RTS_PIN_NUMBER -1
#define HWFC false

void debug_log(const char *fmt, ...)
{
    va_list list;
    va_start(list, fmt);
    char buf[256] = { 0 };
    vsprintf(buf, fmt, list);
    va_end(list);
    for (char *p = buf; *p; p++)
        app_uart_put(*p); // needs fifo library
}

void uart_error_handle(app_uart_evt_t * p_event)
{
}

uint32_t debug_init()
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params = {
        RX_PIN_NUMBER, TX_PIN_NUMBER, RTS_PIN_NUMBER, CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED, false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_error_handle, APP_IRQ_PRIORITY_LOW, err_code);
    APP_ERROR_CHECK(err_code);
    printf("\nUART initialized\n");
    return err_code;
}

#undef APP_ERROR_CHECK
#define APP_ERROR_CHECK(x) if (x!=NRF_SUCCESS) printf("ERROR 0x%04x in line %u\n", (int)x, __LINE__)

#endif // SIMPLE_DEBUG

#define ADDR_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define ADDR_T(a) a[5], a[4], a[3], a[2], a[1], a[0]

// Define payload length
#define TX_PAYLOAD_LENGTH 3 ///< 3 byte payload length when transmitting

static uint8_t data_payload_left[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];    ///< Placeholder for data payload received from host.
static uint8_t data_payload_right[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];    ///< Placeholder for data payload received from host.

static bool packet_received_left, packet_received_right;
static uint8_t ack_payload[TX_PAYLOAD_LENGTH];    ///< Payload to attach to ACK sent to device.
uint32_t left_active = 0;
uint32_t right_active = 0;
void key_handler();

// Key buffers
static uint32_t keys = 0, keys_snapshot = 0;
static uint32_t keys_recv = 0, keys_recv_snapshot = 0;
static uint8_t data_buffer[10];


// Setup switch pins with pullups
static void gpio_config(void)
{
    nrf_gpio_cfg_sense_input(S01, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S02, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S03, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S04, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S05, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S06, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S07, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S08, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S09, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S10, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S11, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S12, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S13, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S14, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S15, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S16, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S17, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S18, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S19, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S20, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S21, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S22, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(S23, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
}


void keys_to_payload(uint8_t * data_payload, uint32_t keys)
{
    data_payload[0] = ((keys & 1<<S01) ? 1:0) << 7 | \
                      ((keys & 1<<S02) ? 1:0) << 6 | \
                      ((keys & 1<<S03) ? 1:0) << 5 | \
                      ((keys & 1<<S04) ? 1:0) << 4 | \
                      ((keys & 1<<S05) ? 1:0) << 3 | \
                      ((keys & 1<<S06) ? 1:0) << 2 | \
                      ((keys & 1<<S07) ? 1:0) << 1 | \
                      ((keys & 1<<S08) ? 1:0) << 0;

    data_payload[1] = ((keys & 1<<S09) ? 1:0) << 7 | \
                      ((keys & 1<<S10) ? 1:0) << 6 | \
                      ((keys & 1<<S11) ? 1:0) << 5 | \
                      ((keys & 1<<S12) ? 1:0) << 4 | \
                      ((keys & 1<<S13) ? 1:0) << 3 | \
                      ((keys & 1<<S14) ? 1:0) << 2 | \
                      ((keys & 1<<S15) ? 1:0) << 1 | \
                      ((keys & 1<<S16) ? 1:0) << 0;

    data_payload[2] = ((keys & 1<<S17) ? 1:0) << 7 | \
                      ((keys & 1<<S18) ? 1:0) << 6 | \
                      ((keys & 1<<S19) ? 1:0) << 5 | \
                      ((keys & 1<<S20) ? 1:0) << 4 | \
                      ((keys & 1<<S21) ? 1:0) << 3 | \
                      ((keys & 1<<S22) ? 1:0) << 2 | \
                      ((keys & 1<<S23) ? 1:0) << 1 | \
                      0 << 0;
}


void payload_to_data_left()
{
    data_buffer[0] = ((data_payload_left[0] & 1<<3) ? 1:0) << 0 |
                     ((data_payload_left[0] & 1<<4) ? 1:0) << 1 |
                     ((data_payload_left[0] & 1<<5) ? 1:0) << 2 |
                     ((data_payload_left[0] & 1<<6) ? 1:0) << 3 |
                     ((data_payload_left[0] & 1<<7) ? 1:0) << 4;

    data_buffer[2] = ((data_payload_left[1] & 1<<6) ? 1:0) << 0 |
                     ((data_payload_left[1] & 1<<7) ? 1:0) << 1 |
                     ((data_payload_left[0] & 1<<0) ? 1:0) << 2 |
                     ((data_payload_left[0] & 1<<1) ? 1:0) << 3 |
                     ((data_payload_left[0] & 1<<2) ? 1:0) << 4;

    data_buffer[4] = ((data_payload_left[1] & 1<<1) ? 1:0) << 0 |
                     ((data_payload_left[1] & 1<<2) ? 1:0) << 1 |
                     ((data_payload_left[1] & 1<<3) ? 1:0) << 2 |
                     ((data_payload_left[1] & 1<<4) ? 1:0) << 3 |
                     ((data_payload_left[1] & 1<<5) ? 1:0) << 4;

    data_buffer[6] = ((data_payload_left[2] & 1<<5) ? 1:0) << 1 |
                     ((data_payload_left[2] & 1<<6) ? 1:0) << 2 |
                     ((data_payload_left[2] & 1<<7) ? 1:0) << 3 |
                     ((data_payload_left[1] & 1<<0) ? 1:0) << 4;

    data_buffer[8] = ((data_payload_left[2] & 1<<1) ? 1:0) << 1 |
                     ((data_payload_left[2] & 1<<2) ? 1:0) << 2 |
                     ((data_payload_left[2] & 1<<3) ? 1:0) << 3 |
                     ((data_payload_left[2] & 1<<4) ? 1:0) << 4;
}


void payload_to_data_right()
{
    data_buffer[1] = ((data_payload_right[0] & 1<<7) ? 1:0) << 0 |
                     ((data_payload_right[0] & 1<<6) ? 1:0) << 1 |
                     ((data_payload_right[0] & 1<<5) ? 1:0) << 2 |
                     ((data_payload_right[0] & 1<<4) ? 1:0) << 3 |
                     ((data_payload_right[0] & 1<<3) ? 1:0) << 4;

    data_buffer[3] = ((data_payload_right[0] & 1<<2) ? 1:0) << 0 |
                     ((data_payload_right[0] & 1<<1) ? 1:0) << 1 |
                     ((data_payload_right[0] & 1<<0) ? 1:0) << 2 |
                     ((data_payload_right[1] & 1<<7) ? 1:0) << 3 |
                     ((data_payload_right[1] & 1<<6) ? 1:0) << 4;

    data_buffer[5] = ((data_payload_right[1] & 1<<5) ? 1:0) << 0 |
                     ((data_payload_right[1] & 1<<4) ? 1:0) << 1 |
                     ((data_payload_right[1] & 1<<3) ? 1:0) << 2 |
                     ((data_payload_right[1] & 1<<2) ? 1:0) << 3 |
                     ((data_payload_right[1] & 1<<1) ? 1:0) << 4;

    data_buffer[7] = ((data_payload_right[1] & 1<<0) ? 1:0) << 0 |
                     ((data_payload_right[2] & 1<<7) ? 1:0) << 1 |
                     ((data_payload_right[2] & 1<<6) ? 1:0) << 2 |
                     ((data_payload_right[2] & 1<<5) ? 1:0) << 3;

    data_buffer[9] = ((data_payload_right[2] & 1<<4) ? 1:0) << 0 |
                     ((data_payload_right[2] & 1<<3) ? 1:0) << 1 |
                     ((data_payload_right[2] & 1<<2) ? 1:0) << 2 |
                     ((data_payload_right[2] & 1<<1) ? 1:0) << 3;
}


//void notification_cb(nrf_impl_notification_t notification);
/*lint -e526 "Symbol RADIO_IRQHandler not defined" */
void RADIO_IRQHandler(void);

static nrf_radio_request_t m_timeslot_request;
static uint32_t m_slot_length;
static volatile bool m_cmd_received = false;
static volatile bool m_gzll_initialized = false;
static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

void HardFault_Handler(uint32_t program_counter, uint32_t link_register)
{
}


void m_configure_next_event(void)
{
    m_slot_length = 10000;
    m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_NO_GUARANTEE;
    m_timeslot_request.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us = 100000;
}


void sys_evt_dispatch_ble_gzll(uint32_t evt_id)
{
    uint32_t err_code;

    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            ASSERT(false);
            break;

        case NRF_EVT_RADIO_SESSION_IDLE:
            ASSERT(false);
            break;

        case NRF_EVT_RADIO_SESSION_CLOSED:
            ASSERT(false);
            break;

        case NRF_EVT_RADIO_BLOCKED:
            //printf("Blocked\n");
            m_configure_next_event();
            err_code = sd_radio_request(&m_timeslot_request);
            APP_ERROR_CHECK(err_code);
            break;

        case NRF_EVT_RADIO_CANCELED:
            m_configure_next_event();
            err_code = sd_radio_request(&m_timeslot_request);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    fs_sys_event_handler(evt_id);
    ble_advertising_on_sys_evt(evt_id);
}


void m_process_gazell()
{
    // detecting received packet from interupt, and unpacking
    if (packet_received_left)
    {
        packet_received_left = false;
        payload_to_data_left();
    }

    if (packet_received_right)
    {
        packet_received_right = false;
        payload_to_data_right();
    }
}


static void m_on_start(void)
{
    bool res = false;
    signal_callback_return_param.params.request.p_next = NULL;
    signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    nrf_gzll_mode_t mode = running_mode == BLE ? NRF_GZLL_MODE_HOST : NRF_GZLL_MODE_DEVICE;

    if (!m_gzll_initialized)
    {
        // Initialize Gazell
        nrf_gzll_init(mode);

        nrf_gzll_set_device_channel_selection_policy(NRF_GZLL_DEVICE_CHANNEL_SELECTION_POLICY_USE_CURRENT);
        nrf_gzll_set_xosc_ctl(NRF_GZLL_XOSC_CTL_MANUAL);
        nrf_gzll_set_max_tx_attempts(0);    // NB! 100?? reversebias

        // Addressing
        nrf_gzll_set_base_address_0(0x01020304);
        nrf_gzll_set_base_address_1(0x05060708);

        // Load data into TX queue
        ack_payload[0] = 0x55;
        nrf_gzll_add_packet_to_tx_fifo(0, data_payload_left, TX_PAYLOAD_LENGTH);
        nrf_gzll_add_packet_to_tx_fifo(1, data_payload_left, TX_PAYLOAD_LENGTH);

        // Enable Gazell to start sending over the air
        nrf_gzll_enable();

        m_gzll_initialized = true;
    } else {
        res = nrf_gzll_set_mode(mode);
        ASSERT(res);
    }

    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0->CC[0] = m_slot_length - 4000;    // TODO: Use define instead of magic number
    NVIC_EnableIRQ(TIMER0_IRQn);

    (void)res;
}


static void m_on_multitimer(void)
{
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    if (nrf_gzll_get_mode() != NRF_GZLL_MODE_SUSPEND)
    {
        signal_callback_return_param.params.request.p_next = NULL;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        (void)nrf_gzll_set_mode(NRF_GZLL_MODE_SUSPEND);
        NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
        NRF_TIMER0->CC[0] = m_slot_length - 1000;
    }
    else
    {
        ASSERT(nrf_gzll_get_mode() == NRF_GZLL_MODE_SUSPEND);
        m_configure_next_event();
        signal_callback_return_param.params.request.p_next = &m_timeslot_request;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    }
}


nrf_radio_signal_callback_return_param_t *m_radio_callback(uint8_t signal_type)
{
    switch (signal_type)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            m_on_start();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            RADIO_IRQHandler();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            m_on_multitimer();
            break;
    }
    return (&signal_callback_return_param);
}


uint32_t gazell_sd_radio_init(void)
{
    uint32_t err_code;
    err_code = sd_radio_session_open(m_radio_callback);
    if (err_code != NRF_SUCCESS)
        return err_code;
    m_configure_next_event();
    err_code = sd_radio_request(&m_timeslot_request);
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        return err_code;
    }
    return NRF_SUCCESS;
}


void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    if (running_mode == BLE)
    {
        return;
    }

    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
    }
}


void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
}


void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    if (running_mode == GAZELL)
    {
        return;
    }

    //printf("gzll data received\n");

    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (pipe == 0)
    {
        packet_received_left = true;
        left_active = 0;
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload_left, &data_payload_length);
    }
    else if (pipe == 1)
    {
        packet_received_right = true;
        right_active = 0;
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload_right, &data_payload_length);
    }
    // not sure if required, I guess if enough packets are missed during blocking uart
    nrf_gzll_flush_rx_fifo(pipe);

    //load ACK payload into TX queue
    ack_payload[0] = 0x55;
    nrf_gzll_add_packet_to_tx_fifo(pipe, ack_payload, TX_PAYLOAD_LENGTH);


    m_process_gazell();
}


void nrf_gzll_disabled(void)
{
}


bool debug_cmd_available(void)
{
    return m_cmd_received;
}


char get_debug_cmd(void)
{
    char cmd = ack_payload[0];
    m_cmd_received = false;
    return cmd;
}


uint8_t get_battery_level(void)
{
    // Configure ADC
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos)
                    | (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)
                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)
                    | (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos)
                    | (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->EVENTS_END = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;

    while (!NRF_ADC->EVENTS_END);

    uint16_t vbg_in_mv = 1200;
    uint8_t adc_max = 255;
    uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->TASKS_STOP = 1;

    int percent = battery_level_in_percent(vbat_current_in_mv); // app_util.h

    printf("Sending BAS report: %u%% (%umV)\n", percent, vbat_current_in_mv);

    return (uint8_t) percent;
}


// Return the key states, masked with valid key pins
static uint32_t read_keys(void)
{
#ifdef FILTER_OUT_UART_PIN_KEY
    uint32_t uart_pin = 1 << TX_PIN_NUMBER;
    return ~NRF_GPIO->IN & INPUT_MASK & ~uart_pin;
#else
    return ~NRF_GPIO->IN & INPUT_MASK;
#endif
}


static void send_data()
{
    keys_to_payload(data_payload_right, keys);

    if (running_mode == GAZELL)
    {
        nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload_right, TX_PAYLOAD_LENGTH);
    }
    else
    {
        payload_to_data_right();
    }
}


#define FILE_ID     0x1111
#define REC_KEY     0x2222

fds_record_desc_t   record_desc;
fds_record_t        record;
fds_record_chunk_t  record_chunk;
fds_flash_record_t  flash_record;

typedef struct
{
    int index;
    int rgb_mode;
    int reserved2;
} savedata_t;

static savedata_t savedata;

void eeprom_read()
{
    fds_find_token_t       ftok = {0}; //Important, make sure you zero init the ftok token
    record.file_id              = FILE_ID;
    record.key                  = REC_KEY;
    record_chunk.p_data         = &savedata;
    record.data.p_chunks        = &record_chunk;
    record_chunk.length_words   = sizeof(savedata)/sizeof(uint32_t);
    record.data.num_chunks      = 1;

    bool found = false;

    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
    {
        found = true;
        ret_code_t ret = fds_record_open(&record_desc, &flash_record);
        APP_ERROR_CHECK(ret);
        savedata = *(savedata_t *)flash_record.p_data;
        fds_record_close(&record_desc);
    }

    if (!found)
    {
        ret_code_t ret = fds_record_write(&record_desc, &record);
        APP_ERROR_CHECK(ret);
    }

    rgb_mode = savedata.rgb_mode;
}


void eeprom_write()
{
    record_chunk.p_data = &savedata;
    ret_code_t ret = fds_record_update(&record_desc, &record);
    APP_ERROR_CHECK(ret);
}


static uint32_t switch_index = 0;

#define ACTIVITY 4*60*1000  // unactivity time till power off (4 minutes)
#define RESET_DELAY 50      // delayed reset

#define SWITCH_COUNT 4
#define PEERS_COUNT (SWITCH_COUNT - 1)
#define RF_INDEX (SWITCH_COUNT - 1)

static const uint8_t switch_keys[] = { S16, S17, S18, KEY_RF };

static uint8_t m_layer = 0;
static uint32_t activity_ticks = 0;
static uint32_t reset_ticks = 0;
static bool m_delayed_reset = false;


static uint32_t advertising_restart(ble_adv_mode_t mode, bool reset)
{
    uint32_t err_code;

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        sd_ble_gap_adv_stop();
        err_code = ble_advertising_start(mode);
    } else {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }

    if (reset)
    {
        ble_advertising_restart_without_whitelist();
    }

    return err_code;
}


static void switch_update(pm_peer_id_t peer_id)
{
    printf("switch_update\n");

    uint32_t err_code;
    ble_gap_addr_t gap_addr;
    uint16_t length = 8;
    uint8_t addr[8];

    err_code = pm_id_addr_get(&gap_addr);
    APP_ERROR_CHECK(err_code);

    for(uint8_t i=0; i<BLE_GAP_ADDR_LEN; i++)
        addr[i] = gap_addr.addr[i];

    addr[3] = switch_index;

    printf("storing address: " ADDR_FMT "\n", ADDR_T(addr));

    err_code = pm_peer_data_app_data_store(m_peer_id, addr, length, NULL);
    APP_ERROR_CHECK(err_code);
}


static void switch_select(uint8_t index)
{
    printf("switch_select: %d\n", (int)index);

    switch_index = index;
    savedata.index = switch_index;
    savedata.rgb_mode = rgb_mode;
    eeprom_write();

    uint32_t err_code;
    ble_gap_addr_t gap_addr;
    err_code = pm_id_addr_get(&gap_addr);
    APP_ERROR_CHECK(err_code);

    gap_addr.addr[3] = index; // switch status 1, 2, or 3

    printf("setting address: " ADDR_FMT "\n", ADDR_T(gap_addr.addr));

    err_code = pm_id_addr_set(&gap_addr);
    APP_ERROR_CHECK(err_code);
}


static void peer_list_find_and_delete_bonds(void)
{
    pm_peer_id_t peer_id;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);

    while (peer_id != PM_PEER_ID_INVALID)
    {
        uint16_t length = 8;
        uint8_t addr[8];

        if (pm_peer_data_app_data_load(peer_id, addr, &length) == NRF_SUCCESS)
        {
            printf ("trying to delete peer id %d stored index %d\n", (int)peer_id, (int)addr[3]);
            if (addr[3] == switch_index)
            {
                printf("DELETING peer %d\n", peer_id);
                pm_peer_delete(peer_id);
            }
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}


static void switch_reset(int index)
{
    printf("switch_reset: %d\n", (int)index);
    switch_index = index;
    peer_list_find_and_delete_bonds();
}


static void switch_init()
{
    eeprom_read();
    switch_index = savedata.index;

    running_mode = (switch_index == RF_INDEX) ? GAZELL : BLE;

#ifdef COMPILE_LEFT
//    switch_index = RF_INDEX;
//    running_mode = GAZELL;
#endif

    if (running_mode == BLE)
    {
        switch_select(switch_index);
    }
}


// temporary, should be moved to layouts
// NB! these called only on key release
void hardware_keys()
{
    bool k1 = keys & (1<<KEY_RF);
    bool k2 = keys & (1<<KEY_ADJUST);
    bool k3 = keys & (1<<KEY_FN);
    bool pinky = keys & (1<<KEY_PINKY);

    if ( k1 && k2 && k3 && pinky)
        m_delayed_reset = true;


    if (false) {
    if (keys & (1<<KEY_ADJUST))
    {
        int index = -1;

        for (int i=0; i<SWITCH_COUNT; i++)
        {
            if (keys & (1 << switch_keys[i]))
            {
                index = i;
                break;
            }
        }

        if (index != -1)
        {
            printf("INDEX: %d\n", index);

            if (keys & (1<<KEY_FN))
            {
                switch_reset(index);
                switch_select(index);
                advertising_restart(BLE_ADV_MODE_FAST, true);
            }
            else
            {
                switch_select(index);
                advertising_restart(BLE_ADV_MODE_FAST, false);
            }

            if (running_mode != ((index == RF_INDEX) ? GAZELL : BLE))
            {
                m_delayed_reset = true;
            }
        }
    }
    }

    if (m_delayed_reset)
    {
        reset_ticks++;
        if (reset_ticks * KEYBOARD_SCAN_INTERVAL > RESET_DELAY)
        {
            reset_ticks = 0;
            m_delayed_reset = false;
            NVIC_SystemReset();
        }
    }
    else
    {
        reset_ticks = 0;
    }
}


uint8_t get_modifier(uint16_t key)
{
    const int modifiers[] = { KC_LCTRL, KC_LSHIFT, KC_LALT, KC_LGUI, KC_RCTRL, KC_RSHIFT, KC_RALT, KC_RGUI };
    for (int b = 0; b < 8; b++)
        if (key == modifiers[b])
            return 1 << b;
    return 0;
}


void key_handler()
{
    if (running_mode == GAZELL)
    {
        return;
    }

    uint8_t buf[8];
    int modifiers = 0;
    int keys_pressed = 0;
    int keys_sent = 0;
    memset(buf, 0, sizeof(buf));

    for (uint8_t row = 0; row < MATRIX_ROWS; row++)
    {
        uint16_t val = (uint16_t) data_buffer[row * 2] | (uint16_t) data_buffer[row * 2 + 1] << 5;

        if (!val)
            continue;

        for (int col = 0; col < MATRIX_COLS; col++)
        {
            if (val & (1 << col))
            {
                keys_pressed++;
                uint16_t key = keymaps[m_layer][row][col];

                if (key == KC_TRNS)
                    key = keymaps[0][row][col];

                uint8_t modifier = get_modifier(key);

                if (modifier)
                {
                    modifiers |= modifier;
                }
                else if (key & QK_LAYER_TAP)
                {
                    m_layer = (key >> 8) & 0xf;
                }
                else if (key == RGBMOD)
                {
                    rgb_mode++;
                    savedata.rgb_mode = rgb_mode;
                    eeprom_write();
                }
                else if (keys_sent < MAX_KEYS_IN_ONE_REPORT)
                {
                    buf[2 + keys_sent++] = key;

#ifdef DISPLAY_ENABLE
display_keypress(key);
#endif

                }
            }
        }
    }

    if (!keys_pressed)
        m_layer = 0;

    buf[0] = modifiers;
    buf[1] = 0; // reserved

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        //printf("Sending HID report: %02x %02x %02x %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        ble_hids_inp_rep_send(&m_hids, INPUT_REPORT_KEYS_INDEX, INPUT_REPORT_KEYS_MAX_LEN, buf);
    }
}


void keyboard_task()
{

#ifdef DISPLAY_ENABLE
    display_update();
#endif

#ifdef RGBLIGHT_ENABLE
    rgb_task();
#endif

    keys_snapshot = read_keys();
    keys_recv_snapshot = data_payload_left[0] | (data_payload_left[1] << 8) | (data_payload_left[2] << 16);

    if (keys == keys_snapshot && keys_recv == keys_recv_snapshot)
    {
        activity_ticks++;
        if (activity_ticks * KEYBOARD_SCAN_INTERVAL > ACTIVITY)
        {
            printf("Shutting down on inactivity...\n");
            nrf_delay_ms(50);
            sd_power_system_off();
        }
    }
    else
    {
        activity_ticks = 0;
    }

    if (keys != keys_snapshot)
    {
        keys = keys_snapshot;
        send_data();
        hardware_keys();
        key_handler();
    }

    if (keys_recv != keys_recv_snapshot)
    {
        printf("keys_recv %d\n", (int)keys_recv);
        keys_recv = keys_recv_snapshot;
        key_handler();
    }
}

void mitosis_init(bool erase_bonds)
{
    printf("Mitosis init\n");

    switch_init();
    gpio_config();

#ifdef DISPLAY_ENABLE
    display_init();
    display_update();
#endif

    nrf_gpio_cfg_output(LED_PIN);

#ifndef RGBLIGHT_ENABLE
    for (int i = 0; i < 3; i++)
    {
        nrf_gpio_pin_set(LED_PIN);
        nrf_delay_ms(100);
        nrf_gpio_pin_clear(LED_PIN);
        nrf_delay_ms(100);
    }
#endif

#ifdef RGBLIGHT_ENABLE
    rgb_init();
#endif

    printf(running_mode == GAZELL ? "RECEIVER MODE\n" : "BLUETOOTH MODE\n");

    printf("SELECTED DEVICE: %d\n", (int)switch_index);

#ifdef COMPILE_REVERSED
    printf("REVERSED\n");
#endif

    // delete bonds on 3 thumb keys on startup
    keys = read_keys();
    bool k1 = keys & (1<<KEY_RF);
    bool k2 = keys & (1<<KEY_ADJUST);
    bool k3 = keys & (1<<KEY_FN);
    bool reset_bonds = k1 && k2 && k3;
    if (reset_bonds) {
        pm_peers_delete();
    }

    gazell_sd_radio_init();
}
