/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "espnow_example.h"

#ifndef MODE_TX
#define MODE_TX 1
#endif
#ifndef MODE_RX
#define MODE_RX 2
#endif
#ifndef MODE
#define MODE MODE_RX
#endif
#ifndef MODE_STR
#define MODE_STR "RX"
#endif
#if (MODE != MODE_TX) && (MODE != MODE_RX)
#error "MODE must be MODE_TX or MODE_RX"
#endif

#define ESPNOW_MAXDELAY 512
#define MASTER_LED_GPIO GPIO_NUM_8
#define MASTER_LED_ON_LEVEL 0
#define MASTER_LED_OFF_LEVEL 1
#define MASTER_LED_BLINK_MS 40
#define BUTTON_LED_GPIO GPIO_NUM_2
#define BUTTON_LED_ON_LEVEL 1
#define BUTTON_LED_OFF_LEVEL 0
#define BUTTON_LED_BLINK_MS 1000
#define MASTER_TRIGGER_GPIO GPIO_NUM_0
#define MASTER_TRIGGER_DEBOUNCE_MS 120
#define MASTER_I2S_BCLK_GPIO GPIO_NUM_4
#define MASTER_I2S_WS_GPIO GPIO_NUM_5
#define MASTER_I2S_DOUT_GPIO GPIO_NUM_1
#define MASTER_I2S_SAMPLE_RATE_HZ 16000
#define MASTER_BUZZER_SAMPLE_RATE_HZ 1000
#define MASTER_BUZZER_NUM_SAMPLES 1000
#define MASTER_BUZZER_DUTY_CENTER 512
#define MASTER_BUZZER_DYNAMIC_PCM_PEAK 32000
#define MASTER_BUZZER_USE_STATIC_HEADER_TABLE 0
#define MASTER_BUZZER_PI 3.14159265358979323846f
#define MASTER_BUZZER_DYNAMIC_FREQ1_HZ 220.0f
#define MASTER_BUZZER_DYNAMIC_FREQ2_HZ 440.0f
#define MASTER_BUZZER_DYNAMIC_T_FS_HZ 1000.0f
#define MASTER_BEEP_DURATION_MS 1000
#define MASTER_I2S_PLAYBACK_SAMPLES ((MASTER_I2S_SAMPLE_RATE_HZ * MASTER_BEEP_DURATION_MS) / 1000)

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;
static TimerHandle_t s_master_led_off_timer = NULL;
static TimerHandle_t s_button_led_off_timer = NULL;
static bool s_master_buzzer_inited = false;
static const uint8_t s_example_fixed_peer_mac[ESP_NOW_ETH_ALEN] = { 0x10, 0x00, 0x3b, 0xcd, 0xd9, 0xbd }; // Green
// static const uint8_t s_example_fixed_peer_mac[ESP_NOW_ETH_ALEN] = { 0x38, 0x44, 0xbe, 0x44, 0x08, 0xcd }; // Blue
static volatile TickType_t s_last_gpio_trigger_tick = 0;
static i2s_chan_handle_t s_master_i2s_tx_chan = NULL;
static int16_t s_master_beep_pcm[MASTER_I2S_PLAYBACK_SAMPLES * 2];
static bool button_pressed = false;
#if MASTER_BUZZER_USE_STATIC_HEADER_TABLE
static const uint16_t s_master_error_beep_samples[MASTER_BUZZER_NUM_SAMPLES] = {
#include "master_error_beep_samples.h"
};
#else
static int16_t s_master_error_beep_samples[MASTER_BUZZER_NUM_SAMPLES];
#endif
static uint8_t s_example_ap_mac[ESP_NOW_ETH_ALEN] = { 0 };
static uint16_t s_example_espnow_seq = 0;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);
static void example_master_led_init(void);
static void example_master_led_blink(void);
static void button_led_init(void);
static void button_blink(void);
static void example_master_gpio_init(void);
static void example_master_buzzer_init(void);
static void example_master_buzzer_prepare_table(void);
static void example_master_buzzer_prepare_pcm(void);
void example_master_buzzer_beep_1s(void);

static void example_master_led_off_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    gpio_set_level(MASTER_LED_GPIO, MASTER_LED_OFF_LEVEL);
}

static void button_led_off_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    gpio_set_level(BUTTON_LED_GPIO, BUTTON_LED_OFF_LEVEL);
}

static void example_master_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MASTER_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(MASTER_LED_GPIO, MASTER_LED_OFF_LEVEL));

    s_master_led_off_timer = xTimerCreate("master_led_off",
                                          pdMS_TO_TICKS(MASTER_LED_BLINK_MS),
                                          pdFALSE,
                                          NULL,
                                          example_master_led_off_timer_cb);
    if (s_master_led_off_timer == NULL) {
        ESP_LOGE(TAG, "Create LED timer fail");
    }
}

static void example_master_led_blink(void)
{
    if (s_master_led_off_timer == NULL) {
        return;
    }

    ESP_ERROR_CHECK(gpio_set_level(MASTER_LED_GPIO, MASTER_LED_ON_LEVEL));
    xTimerStop(s_master_led_off_timer, 0);
    xTimerStart(s_master_led_off_timer, 0);
}

static void button_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(BUTTON_LED_GPIO, BUTTON_LED_OFF_LEVEL));

    s_button_led_off_timer = xTimerCreate("button_led_off",
                                          pdMS_TO_TICKS(BUTTON_LED_BLINK_MS),
                                          pdFALSE,
                                          NULL,
                                          button_led_off_timer_cb);
    if (s_button_led_off_timer == NULL) {
        ESP_LOGE(TAG, "Create button LED timer fail");
    }
}

static void button_blink(void)
{
    if (s_button_led_off_timer == NULL) {
        return;
    }

    ESP_ERROR_CHECK(gpio_set_level(BUTTON_LED_GPIO, BUTTON_LED_ON_LEVEL));
    xTimerStop(s_button_led_off_timer, 0);
    xTimerStart(s_button_led_off_timer, 0);
}

static void IRAM_ATTR example_master_gpio_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    TickType_t now = xTaskGetTickCountFromISR();
    example_espnow_event_t evt = { .id = EXAMPLE_ESPNOW_GPIO_CB };

    if (s_example_espnow_queue == NULL) {
        return;
    }
    if ((now - s_last_gpio_trigger_tick) < pdMS_TO_TICKS(MASTER_TRIGGER_DEBOUNCE_MS)) {
        return;
    }
    s_last_gpio_trigger_tick = now;

    xQueueSendFromISR(s_example_espnow_queue, &evt, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void example_master_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << MASTER_TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err;

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(MASTER_TRIGGER_GPIO, example_master_gpio_isr_handler, NULL));
    ESP_LOGI(TAG, "GPIO0 button enabled (pull-up, active low)");
}

#if MASTER_BUZZER_USE_STATIC_HEADER_TABLE
static int16_t example_master_buzzer_sample_to_pcm(uint16_t sample)
{
    int32_t centered = (int32_t)sample - MASTER_BUZZER_DUTY_CENTER;
    return (int16_t)(centered * 64);
}
#endif

static void example_master_buzzer_init(void)
{
    if (s_master_buzzer_inited) {
        return;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MASTER_I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MASTER_I2S_BCLK_GPIO,
            .ws = MASTER_I2S_WS_GPIO,
            .dout = MASTER_I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
        },
    };

    example_master_buzzer_prepare_table();
    example_master_buzzer_prepare_pcm();
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_master_i2s_tx_chan, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_master_i2s_tx_chan, &std_cfg));
    s_master_buzzer_inited = true;
}

static void example_master_buzzer_prepare_table(void)
{
#if !MASTER_BUZZER_USE_STATIC_HEADER_TABLE
    const float w1 = 2.0f * MASTER_BUZZER_PI * MASTER_BUZZER_DYNAMIC_FREQ1_HZ;
    const float w2 = 2.0f * MASTER_BUZZER_PI * MASTER_BUZZER_DYNAMIC_FREQ2_HZ;

    for (size_t i = 0; i < MASTER_BUZZER_NUM_SAMPLES; i++) {
        float t = (float)i / MASTER_BUZZER_DYNAMIC_T_FS_HZ;
        float x = (sinf(w1 * t) + sinf(w2 * t)) * 0.5f;
        int32_t pcm = (int32_t)((float)MASTER_BUZZER_DYNAMIC_PCM_PEAK * x);

        if (pcm < INT16_MIN) {
            pcm = INT16_MIN;
        } else if (pcm > INT16_MAX) {
            pcm = INT16_MAX;
        }

        s_master_error_beep_samples[i] = (int16_t)pcm;
    }
#endif
}

static void example_master_buzzer_prepare_pcm(void)
{
    for (size_t i = 0; i < MASTER_I2S_PLAYBACK_SAMPLES; i++) {
        uint32_t table_idx = (uint32_t)(((uint64_t)i * MASTER_BUZZER_NUM_SAMPLES) / MASTER_I2S_PLAYBACK_SAMPLES);
        int16_t pcm_sample;

        if (table_idx >= MASTER_BUZZER_NUM_SAMPLES) {
            table_idx = MASTER_BUZZER_NUM_SAMPLES - 1;
        }
#if MASTER_BUZZER_USE_STATIC_HEADER_TABLE
        pcm_sample = example_master_buzzer_sample_to_pcm(s_master_error_beep_samples[table_idx]);
#else
        pcm_sample = s_master_error_beep_samples[table_idx];
#endif
        s_master_beep_pcm[(i * 2)] = pcm_sample;
        s_master_beep_pcm[(i * 2) + 1] = pcm_sample;
    }
}

void example_master_buzzer_beep_1s(void)
{
    size_t bytes_written = 0;
    static const int16_t s_silence_tail[128] = { 0 };
    esp_err_t err;

    if (!s_master_buzzer_inited) {
        example_master_buzzer_init();
    }

    if (s_master_i2s_tx_chan == NULL) {
        return;
    }

    err = i2s_channel_enable(s_master_i2s_tx_chan);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(i2s_channel_write(s_master_i2s_tx_chan,
                                      s_master_beep_pcm,
                                      sizeof(s_master_beep_pcm),
                                      &bytes_written,
                                      pdMS_TO_TICKS(1200)));
    ESP_ERROR_CHECK(i2s_channel_write(s_master_i2s_tx_chan,
                                      s_silence_tail,
                                      sizeof(s_silence_tail),
                                      &bytes_written,
                                      pdMS_TO_TICKS(50)));

    err = i2s_channel_disable(s_master_i2s_tx_chan);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
#if MODE == MODE_TX
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
#else
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
#endif
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
#if MODE == MODE_TX
    ESP_ERROR_CHECK( esp_wifi_get_mac(ESP_IF_WIFI_STA, s_example_ap_mac) );
    ESP_LOGI(TAG, "--- MASTER APP Local STA MAC: "MACSTR, MAC2STR(s_example_ap_mac));
#else
    ESP_ERROR_CHECK( esp_wifi_get_mac(ESP_IF_WIFI_AP, s_example_ap_mac) );
    ESP_LOGI(TAG, "--- MASTER APP Local AP MAC: "MACSTR, MAC2STR(s_example_ap_mac));
#endif

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
#if MODE == MODE_TX
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#else
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    const uint8_t *mac_addr = NULL;
    const uint8_t *des_addr = NULL;

    if (recv_info == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    mac_addr = recv_info->src_addr;
    des_addr = recv_info->des_addr;
    if (mac_addr == NULL || des_addr == NULL) {
        ESP_LOGE(TAG, "Receive cb address error");
        return;
    }

    if (memcmp(des_addr, s_example_ap_mac, ESP_NOW_ETH_ALEN) != 0) {
        ESP_LOGD(TAG, "Drop packet not addressed to this AP, des: "MACSTR, MAC2STR(des_addr));
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    int ret;
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;

    ESP_LOGI(TAG, "Start receiving data");

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGD(TAG, "Send data to "MACSTR", status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                break;
            }

            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                
                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret < 0) {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    break;
                }

                ESP_LOGI(TAG, "Receive %dth data(type=%d, state=%u, magic=%"PRIu32") from: "MACSTR", len: %d",
                         recv_seq, ret, recv_state, recv_magic, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                /* Accept packets from any source MAC by adding peers dynamically. */
                if (!esp_now_is_peer_exist(recv_cb->mac_addr)) {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL) {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESP_IF_WIFI_AP;
                    peer->encrypt = false;
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                    free(peer);
                }

                // Dont send response in Tx mode
                if (MODE == MODE_TX)
                    break;

                if (!button_pressed) {
                    button_pressed = true;
                    memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    example_espnow_data_prepare(send_param);
                    esp_err_t send_ret = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
                    if (send_ret != ESP_OK) {
                        ESP_LOGE(TAG, "Send error: %s (%d)", esp_err_to_name(send_ret), send_ret);
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    example_master_led_blink();
                    button_blink();
                    example_master_buzzer_beep_1s();
                }

                break;
            }

            case EXAMPLE_ESPNOW_GPIO_CB:
            {
                ESP_LOGI(TAG, "GPIO0 button pressed");
                if (MODE == MODE_RX) {
                    button_pressed = false;
                    ESP_LOGI(TAG, "MODE_RX EXIT");
                    break;
                }

                // if (gpio_get_level(MASTER_TRIGGER_GPIO) != 0) {
                //     ESP_LOGI(TAG, "LEVEL 1 EXIT");
                //     break;
                // }

                memcpy(send_param->dest_mac, s_example_fixed_peer_mac, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);
                esp_err_t send_ret = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
                if (send_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Send error: %s (%d)", esp_err_to_name(send_ret), send_ret);
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                } else {
                    ESP_LOGI(TAG, "GPIO0 low -> sent packet to "MACSTR, MAC2STR(send_param->dest_mac));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }
    example_master_gpio_init();

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
#if MODE == MODE_TX
    if (!esp_now_is_peer_exist(s_example_fixed_peer_mac)) {
        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(TAG, "Malloc peer information fail");
            vQueueDelete(s_example_espnow_queue);
            s_example_espnow_queue = NULL;
            esp_now_deinit();
            return ESP_FAIL;
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESP_IF_WIFI_STA;
        peer->encrypt = false;
        memcpy(peer->peer_addr, s_example_fixed_peer_mac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
        free(peer);
    }
#endif
    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 1;
    send_param->magic = esp_random();
    // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
#if MODE == MODE_TX
    memcpy(send_param->dest_mac, s_example_fixed_peer_mac, ESP_NOW_ETH_ALEN);
#else
    memset(send_param->dest_mac, 0, ESP_NOW_ETH_ALEN);
#endif
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2560, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    if (s_master_led_off_timer != NULL) {
        xTimerStop(s_master_led_off_timer, portMAX_DELAY);
        xTimerDelete(s_master_led_off_timer, portMAX_DELAY);
        s_master_led_off_timer = NULL;
    }
    if (s_button_led_off_timer != NULL) {
        xTimerStop(s_button_led_off_timer, portMAX_DELAY);
        xTimerDelete(s_button_led_off_timer, portMAX_DELAY);
        s_button_led_off_timer = NULL;
    }
    gpio_isr_handler_remove(MASTER_TRIGGER_GPIO);
    if (s_master_i2s_tx_chan != NULL) {
        esp_err_t err = i2s_channel_disable(s_master_i2s_tx_chan);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "i2s disable failed: %s", esp_err_to_name(err));
        }
        i2s_del_channel(s_master_i2s_tx_chan);
        s_master_i2s_tx_chan = NULL;
    }
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "Build MODE: %s", MODE_STR);

    example_master_led_init();
    button_led_init();
    example_master_buzzer_init();
    example_wifi_init();
    example_espnow_init();
}
