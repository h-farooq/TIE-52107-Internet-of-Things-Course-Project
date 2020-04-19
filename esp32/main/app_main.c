/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file app_main.c
 * @brief Example application for the ESP32 Frequency Counter component.
 *
 * Includes an example of re-routing a GPIO input to an output, in this case
 * making an external LED show the state of the signal being measured.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "frequency_count.h"
#include "frequency_count.c"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

// #define TAG "app"


#define GPIO_SIGNAL_INPUT (CONFIG_FREQ_SIGNAL_GPIO)
#define GPIO_RMT_GATE     (CONFIG_SAMPLING_WINDOW_GPIO)
#define GPIO_LED          (2) //CONFIG_LED_GPIO)

// internal signals for GPIO constant levels
#define GPIO_CONSTANT_LOW   0x30
#define GPIO_CONSTANT_HIGH  0x38

#define PCNT_UNIT         (0)
#define PCNT_CHANNEL      (PCNT_CHANNEL_0)
#define RMT_CHANNEL       (RMT_CHANNEL_0)
#define RMT_MAX_BLOCKS    (2)   // allow up to 2 * 64 * (2 * 32767) RMT periods in window
#define RMT_CLK_DIV       160   // results in 2us steps (80MHz / 160 = 0.5 MHz
//#define RMT_CLK_DIV       20    // results in 0.25us steps (80MHz / 20 = 4 MHz
//#define RMT_CLK_DIV       1     // results in 25ns steps (80MHz / 2 / 1 = 40 MHz)

#define SAMPLE_PERIOD 1  // seconds

// The counter is signed 16-bit, so maximum positive value is 32767
// The filter is unsigned 10-bit, maximum value is 1023. Use full period of maximum frequency.
// For higher expected frequencies, the sample period and filter must be reduced.

// suitable up to 16,383.5 Hz
#define WINDOW_DURATION 1  // seconds
#define FILTER_LENGTH 1023  // APB @ 80MHz, limits to < 39,100 Hz

// suitable up to 163,835 Hz
//#define WINDOW_LENGTH 0.1  // seconds
//#define FILTER_LENGTH 122  // APB @ 80MHz, limits to < 655,738 Hz

// suitable up to 1,638,350 Hz
//#define WINDOW_LENGTH 0.01  // seconds
//#define FILTER_LENGTH 12  // APB @ 80MHz, limits to < 3,333,333 Hz

// suitable up to 16,383,500 Hz - no filter
//#define WINDOW_LENGTH 0.001  // seconds
//#define FILTER_LENGTH 0  // APB @ 80MHz, limits to < 40 MHz

#define MOTION_DETECTION_THRESHOLD 20

bool isMQTTConnected = false;
esp_mqtt_client_handle_t client;

static void window_start_callback(void)
{
    ESP_LOGI(TAG, "Begin sampling");
    gpio_matrix_in(GPIO_SIGNAL_INPUT, SIG_IN_FUNC228_IDX, false);
}

static void frequency_callback(double hz)
{
    gpio_matrix_in(GPIO_CONSTANT_LOW, SIG_IN_FUNC228_IDX, false);
    ESP_LOGI(TAG, "Frequency %f Hz", hz);
    if(hz > MOTION_DETECTION_THRESHOLD){
        if(isMQTTConnected){
            int msg_id = esp_mqtt_client_publish(client, "motion/test", "1", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        }
        else{
            ESP_LOGE(TAG, "MQTT not connected");
        }
    }
    // else if(hz==0){
    //     if(isMQTTConnected){
    //         int msg_id = esp_mqtt_client_publish(client, "motion/test", "-1", 0, 1, 0);
    //         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    //     }
    //     else{
    //         ESP_LOGE(TAG, "MQTT not connected");
    //     }
    // }
}

static void config_led(void)
{
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // route incoming frequency signal to onboard LED when sampling
    gpio_matrix_out(GPIO_LED, SIG_IN_FUNC228_IDX, false, false);
}
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            isMQTTConnected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            isMQTTConnected = false;
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://94.237.87.91",
        .username = "haseeb",
        .password = "12345"
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    return client;
}
void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // client = mqtt_app_start();
    // while(!isMQTTConnected);
    
    // int msg_id = esp_mqtt_client_publish(client, "motion/test", "-5", 0, 1, 0);
    // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

    // int msg_id = esp_mqtt_client_subscribe(client, "motion/test", 0);
    // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    //LED config
    config_led();

    //Frequency calculations
    frequency_count_configuration_t * config = malloc(sizeof(*config));
    config->pcnt_gpio = GPIO_SIGNAL_INPUT;
    config->pcnt_unit = PCNT_UNIT;
    config->pcnt_channel = PCNT_CHANNEL;
    config->rmt_gpio = GPIO_RMT_GATE;
    config->rmt_channel = RMT_CHANNEL;
    config->rmt_clk_div = RMT_CLK_DIV;
    config->rmt_max_blocks = 2;
    config->sampling_period_seconds = SAMPLE_PERIOD;
    config->sampling_window_seconds = WINDOW_DURATION;
    config->filter_length = FILTER_LENGTH;
    config->window_start_callback = &window_start_callback;
    config->frequency_update_callback = &frequency_callback;

    // task takes ownership of allocated memory
    xTaskCreate(&frequency_count_task_function, "frequency_count_task", 4096, config, configMAX_PRIORITIES-1, NULL);
}

