/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "manchester_encoder.h"
#include "led_strip.h"

#define TX_PIN 18
#define RX_PIN 19

#define TICK_RESOLUTION 80 * 1000 * 1000 // 50MHz resolution

static const char *TAG = "Manchester Encoding Recieve";

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static uint32_t parse_data_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {
    
    // printf("%d symbols: NEC frame start---\r\n", symbol_num);
    // int e = 0;
    // for (size_t i = 0; i < symbol_num; i++) {
    //     e += rmt_nec_symbols[i].duration0 + rmt_nec_symbols[i].duration1;
    //     printf("{%d:%d ticks},{%d:%d ticks}, e: %d\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
    //            rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1, e);
    // }
    // printf("---NEC frame end: %d ticks\r\n", e);

    int ts = 18;
    int c = 0;
    int i = 0;
    bool data[36] = { false };
    int dataIndex = 0;

    while(c < 567) {
        c += rmt_nec_symbols[i].duration0;

        if(c > ts) {
            // printf("Sampled, Duration 0, i: %d\r\n", i);
            if(rmt_nec_symbols[i].level0 == 1)
                data[dataIndex++] = false;
            else
                data[dataIndex++] = true;

            ts += 16;
        }

        c += rmt_nec_symbols[i].duration1;

        if(c > ts) {
            
            
            if(rmt_nec_symbols[i].level1 == 0)
                data[dataIndex++] = true;
            else
                data[dataIndex++] = false;

            ts += 16;
        }

        i++;
    }

    // printf("0b");
    uint32_t dataDecoded = 0;
    for(int i = 32; i > 0; i--) {
        // printf("%d", (data[i])? 1 : 0);
        dataDecoded |= ((data[i])? 1 << (i - 1) : 0 << (i - 1));
    }

    ESP_LOGI("parse_data_frame", "Recieved frame: 0x%lx", dataDecoded);


    return(dataDecoded);
}

void clearLED() {
    led_strip_handle_t led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = 48,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

void app_main(void) {

    clearLED();

    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = TICK_RESOLUTION,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = RX_PIN,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 80,
        .signal_range_max_ns = 370,
    };

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;

    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    while(1) {
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(500)) == pdPASS) {
            // parse the receive symbols and print the result
            parse_data_frame(rx_data.received_symbols, rx_data.num_symbols);

            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }
    }


}
