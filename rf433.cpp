#pragma GCC optimize("O0")

#include "rf433.hpp"

#include <stdio.h>
#include <string.h>

#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

ESP_EVENT_DEFINE_BASE(RF433_EVENTS);

static const char *TAG = "RF433";

// #define COCO_ALPHA_TIME 295
// #define COCONEW_ALPHA_TIME 285
// #define COCONEW_BETA_TIME 285
#define MIN_PULSE_WIDTH (250)
#define MAX_PULSE_WIDTH (4 * 1000 * 1000)

enum coco_encoding_type { COCO_CLASSIC, COCO_NEW };

typedef struct {
    rmt_encoder_t base;
    coco_encoding_type type;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t start_symbol;
    rmt_symbol_word_t sequences[2][2];
    rmt_symbol_word_t end_symbol;
    uint32_t bit;
    int8_t index;
} coco_encoder_t;

IRAM_ATTR static size_t rmt_coco_encode(rmt_encoder_t *encoder,
                                        rmt_channel_handle_t channel,
                                        const void *primary_data,
                                        size_t data_size,
                                        rmt_encode_state_t *ret_state)
{
    coco_encoder_t *coco_encoder = __containerof(encoder, coco_encoder_t, base);

    size_t encodedSymbols = 0;
    int returnState = RMT_ENCODING_RESET;
    uint32_t code = *(uint32_t *)primary_data;

    rmt_encoder_handle_t copy_encoder = coco_encoder->copy_encoder;

    bool shouldYield = false;
    while (!shouldYield) {
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;

        if (coco_encoder->index == -1) {
            if (coco_encoder->type == COCO_NEW) {
                encodedSymbols += copy_encoder->encode(
                    copy_encoder, channel, &coco_encoder->start_symbol,
                    sizeof(rmt_symbol_word_t), &session_state);
            } else {
                session_state = RMT_ENCODING_COMPLETE;
            }

            if (session_state & RMT_ENCODING_COMPLETE) {
                if (coco_encoder->type == COCO_CLASSIC) {
                    coco_encoder->bit = 1;
                    coco_encoder->index = 12;
                } else {
                    coco_encoder->bit = 1 << 31;
                    coco_encoder->index = 32;
                }

                returnState |= RMT_ENCODING_COMPLETE;
                shouldYield = true;
            }

        } else if (coco_encoder->index > 0) {
            if (code & coco_encoder->bit) {
                encodedSymbols += copy_encoder->encode(
                    copy_encoder, channel, &coco_encoder->sequences[1],
                    sizeof(rmt_symbol_word_t) * 2, &session_state);

            } else {
                encodedSymbols += copy_encoder->encode(
                    copy_encoder, channel, &coco_encoder->sequences[0],
                    sizeof(rmt_symbol_word_t) * 2, &session_state);
            }

            if (session_state & RMT_ENCODING_COMPLETE) {
                if (coco_encoder->type == COCO_CLASSIC) {
                    coco_encoder->bit <<= 1;
                } else {
                    coco_encoder->bit >>= 1;
                }

                coco_encoder->index--;
            }

        } else {
            encodedSymbols += copy_encoder->encode(
                copy_encoder, channel, &coco_encoder->end_symbol,
                sizeof(rmt_symbol_word_t), &session_state);

            if (session_state & RMT_ENCODING_COMPLETE) {
                coco_encoder->index = -1;
                returnState |= RMT_ENCODING_COMPLETE;
                shouldYield = true;
            }
        }

        if (session_state & RMT_ENCODING_MEM_FULL) {
            returnState |= RMT_ENCODING_MEM_FULL;
            shouldYield = true;
        }
    }

    *ret_state = (rmt_encode_state_t)returnState;

    return encodedSymbols;
}

static esp_err_t rmt_coco_encoder_reset(rmt_encoder_t *encoder)
{
    coco_encoder_t *coco_encoder = __containerof(encoder, coco_encoder_t, base);

    rmt_encoder_reset(coco_encoder->copy_encoder);
    coco_encoder->index = -1;

    return ESP_OK;
}

static esp_err_t rmt_coco_encoder_delete(rmt_encoder_t *encoder)
{
    coco_encoder_t *coco_encoder = __containerof(encoder, coco_encoder_t, base);

    rmt_del_encoder(coco_encoder->copy_encoder);
    free(coco_encoder);

    return ESP_OK;
}

esp_err_t rmt_new_coco_encoder(gpio_num_t pin, rmt_encoder_handle_t *retEncoder)
{
    esp_err_t ret = ESP_OK;

    coco_encoder_t *cocoEncoder =
        (coco_encoder_t *)calloc(1, sizeof(coco_encoder_t));

    cocoEncoder->base.encode = rmt_coco_encode;
    cocoEncoder->base.del = rmt_coco_encoder_delete;
    cocoEncoder->base.reset = rmt_coco_encoder_reset;
    cocoEncoder->type = COCO_NEW;

    // START = L HHHHHHHHH
    cocoEncoder->start_symbol = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 2700,
        .level1 = 0,
    }};

    // 0 = L l L H
    cocoEncoder->sequences[0][0] = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 200,
        .level1 = 0,
    }};
    cocoEncoder->sequences[0][1] = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 1200,
        .level1 = 0,
    }};

    // 1 = L H L l
    cocoEncoder->sequences[1][0] = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 1200,
        .level1 = 0,
    }};
    cocoEncoder->sequences[1][1] = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 200,
        .level1 = 0,
    }};

    cocoEncoder->end_symbol = {{
        .duration0 = 270,
        .level0 = 1,
        .duration1 = 9000,
        .level1 = 0,
    }};

    rmt_copy_encoder_config_t copy_encoder_config;
    rmt_new_copy_encoder(&copy_encoder_config, &cocoEncoder->copy_encoder);

    rmt_encoder_reset(&cocoEncoder->base);

    *retEncoder = &cocoEncoder->base;

    return ret;
}

esp_err_t rmt_new_classic_coco_encoder(gpio_num_t pin,
                                       rmt_encoder_handle_t *retEncoder)
{
    esp_err_t ret = ESP_OK;

    coco_encoder_t *cocoEncoder =
        (coco_encoder_t *)calloc(1, sizeof(coco_encoder_t));

    cocoEncoder->base.encode = rmt_coco_encode;
    cocoEncoder->base.del = rmt_coco_encoder_delete;
    cocoEncoder->base.reset = rmt_coco_encoder_reset;
    cocoEncoder->type = COCO_CLASSIC;

    // 0 = L H L H
    cocoEncoder->sequences[0][0] = {{
        .duration0 = 400,
        .level0 = 1,
        .duration1 = 1000,
        .level1 = 0,
    }};
    cocoEncoder->sequences[0][1] = {{
        .duration0 = 400,
        .level0 = 1,
        .duration1 = 1000,
        .level1 = 0,
    }};

    // 1 = L H H L
    cocoEncoder->sequences[1][0] = {{
        .duration0 = 400,
        .level0 = 1,
        .duration1 = 1000,
        .level1 = 0,
    }};
    cocoEncoder->sequences[1][1] = {{
        .duration0 = 1000,
        .level0 = 1,
        .duration1 = 400,
        .level1 = 0,
    }};

    // END = L HHHHH
    cocoEncoder->end_symbol = {{
        .duration0 = 400,
        .level0 = 1,
        .duration1 = 10000,
        .level1 = 0,
    }};

    rmt_copy_encoder_config_t copy_encoder_config;
    rmt_new_copy_encoder(&copy_encoder_config, &cocoEncoder->copy_encoder);

    rmt_encoder_reset(&cocoEncoder->base);

    *retEncoder = &cocoEncoder->base;

    return ret;
}

CocoTransmitter::CocoTransmitter(gpio_num_t pin)
{
    rmt_tx_channel_config_t channelConfig = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = 1 * 1000 * 1000,
        .mem_block_symbols = 128,
        .trans_queue_depth = 1,
        .flags =
            {
                .invert_out = false,
                .with_dma = false,
                .io_loop_back = false,
                .io_od_mode = false,
            },
        .intr_priority = 0,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&channelConfig, &txChannel));

    ESP_ERROR_CHECK(rmt_enable(txChannel));

    ESP_ERROR_CHECK(rmt_new_classic_coco_encoder(pin, &classicEncoder));

    ESP_ERROR_CHECK(rmt_new_coco_encoder(pin, &newEncoder));
}

void CocoTransmitter::sendCocoCode(uint addr, uint unit, bool state,
                                   int numTransmissions)
{
    static uint32_t code;

    code = addr << 6 | (state & 0b1) << 4 | (unit & 0b1111);

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags =
            {
                .eot_level = 0,
            },
    };

    for (int i = numTransmissions; i > 0; i--) {
        rmt_transmit(txChannel, this->newEncoder, &code, sizeof(code),
                     &transmit_config);
        if (i > 1) {
            rmt_tx_wait_all_done(txChannel, 2000);
        }
    }
}

void CocoTransmitter::sendCocoClassicCode(uint addr, uint unit, bool state,
                                          int numTransmissions)
{
    uint32_t code;

    code =
        (state ? 0b1110 : 0b0110) << 8 | (unit & 0b1111) << 4 | (addr & 0b1111);

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags =
            {
                .eot_level = 0,
            },
    };

    for (int i = numTransmissions; i > 0; i--) {
        rmt_transmit(txChannel, this->classicEncoder, &code, sizeof(code),
                     &transmit_config);
        if (i > 1) {
            rmt_tx_wait_all_done(txChannel, 2000);
        }
    }
}

void CocoTransmitter::waitTillWriteCompletes()
{
    rmt_tx_wait_all_done(txChannel, 2000);
}

CocoReceiver::CocoReceiver(gpio_num_t pin)
{
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1 * 1000 * 1000,
        .mem_block_symbols = 128,
        .flags =
            {
                .invert_in = false,
                .with_dma = false,
                .io_loop_back = false,
            },
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    ESP_LOGI(TAG, "register RX done callback");
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rxDoneCallback,
    };
    ESP_ERROR_CHECK(
        rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue));

    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    xTaskCreatePinnedToCore(CocoReceiver::receiverTask, "rf433_rx_task", 4096,
                            this, 10, &receiverTaskHandle, 0);
}

void CocoReceiver::receiverTask(void *arg)
{
    CocoReceiver *receiver = (CocoReceiver *)arg;

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = MIN_PULSE_WIDTH,
        .signal_range_max_ns = MAX_PULSE_WIDTH,
    };

    rmt_symbol_word_t raw_symbols[80];
    rmt_rx_done_event_data_t rx_data;
    ESP_ERROR_CHECK(rmt_receive(receiver->rx_chan, raw_symbols,
                                sizeof(raw_symbols), &receive_config));
    while (1) {
        if (xQueueReceive(receiver->receive_queue, &rx_data,
                          pdMS_TO_TICKS(1000)) == pdPASS) {
            coco_event_t event;

            ESP_ERROR_CHECK(rmt_receive(receiver->rx_chan, raw_symbols,
                                        sizeof(raw_symbols), &receive_config));

            if (decodeCocoNewCode(rx_data.received_symbols, rx_data.num_symbols,
                                  &event)) {
                esp_event_post(RF433_EVENTS, RF433_EVENT_BUTTON_PRESS, &event,
                               sizeof(event), 100);
            }

            if (decodeCocoClassicCode(rx_data.received_symbols,
                                      rx_data.num_symbols, &event)) {
                esp_event_post(RF433_EVENTS, RF433_EVENT_BUTTON_PRESS, &event,
                               sizeof(event), 100);
            }
        }
    }
}

IRAM_ATTR bool CocoReceiver::rxDoneCallback(
    rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    QueueHandle_t receive_queue = (QueueHandle_t)user_data;

    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

bool CocoReceiver::normalizeClassicPulse(uint32_t duration0, uint32_t duration1,
                                         uint8_t *t0, uint8_t *t1)
{
    if (duration0 < 200 || duration1 < 200) {
        return false;
    }

    if (duration0 < duration1) {
        *t0 = 1;

        *t1 = duration1 / duration0;
        if (*t1 > 1 && *t1 < 4) {
            *t1 = 2;
        } else if (*t1 >= 4) {
            return false;
        }

    } else {
        *t1 = 1;

        *t0 = duration0 / duration1;
        if (*t0 > 1 && *t0 < 4) {
            *t0 = 2;
        } else if (*t0 >= 4) {
            return false;
        }
    }

    return true;
}

bool CocoReceiver::normalizePulse(uint32_t duration0, uint32_t duration1,
                                  uint8_t *t0, uint8_t *t1)
{
    if (duration0 < 200 || duration1 < 200) {
        return false;
    }

    if (duration0 < duration1) {
        *t0 = 1;

        *t1 = duration1 / duration0;
        if (*t1 == 1) {
        } else if (*t1 > 1 && *t1 < 8) {
            *t1 = 2;
        } else if (*t1 < 12) {
            *t1 = 10;
        } else {
            return false;
        }
    } else {
        *t1 = 1;

        *t0 = duration0 / duration1;
        if (*t0 == 1) {
        } else if (*t0 > 1 && *t0 < 8) {
            *t0 = 2;
        } else if (*t0 < 12) {
            *t0 = 10;
        } else {
            return false;
        }
    }

    return true;
}

bool CocoReceiver::decodeCocoClassicCode(rmt_symbol_word_t *items, int length,
                                         coco_event_t *event)
{
    uint32_t code = 0;

    if (length != 25) {
        return false;
    }

    for (int i = 0; i < (length - 1); i += 2) {
        uint8_t t0, t1, t2, t3;
        bool p0Valid, p1Valid;

        p0Valid = normalizeClassicPulse(items[i].duration0, items[i].duration1,
                                        &t0, &t1);

        p1Valid = normalizeClassicPulse(items[i + 1].duration0,
                                        items[i + 1].duration1, &t2, &t3);

        if (p0Valid && p1Valid) {
            if (t0 == 1 && t1 == 2 && t2 == 1 && t3 == 2) {
                code >>= 1;
            } else if (t0 == 1 && t1 == 2 && t2 == 2 && t3 == 1) {
                code >>= 1;
                code |= (1 << 11);
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    event->press.state = ((code >> 8) & 0b1111) == 14 ? 1 : 0;
    event->press.unit = (code >> 4) & 0b1111;
    event->press.addr = code & 0b1111;

    return true;
}

bool CocoReceiver::decodeCocoNewCode(rmt_symbol_word_t *items, int length,
                                     coco_event_t *event)
{
    uint32_t code = 0;
    uint8_t t0, t1, t2, t3;
    bool p0Valid, p1Valid;

    if (length != 66) {
        return false;
    }

    // for (int i = 0; i < 66; i++) {
    //     ESP_LOGI(TAG, "pulse %u %u %u %u", items[i].duration0, items[i].level0, items[i].duration1, items[i].level1);
    // }

    p0Valid = normalizePulse(items[0].duration0, items[0].duration1, &t0, &t1);

    if (!p0Valid || t0 != 1 || t1 != 10) {
        return false;
    }

    for (int i = 1; i < (length - 1); i += 2) {
        p0Valid =
            normalizePulse(items[i].duration0, items[i].duration1, &t0, &t1);

        p1Valid = normalizePulse(items[i + 1].duration0, items[i + 1].duration1,
                                 &t2, &t3);

        if (p0Valid && p1Valid) {
            if (t0 == 1 && t1 == 2 && t2 == 1 && t3 == 1) {
                code <<= 1;
                code |= 1;
            } else if (t0 == 1 && t1 == 1 && t2 == 1 && t3 == 2) {
                code <<= 1;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    event->press.addr = (code >> 6);
    event->press.unit = code & 0b111;
    event->press.state = (code >> 3) & 0x07;

    uint32_t bit = 1;
    for (int i = 31; i >= 0; i--) {
        bit <<= 1;
    }

    return true;
}
