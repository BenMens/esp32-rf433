#ifndef _RF433_HPP
#define _RF433_HPP

#include <stdio.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(RF433_EVENTS);

enum { RF433_EVENT_BUTTON_PRESS, RF433_EVENT_RECEIVED };

typedef union {
    struct {
        uint32_t addr;
        uint8_t unit;
        uint8_t state;
    } press;
    struct {
        rmt_symbol_word_t symbol;
    } received;

} rf433_event_t;

class RF433Transmitter
{
   public:
    RF433Transmitter(gpio_num_t pin, rmt_channel_handle_t tx_channel);

    void sendCocoClassicCode(uint addr, uint unit, bool state);
    void sendCocoCode(uint addr, uint unit, bool state);

   private:
    rmt_channel_handle_t rmtChannel;
};

class RF433Receiver
{
   public:
    RF433Receiver(gpio_num_t pin);

   private:
    static bool decodeCocoClassicCode(rmt_symbol_word_t *items, int length,
                                      rf433_event_t *event);
    static bool decodeCocoNewCode(rmt_symbol_word_t *items, int length,
                                  rf433_event_t *event);
    static bool normalizePulse(uint32_t duration0, uint32_t duration1,
                               uint16_t period, uint8_t *t0, uint8_t *t1);

    static bool rxDoneCallback(rmt_channel_handle_t channel,
                               const rmt_rx_done_event_data_t *edata,
                               void *user_data);

    static void receiverTask(void *arg);
    TaskHandle_t receiverTaskHandle;

    rmt_channel_handle_t rx_chan;
    QueueHandle_t receive_queue;
};

#endif