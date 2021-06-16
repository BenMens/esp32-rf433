#ifndef _RF433_HPP
#define _RF433_HPP

#include <stdio.h>
#include "driver/rmt.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(RF433_EVENTS);

enum {
    RF433_EVENT_BUTTON_PRESS
};

typedef struct {
  uint32_t addr;
  uint8_t unit;
  uint8_t state;
} rf433_event_t;


class RF433Transmitter {
  public:
    RF433Transmitter(gpio_num_t pin, rmt_channel_t tx_channel);

    void sendCocoClassicCode(uint addr, uint unit, bool state);
    void sendCocoCode(uint addr, uint unit, bool state);
      
  private: 
    rmt_channel_t rmtChannel;
};

class RF433Receiver {
  public:

    RF433Receiver(gpio_num_t pin, rmt_channel_t rx_channel);

  private: 
    static bool decodeCocoClassicCode(
        rmt_item32_t *items, 
        int length, 
        rf433_event_t *event);
    static bool decodeCocoNewCode(
        rmt_item32_t *items, 
        int length,
        rf433_event_t *event);
    static bool normalizePulse(
        uint32_t duration0, 
        uint32_t duration1,
        uint16_t period,
        uint8_t *t0,
        uint8_t *t1);

    static void receiverTask(void *arg);
    TaskHandle_t receiverTaskHandle;

    rmt_channel_t rmtChannel;
};

#endif