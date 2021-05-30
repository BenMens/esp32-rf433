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
  uint32_t data;
} rf433_event_t;


class RF433Transmitter {
  public:
    RF433Transmitter(gpio_num_t pin, rmt_channel_t tx_channel);

    void sendCocoCode(uint addr, uint grp, bool on);
    void sendElroCode(uint addr, uint grp, bool on);
      
  private: 
    rmt_channel_t rmtChannel;
};

class RF433Receiver {
  public:

    RF433Receiver(gpio_num_t pin, rmt_channel_t rx_channel);

  private: 
    static bool cocoDurationValid(uint32_t duration0, uint32_t duration1);
    static uint32_t decodeCocoElroCode(rmt_item32_t *items, int length);
    static void receiverTask(void *arg);
    TaskHandle_t receiverTaskHandle;

    rmt_channel_t rmtChannel;
};

#endif