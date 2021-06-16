#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rf433.hpp"

ESP_EVENT_DEFINE_BASE(RF433_EVENTS);

static const char *TAG = "RF433";

#define COCO_ALPHA_TIME    378
#define COCONEW_ALPHA_TIME 260


RF433Transmitter::RF433Transmitter(gpio_num_t pin, rmt_channel_t tx_channel) 
  : rmtChannel(tx_channel)
{
  static rmt_config_t rmt_tx_443_config = 
      RMT_DEFAULT_CONFIG_TX(pin, rmtChannel);

  rmt_tx_443_config.mem_block_num = 1;
  rmt_tx_443_config.tx_config.loop_en = false;
  rmt_tx_443_config.tx_config.carrier_en = false;
  rmt_tx_443_config.tx_config.idle_output_en = true;
  rmt_tx_443_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  rmt_config(&rmt_tx_443_config);
  rmt_driver_install(rmtChannel, 1000, 0);
}
     

void RF433Transmitter::sendCocoCode(uint addr, uint unit, bool state) {

  rmt_item32_t items[67];
  uint32_t code;

  code = addr << 6
         | (state & 0b1) << 4
         | (unit & 0b1111);

  char bitString[33];

  uint32_t bit = 1; 
  for (int i = 31; i >= 0; i--) {
    bitString[i] = (code & bit) ? '1' : '0';
    bit <<= 1;
  }
  bitString[32] = 0;

  ESP_LOGI( TAG, "coco send: %s", bitString);
            
  // start
  items[0].duration0 = 1 * COCONEW_ALPHA_TIME;
  items[0].level0 = 1; 
  items[0].duration1 = 10 * COCONEW_ALPHA_TIME;
  items[0].level1 = 0; 

  // bits
  bit = 1 << 31;
  for (int i = 0 ; i < 32; i++) {
    if (code & bit) {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCONEW_ALPHA_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = 5 * COCONEW_ALPHA_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCONEW_ALPHA_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCONEW_ALPHA_TIME;
    } else {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCONEW_ALPHA_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = COCONEW_ALPHA_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCONEW_ALPHA_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = 5 * COCONEW_ALPHA_TIME;
    }

    bit >>= 1;
  }

  // end transmission
  items[65].level0 = 1; 
  items[65].duration0 = COCONEW_ALPHA_TIME;
  items[65].level1 = 0; 
  items[65].duration1 = 40 * COCONEW_ALPHA_TIME;
  items[66].level0 = 1; 
  items[66].duration0 = 0;
  items[66].level1 = 0; 
  items[66].duration1 = 0;

  for (int i = 0; i < 10; i++) {
    rmt_write_items(rmtChannel, items, 67, true);
  }
}

void RF433Transmitter::sendCocoClassicCode(uint addr, uint unit, bool state) {

  rmt_item32_t items[27];
  uint32_t code;

  code = (state ? 0b1110: 0b0110) << 8
         | (unit & 0b1111) << 4
         | (addr & 0b1111);

  char bitString[33];

  uint32_t bit = 1; 
  for (int i = 31; i >= 0; i--) {
    bitString[i] = (code & bit) ? '1' : '0';
    bit <<= 1;
  }
  bitString[32] = 0;

  ESP_LOGI( TAG, "coco send: %s", bitString);

  // start
  items[0].duration0 = 1 * COCO_ALPHA_TIME;
  items[0].level0 = 1; 
  items[0].duration1 = 31 * COCO_ALPHA_TIME;
  items[0].level1 = 0; 

  // bits
  bit = 1;
  for (int i = 0 ; i < 12; i++) {
    if (code & bit) {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_ALPHA_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = 3 * COCO_ALPHA_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = 3 * COCO_ALPHA_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCO_ALPHA_TIME;
    } else {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_ALPHA_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = 3 * COCO_ALPHA_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCO_ALPHA_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = 3 * COCO_ALPHA_TIME;
    }

    bit <<= 1;
  }

  // end transmission
  items[25].level0 = 0; 
  items[25].duration0 = 0;
  items[25].level1 = 0; 
  items[25].duration1 = 0;

  for (int i = 0; i < 10; i++) {
    rmt_write_items(rmtChannel, items, 26, true);
  }
}


RF433Receiver::RF433Receiver(gpio_num_t pin, rmt_channel_t rx_channel) 
  : rmtChannel(rx_channel)
{
  rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(pin, rmtChannel);
  rmt_rx_config.mem_block_num = 2;
  rmt_rx_config.rx_config.idle_threshold = 8000;
  rmt_rx_config.rx_config.filter_en = true;
  rmt_rx_config.rx_config.filter_ticks_thresh = 255;
  rmt_config(&rmt_rx_config);
  rmt_driver_install(rmtChannel, 1000, 0);

  rmt_rx_start(rmtChannel, true);

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  xTaskCreatePinnedToCore(RF433Receiver::receiverTask, 
                          "rf433_rx_task", 
                          2048, 
                          this, 
                          10, 
                          &receiverTaskHandle, 
                          chip_info.cores - 1);
}


void RF433Receiver::receiverTask(void *arg) {
  RF433Receiver *receiver = (RF433Receiver *)arg;

  RingbufHandle_t rb = NULL;
  uint32_t length = 0;
  rmt_item32_t *items = NULL;

  rmt_get_ringbuf_handle(receiver->rmtChannel, &rb);

  ESP_LOGI(TAG, "length %d", length);

  while (rb) {
    items = (rmt_item32_t *)xRingbufferReceive(rb, &length, 1000);

    if (items) {
      length /= sizeof(rmt_item32_t);

      rf433_event_t event;

      if (decodeCocoNewCode(items, length, &event)) {
        esp_event_post( RF433_EVENTS, 
                        RF433_EVENT_BUTTON_PRESS, 
                        &event, 
                        sizeof(event), 
                        100);
      }

      if (decodeCocoClassicCode(items, length, &event)) {
        esp_event_post( RF433_EVENTS, 
                        RF433_EVENT_BUTTON_PRESS, 
                        &event, 
                        sizeof(event), 
                        100);
      }

      //after parsing the data, return spaces to ringbuffer.
      vRingbufferReturnItem(rb, (void *)items);
    }
  }
  
  rmt_driver_uninstall(receiver->rmtChannel);
  vTaskDelete(NULL);
}

bool RF433Receiver::normalizePulse(
    uint32_t duration0, 
    uint32_t duration1,
    uint16_t period,
    uint8_t *t0,
    uint8_t *t1
    ) {

  // allow 20% offset
  uint16_t min_period = period - (period / 5);
  uint16_t max_period = period + (period / 5);

  if ((duration0 > min_period && duration0 < max_period) 
      || (duration1 > min_period && duration1 < max_period)) {

    duration0 = (duration0 * 10) / period;
    *t0 = duration0 / 10;
    if (duration0 % 10 > 5) {
      *t0 += 1;
    }

    duration1 = (duration1 * 10) / period;
    *t1 = duration1 / 10;
    if (duration1 % 10 > 5) {
      *t1 += 1;
    }

    if (*t0 == 0 || *t1 == 0 ) {
      return false;
    }

    if (*t0 <= 40 && *t1 <= 40) {
      return true;
    }
  }

   return false;
}



bool RF433Receiver::decodeCocoClassicCode(
    rmt_item32_t *items, 
    int length, 
    rf433_event_t *event) 
{
  uint32_t code = 0;

  if (length != 25) {
    return false;
  }

  for (int i = 0 ; i < (length - 1); i += 2) {
    uint8_t t0, t1, t2, t3;
    bool p0Valid, p1Valid;

    p0Valid = normalizePulse(
        items[i].duration0, 
        items[i].duration1,
        320,
        &t0,
        &t1);

    p1Valid = normalizePulse(
        items[i + 1].duration0, 
        items[i + 1].duration1,
        320,
        &t2,
        &t3);

    if (p0Valid && p1Valid) {
      if (t0 == 1 && t1 == 3  && t2 == 1 && t3 == 3) {
        code >>= 1;
      } else if (t0 == 1 && t1 == 3  && t2 == 3 && t3 == 1) {
        code >>= 1;
        code |= (1 << 11);
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  event->state = ((code >> 8) & 0b1111) == 14 ? 1 : 0;
  event->unit = (code >> 4) & 0b1111;
  event->addr = code & 0b1111;

  char bitString[33];

  uint32_t bit = 1; 
  for (int i = 31; i >= 0; i--) {
    bitString[i] = (code & bit) ? '1' : '0';
    bit <<= 1;
  }
  bitString[32] = 0;

  ESP_LOGI(TAG, "received COCO CLASSIC %s", bitString);

  return true;
}


bool RF433Receiver::decodeCocoNewCode(
    rmt_item32_t *items, 
    int length, 
    rf433_event_t *event) 
{
  uint32_t code = 0;
  uint8_t t0, t1, t2, t3;
  bool p0Valid, p1Valid;


  if (length != 66) {
    return false;
  }

  p0Valid = normalizePulse(
    items[0].duration0, 
    items[0].duration1,
    COCONEW_ALPHA_TIME,
    &t0,
    &t1);

  if (!p0Valid || t0 != 1 || t1 != 10) {
    return false;
  }

  for (int i = 1 ; i < (length - 1); i += 2) {
    p0Valid = normalizePulse(
        items[i].duration0, 
        items[i].duration1,
        COCONEW_ALPHA_TIME,
        &t0,
        &t1);

    p1Valid = normalizePulse(
        items[i + 1].duration0, 
        items[i + 1].duration1,
        COCONEW_ALPHA_TIME,
        &t2,
        &t3);

    if (p0Valid && p1Valid) {
      if (t0 == 1 && t1 == 5  && t2 == 1 && t3 == 1) {
        code <<= 1;
        code |= 1;
      } else if (t0 == 1 && t1 == 1  && t2 == 1 && t3 == 5) {
        code <<= 1;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  event->addr = (code >> 6);
  event->unit = code & 0b111;
  event->state = (code >> 4) & 0b1;

  char bitString[33];

  uint32_t bit = 1; 
  for (int i = 31; i >= 0; i--) {
    bitString[i] = (code & bit) ? '1' : '0';
    bit <<= 1;
  }
  bitString[32] = 0;

  ESP_LOGI(TAG, "received COCO NEW %s", bitString);

  return true;
}
