#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rf433.hpp"

ESP_EVENT_DEFINE_BASE(RF433_EVENTS);

static const char *TAG = "RF433";

#define COCO_T_SHORT_1_TIME  500
#define COCO_T_LONG_1_TIME  1000
#define COCO_T_SHORT_0_TIME  400
#define COCO_T_LONG_0_TIME   900


// KaKu
//          on  grp  adr
// A1 On  1110 0000 0000
// A1 Off 0110 0000 0000
// A2 On  1110 0001 0000
// A2 Off 0110 0001 0000
// A3 On  1110 0010 0000
// A4 Off 0110 0010 0000

// B1 On  1110 0000 0001
// B1 Off 0110 0000 0001
// B2 On  1110 0001 0001
// B2 Off 0110 0001 0001
// B3 On  1110 0010 0001
// B4 Off 0110 0010 0001

// C1 On  1110 0000 0010
// C1 Off 0110 0000 0010
// C2 On  1110 0001 0010
// C2 Off 0110 0001 0010
// C3 On  1110 0010 0010
// C4 Off 0110 0010 0010

// D1 On  1110 0000 0011
// D1 Off 0110 0000 0011
// D2 On  1110 0001 0011
// D2 Off 0110 0001 0011
// D3 On  1110 0010 0011
// D4 Off 0110 0010 0011
        

// Elro recieved (note: invert for sending like (true 00001 00111))
//
// A  On  10 11110 11000
// A  Off 01 11110 11000
// B  On  10 11101 11000
// B  Off 01 11101 11000
// C  On  10 11011 11000
// C  Off 01 11011 11000
// D  On  10 10111 11000
// D  Off 01 10111 11000


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
     

void RF433Transmitter::sendCocoCode(uint addr, uint grp, bool on) {

  rmt_item32_t items[26];
  uint32_t code;

  code = (on ? 0b1110: 0b0110) << 8
         | (grp & 0b1111) << 4
         | (addr & 0b1111);

  ESP_LOGI( TAG, 
            "coco send: %s%s%s%s%s%s%s%s%s%s%s%s", 
            (code & 1 << 11) ? "1" : "0",
            (code & 1 << 10) ? "1" : "0",
            (code & 1 <<  9) ? "1" : "0",
            (code & 1 <<  8) ? "1" : "0",
            (code & 1 <<  7) ? "1" : "0",
            (code & 1 <<  6) ? "1" : "0",
            (code & 1 <<  5) ? "1" : "0",
            (code & 1 <<  4) ? "1" : "0",
            (code & 1 <<  3) ? "1" : "0",
            (code & 1 <<  2) ? "1" : "0",
            (code & 1 <<  1) ? "1" : "0",
            (code & 1 <<  0) ? "1" : "0"
          );
  // start
  items[0].duration0 = 12 * COCO_T_SHORT_0_TIME;
  items[0].level0 = 0; 
  items[0].duration1 = 12 * COCO_T_SHORT_0_TIME;
  items[0].level1 = 0; 

  // bits
  uint32_t bit = 1;
  for (int i = 0 ; i < 12; i++) {
    if (code & bit) {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = COCO_T_LONG_0_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCO_T_LONG_1_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCO_T_SHORT_0_TIME;
    } else {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = COCO_T_LONG_0_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCO_T_LONG_0_TIME;
    }

    bit <<= 1;
  }

  // stop
  items[25].level0 = 1; 
  items[25].duration0 = COCO_T_SHORT_1_TIME;
  items[25].level1 = 0; 
  items[25].duration1 = 0;

  for (int i = 1; i < 10; i++) {
    rmt_write_items(rmtChannel, items, 26, true);
  }
}

void RF433Transmitter::sendElroCode(uint addr, uint grp, bool on) {

  rmt_item32_t items[26];
  uint32_t code;

  code = (on ? 0b01: 0b10) << 10
         | (grp  & 0b11111) << 5
         | (addr & 0b11111);

  code ^= 0xffffffff;

  ESP_LOGI( TAG, 
            "elro send: %s%s%s%s%s%s%s%s%s%s%s%s", 
            (code & 1 << 11) ? "1" : "0",
            (code & 1 << 10) ? "1" : "0",
            (code & 1 <<  9) ? "1" : "0",
            (code & 1 <<  8) ? "1" : "0",
            (code & 1 <<  7) ? "1" : "0",
            (code & 1 <<  6) ? "1" : "0",
            (code & 1 <<  5) ? "1" : "0",
            (code & 1 <<  4) ? "1" : "0",
            (code & 1 <<  3) ? "1" : "0",
            (code & 1 <<  2) ? "1" : "0",
            (code & 1 <<  1) ? "1" : "0",
            (code & 1 <<  0) ? "1" : "0"
          );

  // start
  items[0].duration0 = 12 * COCO_T_SHORT_0_TIME;
  items[0].level0 = 0; 
  items[0].duration1 = 12 * COCO_T_SHORT_0_TIME;
  items[0].level1 = 0; 

  // bits
  uint32_t bit = 1;
  for (int i = 0 ; i < 12; i++) {
    if (code & bit) {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = COCO_T_LONG_0_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCO_T_LONG_1_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCO_T_SHORT_0_TIME;
    } else {
      items[i * 2 + 1].level0 = 1; 
      items[i * 2 + 1].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 1].level1 = 0; 
      items[i * 2 + 1].duration1 = COCO_T_LONG_0_TIME;
      items[i * 2 + 2].level0 = 1; 
      items[i * 2 + 2].duration0 = COCO_T_SHORT_1_TIME;
      items[i * 2 + 2].level1 = 0; 
      items[i * 2 + 2].duration1 = COCO_T_LONG_0_TIME;
    }

    bit <<= 1;
  }

  // stop
  items[25].level0 = 1; 
  items[25].duration0 = COCO_T_SHORT_1_TIME;
  items[25].level1 = 0; 
  items[25].duration1 = 0;

  for (int i = 1; i < 10; i++) {
    rmt_write_items(rmtChannel, items, 26, true);
  }
}

RF433Receiver::RF433Receiver(gpio_num_t pin, rmt_channel_t rx_channel) 
  : rmtChannel(rx_channel)
{
  rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(pin, rmtChannel);
  rmt_rx_config.rx_config.idle_threshold = 2000;
  rmt_rx_config.rx_config.filter_en = true;
  rmt_rx_config.rx_config.filter_ticks_thresh = 255;
  rmt_config(&rmt_rx_config);
  rmt_driver_install(rmtChannel, 500, 0);

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

  while (rb) {
    items = (rmt_item32_t *)xRingbufferReceive(rb, &length, 10);

    if (items) {
      length /= sizeof(rmt_item32_t);
      bool valid = true;

      // if (length == 25) {
      //   for (int i = 0; i < length; i++) {
      //     if (!coco_duration_valid(items[i].duration0, items[i].duration1)) {
      //       ESP_LOGW( TAG, 
      //         "rx_433_received %2d - (%1d, %6d), (%1d, %6d)",
      //         i,
      //         items[i].level0,
      //         items[i].duration0,
      //         items[i].level1,
      //         items[i].duration1);

      //     } else {
      //       ESP_LOGI( TAG, 
      //         "rx_433_received %2d - (%1d, %6d), (%1d, %6d)",
      //         i,
      //         items[i].level0,
      //         items[i].duration0,
      //         items[i].level1,
      //         items[i].duration1);
      //     }
      //   }
      // }
 
      for (int i = 0; i < length - 1; i++) {
        if (!cocoDurationValid(items[i].duration0, items[i].duration1)) {
          valid = false;
          break;
        }
      }

      if (valid) {
        decodeCocoElroCode(items, length);
      }

      //after parsing the data, return spaces to ringbuffer.
      vRingbufferReturnItem(rb, (void *)items);
    }
  }
  
  rmt_driver_uninstall(receiver->rmtChannel);
  vTaskDelete(NULL);
}


bool RF433Receiver::cocoDurationValid(uint32_t duration0, 
                                      uint32_t duration1) {
  if (duration0 == 0) {
    return false;
  }

  if (duration1 == 0) {
    return false;
  }

  if ((duration0 + duration1) < 1000) {
    return false;
  }

  if ((duration0 + duration1) > 1600) {
    return false;
  }

  if (duration0 - duration1 > 200) {
    return true;
  } 
  
  if (duration1 - duration0 > 200) {
    return true;
  } 
  
  return false;
}

uint32_t RF433Receiver::decodeCocoElroCode(rmt_item32_t *items, int length) 
{
  uint32_t result = 0;
  uint32_t currentBit = 1;

  if (length != 25) {
    return -1;
  }

  for (int i = 0 ; i < (length - 1); i += 2) {
    uint8_t nibble = 0;

    if (cocoDurationValid(items[i].duration0, items[i].duration1)) {
      if (items[i].duration0 > items[i].duration1) {
        nibble |= 0b10;
      } else {
        nibble |= 0b01;
      }
    } else {
      return -1;
    }

    nibble <<= 2;

    if (cocoDurationValid(items[i + 1].duration0, items[i + 1].duration1)) {
      if (items[i + 1].duration0 > items[i + 1].duration1) {
        nibble |= 0b10;
      } else {
        nibble |= 0b01;
      }
    } else {
      return -1;
    }

    if (nibble == 0b0110) {
      result |= currentBit;
      currentBit <<= 1;
    } else if (nibble == 0b0101) {
      currentBit <<= 1;
    } else {
      return -1;
    }
  }

  rf433_event_t event = {
    .data = result
  };

  esp_event_post( RF433_EVENTS, 
                  RF433_EVENT_BUTTON_PRESS, 
                  &event, 
                  sizeof(event), 
                  100);

  return result;
}
