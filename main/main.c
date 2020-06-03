#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_sleep.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>

#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "esp32-ulp-gpio-wakeup";

#define ROTARY_ENCODER_A        (gpio_num_t)33
#define ROTARY_ENCODER_B        (gpio_num_t)32
#define ROTARY_ENCODER_SW       (gpio_num_t)25

static RTC_DATA_ATTR int bootCount = 0;

void app_main(void) {
  if (bootCount++ == 0) {
    // Not really necessary but except that it gives enough time for
    // the first message to be displayed immediately after flashing
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  switch (cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_UNDEFINED ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_ALL : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_ALL ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_EXT0 : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_EXT0 ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_EXT1 : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_EXT1 ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_TIMER : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_TIMER ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_TOUCHPAD : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_TOUCHPAD ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_ULP : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_ULP ***");
      break;
    }

    case ESP_SLEEP_WAKEUP_GPIO : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_GPIO ***");	// Light Sleep Only
      break;
    }

    case ESP_SLEEP_WAKEUP_UART : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_UART ***");	// Light Sleep Only
      break;
    }

    default : {
      ESP_LOGI(TAG, "*** ESP_SLEEP_WAKEUP_??? ***");	// Light Sleep Only
      break;
    }
  }

  gpio_pad_select_gpio(ROTARY_ENCODER_A);
  gpio_pad_select_gpio(ROTARY_ENCODER_B);
  gpio_pad_select_gpio(ROTARY_ENCODER_SW);

  gpio_set_direction(ROTARY_ENCODER_A, GPIO_MODE_INPUT);
  gpio_set_direction(ROTARY_ENCODER_B, GPIO_MODE_INPUT);
  gpio_set_direction(ROTARY_ENCODER_SW, GPIO_MODE_INPUT);

  gpio_set_pull_mode(ROTARY_ENCODER_A, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(ROTARY_ENCODER_B, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(ROTARY_ENCODER_SW, GPIO_PULLUP_ONLY);

  // Deep sleep resumes at begining of app_main
  //esp_bluedroid_disable();
  //esp_bt_controller_disable();
  //esp_wifi_stop();

  ESP_LOGI(TAG, "Entering Deep Sleep");

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  if (err == ESP_OK) {
    /* Set ULP wake up period to 20ms */
    err = ulp_set_wakeup_period(0, 20000);
    if (err == ESP_OK) {
      static gpio_num_t array[] = {
        GPIO_NUM_25,
        GPIO_NUM_32,
        GPIO_NUM_33
      };

      int numItems = sizeof(array) / sizeof(gpio_num_t);
      if (numItems > 0) {
        ulp_rtc_gpio_mask = 0x00;

        for (int i=0;i<numItems;i++) {
          gpio_num_t gpio_num = array[i];

          if (rtc_gpio_is_valid_gpio(gpio_num)) {
            int rtcio_num = rtc_io_number_get(gpio_num);

            /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
            rtc_gpio_init(gpio_num);
            rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
            rtc_gpio_pulldown_dis(gpio_num);
            rtc_gpio_pullup_dis(gpio_num);
            rtc_gpio_hold_en(gpio_num);

            ulp_rtc_gpio_mask |= (1 << rtcio_num);
          }
        }

        if (ulp_rtc_gpio_mask != 0) {
          err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
          if (err == ESP_OK) {
            esp_sleep_enable_ulp_wakeup();
            esp_deep_sleep_start();
          } else {
            ESP_LOGE(TAG, "ulp_run> Error %s", esp_err_to_name(err));
          }
        } else {
          ESP_LOGW(TAG, "ulp_rtc_gpio_mask=0x%04x", ulp_rtc_gpio_mask);
        }
      } else {
        ESP_LOGW(TAG, "numItems=%d", numItems);
      }
    } else {
      ESP_LOGE(TAG, "ulp_set_wakeup_period> Error %s", esp_err_to_name(err));
    }
  } else {
    ESP_LOGE(TAG, "ulp_load_binary> Error %s", esp_err_to_name(err));
  }
}
