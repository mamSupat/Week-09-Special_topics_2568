#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2
#define BLINK_PERIOD 1000  // 1 วินาที

void app_main(void) {
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    
    bool led_state = false;
    
    while (1) {
        // สลับสถานะ LED
        led_state = !led_state;
        gpio_set_level(LED_GPIO, led_state);
        
        ESP_LOGI("LED", "LED is %s", led_state ? "ON" : "OFF");
        
        // หน่วงเวลา 1 วินาที
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
    }
}