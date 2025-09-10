#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2

void app_main(void) {
    
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    
    
    gpio_set_level(LED_GPIO, 1);
    ESP_LOGI("LED", "LED is ON for 3 seconds...");
    
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI("LED", "LED is OFF!");
    
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}