#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"     

#define LED_GPIO GPIO_NUM_2

void app_main(void) {
    
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    
    gpio_set_level(LED_GPIO, 1);

    ESP_LOGI("LED", "LED is ON!");

   
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
