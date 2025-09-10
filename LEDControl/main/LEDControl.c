#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// กำหนดค่าคงที่
#define LED_GPIO    GPIO_NUM_2      // ใช้ GPIO 2
#define BLINK_PERIOD 1000          // 1 วินาที (1000ms)

static const char *TAG = "LED_CONTROL";

/**
 * @brief กำหนดค่าเริ่มต้นสำหรับ GPIO
 */
void led_init(void) {
    ESP_LOGI(TAG, "Initializing LED on GPIO %d", LED_GPIO);
    
    // 1️⃣ สร้าง GPIO configuration structure
    gpio_config_t io_conf = {0};
    
    // 2️⃣ กำหนด pin ที่ต้องการใช้ (bit mask)
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    
    // 3️⃣ กำหนดเป็น output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    
    // 4️⃣ ปิด pull-up/pull-down (ไม่จำเป็นสำหรับ output)
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    // 5️⃣ ปิด interrupt (ไม่ใช้ในการควบคุม LED)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    // 6️⃣ Apply configuration
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // 7️⃣ Set initial state (LED OFF)
    gpio_set_level(LED_GPIO, 0);
    
    ESP_LOGI(TAG, "LED initialization completed");
}

/**
 * @brief ควบคุม LED (เปิด/ปิด)
 * @param state: 1 = LED ON, 0 = LED OFF
 */
void led_control(int state) {
    gpio_set_level(LED_GPIO, state);
    ESP_LOGI(TAG, "LED %s", state ? "ON" : "OFF");
}

/**
 * @brief Task สำหรับกระพริบ LED
 */
void led_blink_task(void *pvParameters) {
    int led_state = 0;
    
    while (1) {
        // สลับสถานะ LED
        led_state = !led_state;
        led_control(led_state);
        
        // หน่วงเวลา
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "ESP32 LED Control Demo Started");
    
    // Initialize LED
    led_init();
    
    // Create LED blink task
    xTaskCreate(
        led_blink_task,          // Function pointer
        "led_blink_task",        // Task name
        2048,                    // Stack size (bytes)
        NULL,                    // Parameters
        5,                       // Priority
        NULL                     // Task handle
    );
    
    ESP_LOGI(TAG, "LED blink task created");
}