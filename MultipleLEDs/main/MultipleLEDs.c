#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Define constants for the three LEDs
#define LED1_GPIO GPIO_NUM_2
#define LED2_GPIO GPIO_NUM_4
#define LED3_GPIO GPIO_NUM_5

#define BLINK_PERIOD 1000 // 1 second (1000ms)

static const char *TAG = "LED_CONTROL";

/**
 * @brief Initializes GPIO for all three LEDs
 */
void led_init(void) {
    ESP_LOGI(TAG, "Initializing LEDs on GPIOs %d, %d, and %d", LED1_GPIO, LED2_GPIO, LED3_GPIO);

    // 1. Create GPIO configuration structure
    gpio_config_t io_conf = {0};

    // 2. Set pin bit mask for all three LEDs
    io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO);

    // 3. Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    // 4. Disable pull-up/pull-down (not necessary for output)
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // 5. Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // 6. Apply configuration
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 7. Set initial state (LEDs OFF)
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
    gpio_set_level(LED3_GPIO, 0);

    ESP_LOGI(TAG, "LED initialization completed");
}

/**
 * @brief Task for blinking all three LEDs
 */
void led_blink_task(void *pvParameters) {
    int led_state = 0;
    while (1) {
        // Toggle the state of all LEDs
        led_state = !led_state;

        // Set the level for each LED
        gpio_set_level(LED1_GPIO, led_state);
        gpio_set_level(LED2_GPIO, led_state);
        gpio_set_level(LED3_GPIO, led_state);

        ESP_LOGI(TAG, "LEDs are %s", led_state ? "ON" : "OFF");

        // Delay for the blink period
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD));
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "ESP32 3-LED Control Demo Started");

    // Initialize all LEDs
    led_init();

    // Create LED blink task
    xTaskCreate(
        led_blink_task,         // Function pointer
        "led_blink_task",       // Task name
        2048,                   // Stack size (bytes)
        NULL,                   // Parameters
        5,                      // Priority
        NULL                    // Task handle
    );

    ESP_LOGI(TAG, "LED blink task created");
}