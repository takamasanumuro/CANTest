#include <Arduino.h>
#include "LedBlinker.h"
QueueHandle_t can_blink_queue = NULL;

void led_blinker_init() {
    can_blink_queue = xQueueCreate(1, sizeof(uint8_t));
    if (can_blink_queue == NULL) {
        Serial.println("Failed to create CAN blink queue");
        return;
    }
    xTaskCreate(can_led_task, "CAN_LED_Task", 2048, NULL, 1, NULL);
}

void led_blinker_push() {
    uint8_t dummy = 0;
    if (can_blink_queue == NULL) {
        Serial.println("CAN blink queue not initialized");
        return;
    }
    xQueueSend(can_blink_queue, &dummy, 0); // Send a dummy value to the queue
}

void can_led_task(void* arg) {
    pinMode(CAN_LED_GPIO, OUTPUT);
    uint8_t dummy = 0;
    while (true) {  
        if (xQueueReceive(can_blink_queue, &dummy, portMAX_DELAY) == pdTRUE) {
            digitalWrite(CAN_LED_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(CAN_LED_GPIO, LOW);
        }
    }
}