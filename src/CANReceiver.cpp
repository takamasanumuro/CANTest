// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "MotorCANManager.h"

static const char* TAG = "CAN";

MotorCANManager motor_can_manager;

void can_receive_task(void* parameter) {
    while (true) {
        twai_message_t message;
        if (twai_receive(&message, pdMS_TO_TICKS(portMAX_DELAY)) == ESP_OK) {
            motor_can_manager.handle_can_frame(message);
        }
    }
}

static void ledBlinker(void *parameter) {

    pinMode(GPIO_NUM_2, OUTPUT);
    while (true) {
        digitalWrite(GPIO_NUM_2, HIGH);
        delay(500);
        digitalWrite(GPIO_NUM_2, LOW);
        delay(500);
    }
}

void simple_reception_test() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        char buffer[256] = {0};
        sprintf(buffer, "Received message: ID=0x%X, Length=%d\n", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(buffer + strlen(buffer), "0x%02X ", message.data[i]);
        }
        strcat(buffer, "\n");
        Serial.print(buffer);
    }
}

void setup() {
    xTaskCreate(ledBlinker, "ledBlinker", 2048, NULL, 1, NULL);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_13, GPIO_NUM_12, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    Serial.begin(115200);
    while (!Serial);

    ESP_LOGI(TAG, "TWAI driver installed and started");
    xTaskCreate(can_receive_task, "CANReceiveTask", 2048, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL); // Delete the task to free up resources
}

