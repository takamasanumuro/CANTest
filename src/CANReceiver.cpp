// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "MotorCANManager.h"
#include "BMSCANManager.h"
#include "LedBlinker.h"


static const char* TAG = "CAN";

MotorCANManager motor_can_manager;
BMSCANManager bms_can_manager;

void can_receive_task(void* parameter) {
    while (true) {
        twai_message_t message;
        if (twai_receive(&message, pdMS_TO_TICKS(portMAX_DELAY)) != ESP_OK) continue;

        led_blinker_push(); // Blink the LED on CAN message reception
        if (bms_can_manager.handle_can_frame(message)) continue;
        if (motor_can_manager.handle_can_frame(message)) continue;
    }
}

void can_transmit_task(void* parameter) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        
        bms_can_manager.poll_bms_data(); // Poll BMS data every second
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

void queue_listener_task(void* parameter) {
    QueueHandle_t motor_queue = xQueueCreate(10, sizeof(MotorCANManager::MotorData));
    if (motor_queue == NULL) {
        Serial.println("\nFailed to create motor queue");
        vTaskDelete(NULL);
    }
    motor_can_manager.set_data_queue(motor_queue);

    QueueHandle_t bms_queue = xQueueCreate(10, sizeof(BMSCANManager::BMSData));
    if (bms_queue == NULL) {
        Serial.println("\nFailed to create BMS queue");
        vTaskDelete(NULL);
    }

    bms_can_manager.set_data_queue(bms_queue);

    while (true) {
        MotorCANManager::MotorData motor_data;
        if (xQueueReceive(motor_queue, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]Motor Data: Bus Voltage=%.1fV, Bus Current=%.1fA, Phase Current=%.1fA, RPM=%d\n",
                motor_data.electrical_data.bus_voltage_dV / 10.f, motor_data.electrical_data.bus_current_dA / 10.f, motor_data.electrical_data.phase_current_dA / 10.f, motor_data.electrical_data.rpm);
        }

        BMSCANManager::BMSData bms_data;
        if (xQueueReceive(bms_queue, &bms_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]BMS Data: Voltage=%.1fV, Current=%.1fA, SOC=%.1f%%\n",
                bms_data.voltage_data.cumulative_voltage_decivolts / 10.f,
                bms_data.voltage_data.current_deciamps / 10.f,
                bms_data.voltage_data.soc_millipercent / 10.f);
        }
    }
    vTaskDelete(NULL); // Delete the task to free up resources
}

void setup() {


    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_13, GPIO_NUM_12, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    Serial.begin(921600); // Too low baud rates can cause missing CAN messages output on the serial monitor
    while (!Serial);

    ESP_LOGI(TAG, "TWAI driver installed and started");
    led_blinker_init(); // Initialize the LED blinker
    xTaskCreate(can_receive_task, "CANReceiveTask", 4096, NULL, 3, NULL);
    xTaskCreate(can_transmit_task, "CANTransmitTask", 4096, NULL, 2, NULL);
    xTaskCreate(queue_listener_task, "QueueListenerTask", 4096, NULL, 1, NULL); // Create the queue listener task
}

void loop() {
    vTaskDelete(NULL); // Delete the task to free up resources
}

