// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h" 
#include <map>

static const char* TAG = "CAN";
#define CAN_ELECTRICAL_DATA_ID  0x180117EF
#define CAN_STATE_DATA_ID 0x180217EF

struct MotorElectricalData {
	uint16_t bus_voltage_dV; // Bus voltage in 0.1V units
	int16_t bus_current_dA; // Bus current in 0.1A units (signed)
	int16_t phase_current_dA; // Phase current in 0.1A units (signed)
	int16_t rpm_centi; // RPM in 0.1 units (signed)
};

struct MotorStateData {
	int8_t controller_temp_C; // Controller temperature in Celsius units (signed)
	int8_t motor_temp_C; // Motor temperature in Celsius units (signed)
	uint8_t accelerator_percent; // Accelerator pedal pos;ition in percent (0-100)
	uint8_t status; // Gear, brake, operation mode, DC contactor
	uint32_t error; // Error codes
};

void handle_electrical_data(const twai_message_t& message) {
	MotorElectricalData data;
	data.bus_voltage_dV = message.data[0] | (message.data[1] << 8);
	data.bus_current_dA = message.data[2] | (message.data[3] << 8) - 3200;
	data.phase_current_dA = message.data[4] | (message.data[5] << 8) - 3200;
	data.rpm_centi = message.data[6] | (message.data[7] << 8) - 32000;
	Serial.printf("Received Electrical Data: Bus Voltage=%dV, Bus Current=%dA, Phase Current=%dA, RPM=%d\n",
		data.bus_voltage_dV, data.bus_current_dA, data.phase_current_dA, data.rpm_centi);
}

void handle_state_data(const twai_message_t& message) {
	MotorStateData data;
	data.controller_temp_C = message.data[0] - 40;
	data.motor_temp_C = message.data[1] - 40;
	data.accelerator_percent = message.data[2];
	data.status = message.data[3];
	data.error = message.data[4] | (message.data[5] << 8) | (message.data[6] << 16) || (message.data[7] << 24);
	Serial.printf("Received State Data: Controller Temp=%dC, Motor Temp=%dC, Accelerator=%d%%, Status=%d, Error=0x%X\n",
		data.controller_temp_C, data.motor_temp_C, data.accelerator_percent, data.status, data.error);
}

void handle_can_frame(const twai_message_t& message) {
	if (message.identifier == CAN_ELECTRICAL_DATA_ID) {
		handle_electrical_data(message);
	} else if (message.identifier == CAN_STATE_DATA_ID) {
		handle_state_data(message);
	} else {
		Serial.printf("Unknown CAN ID: 0x%X\n", message.identifier);
	}
}

void can_receive_task(void* parameter) {
	
	while (true) {
		twai_message_t message;
		if (twai_receive(&message, pdMS_TO_TICKS(portMAX_DELAY)) == ESP_OK) {
			handle_can_frame(message);
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

