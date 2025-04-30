// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h" 
#include <map>

static const char* TAG = "CAN";
#define CAN_ELECTRICAL_DATA_ID_LEFT  0x180117F0
#define CAN_STATE_DATA_ID_LEFT 0x180217F0
#define CAN_ELECTRICAL_DATA_ID_RIGHT  0x180117EF
#define CAN_STATE_DATA_ID_RIGHT 0x180217EF

class MotorCANManager {
public:
	struct MotorElectricalData {
		uint16_t bus_voltage_dV; // Bus voltage in 0.1V units
		int16_t bus_current_dA; // Bus current in 0.1A units (signed)
		int16_t phase_current_dA; // Phase current in 0.1A units (signed)
		int16_t rpm; // RPM in 0.1 units (signed)
	};

	struct MotorStateData {
		int8_t controller_temp_C;      // Controller temperature in Celsius units (signed)
		int8_t motor_temp_C;           // Motor temperature in Celsius units (signed)
		uint8_t accelerator_percent;   // Accelerator pedal position in percent (0-100)

		/**
		 * Status byte bitfields:
		 * Bits 0-2: Gear
		 *   0: NO
		 *   1: R (Reverse)
		 *   2: N (Neutral)
		 *   3: D1 (Drive 1)
		 *   4: D2 (Drive 2)
		 *   5: D3 (Drive 3)
		 *   6: S (Sport)
		 *   7: P (Park)
		 * Bit 3: Brake
		 *   0: No brake
		 *   1: Brake
		 * Bits 4-6: Operation Mode
		 *   0: Stopped
		 *   1: Drive
		 *   2: Cruise
		 *   3: EBS
		 *   4: Hold
		 * Bit 7: DC Contactor
		 *   0: OFF
		 *   1: ON
		 */
		uint8_t status;                // Status flags (gear, brake, operation mode, DC contactor)

		/**
		 * Error code bitfields (bit index = error):
		 *  0: Overcurrent
		 *  1: Overload
		 *  2: Overvoltage
		 *  3: Undervoltage
		 *  4: Controller Overheat
		 *  5: Motor Overheat
		 *  6: Motor Stalled
		 *  7: Motor Out of phase
		 *  8: Motor Sensor
		 *  9: Motor AUX Sensor
		 * 10: Encoder Misaligned
		 * 11: Anti-Runaway Engaged
		 * 12: Main Accelerator
		 * 13: AUX Accelerator
		 * 14: Pre-charge
		 * 15: DC Contactor
		 * 16: Power valve
		 * 17: Current Sensor
		 * 18: Auto-tune
		 * 19: RS485
		 * 20: CAN
		 * 21: Software
		 */
		uint32_t error;                // Error codes (bitmask, see above)
	};

	struct MotorData {
		MotorElectricalData electrical_data;
		MotorStateData state_data;
	};

	MotorCANManager() {
		initialize_can_handlers();
	}

	void handle_can_frame(const twai_message_t& message) {
		if (can_handlers.find(message.identifier) != can_handlers.end()) {
			can_handlers[message.identifier](message);
		} else {
			Serial.printf("Unknown CAN ID: 0x%X\n", message.identifier);
		}
	}

	void handle_electrical_data(const twai_message_t& message, MotorElectricalData& data) {
		data.bus_voltage_dV = (message.data[0] | (message.data[1] << 8));
		data.bus_current_dA = (message.data[2] | (message.data[3] << 8)) - 32000;
		data.phase_current_dA = (message.data[4] | (message.data[5] << 8)) - 32000;
		data.rpm = (message.data[6] | (message.data[7] << 8)) - 32000;
		Serial.printf("Received Electrical Data: Bus Voltage=%.1fV, Bus Current=%.1fA, Phase Current=%.1fA, RPM=%d\n",
			data.bus_voltage_dV / 10.f, data.bus_current_dA / 10.f, data.phase_current_dA / 10.f, data.rpm);
	}
	
	void handle_state_data(const twai_message_t& message, MotorStateData& data) {
		data.controller_temp_C = message.data[0] - 40;
		data.motor_temp_C = message.data[1] - 40;
		data.accelerator_percent = message.data[2];
		data.status = message.data[3];
		data.error = message.data[4] | (message.data[5] << 8) | (message.data[6] << 16) || (message.data[7] << 24);
		Serial.printf("Received State Data: Controller Temp=%dC, Motor Temp=%dC, Accelerator=%d%%, Status=0x%02X, Error=0x%02X\n",
			data.controller_temp_C, data.motor_temp_C, data.accelerator_percent, data.status, data.error);
		decode_motor_status(data.status);
		decode_motor_error(data.error);
	}

private:
	MotorData motor_data_left;
	MotorData motor_data_right;


	using CANHandler = std::function<void(const twai_message_t&)>;
	std::map<uint32_t, CANHandler> can_handlers;

	// Initialize the CAN lookup table
	void initialize_can_handlers() {
		can_handlers[CAN_ELECTRICAL_DATA_ID_LEFT] = [this](const twai_message_t& message) {
			handle_electrical_data(message, motor_data_left.electrical_data);
		};
		can_handlers[CAN_STATE_DATA_ID_LEFT] = [this](const twai_message_t& message) {
			handle_state_data(message, motor_data_left.state_data);
		};
		can_handlers[CAN_ELECTRICAL_DATA_ID_RIGHT] = [this](const twai_message_t& message) {
			handle_electrical_data(message, motor_data_right.electrical_data);
		};
		can_handlers[CAN_STATE_DATA_ID_RIGHT] = [this](const twai_message_t& message) {
			handle_state_data(message, motor_data_right.state_data);
		};
	}

	void decode_motor_status(uint8_t status) {
		uint8_t gear = status & (BIT2 | BIT1 | BIT0); // Mask for gear bits
		const char* gear_str = "";
		switch (gear) {
			case 0b000: gear_str = "NO"; break;
			case 0b001: gear_str = "R"; break;
			case 0b010: gear_str = "N"; break;
			case 0b011: gear_str = "D1"; break;
			case 0b100: gear_str = "D2"; break;
			case 0b101: gear_str = "D3"; break;
			case 0b110: gear_str = "S"; break;
			case 0b111: gear_str = "P"; break;
			default: gear_str = "Unknown gear"; break;
		}
	
		bool brake = (status & BIT3) >> 3; // Brake bit
		const char* brake_str = brake ? "ON" : "OFF";
	
		uint8_t operation_mode = (status & (BIT6 | BIT5 | BIT4)) >> 4;
		const char* operation_mode_str = "";
		switch (operation_mode) {
			case 0b000: operation_mode_str = "Stopped"; break;
			case 0b001: operation_mode_str = "Drive"; break;
			case 0b010: operation_mode_str = "Cruise"; break;
			case 0b011: operation_mode_str = "EBS"; break;
			case 0b100: operation_mode_str = "Hold"; break;
		}
	
		uint8_t dc_contactor = (status & BIT7) >> 7; // DC contactor bit
		const char* dc_contactor_str = dc_contactor ? "ON" : "OFF";
	
		printf("Gear: %s, Brake: %s, Operation Mode: %s, DC Contactor: %s\n",
			gear_str, brake_str, operation_mode_str, dc_contactor_str);
	}
	
	void decode_motor_error(uint32_t error) {
		std::map<uint32_t, const char*> error_map = {
			{BIT0, "Motor Overcurrent"},
			{BIT1, "Motor Overload"},
			{BIT2, "Motor Overvoltage"},
			{BIT3, "Motor Undervoltage"},
			{BIT4, "Controller Overheat"},
			{BIT5, "Motor Overheat"},
			{BIT6, "Motor Stalled"},
			{BIT7, "Motor Out of Phase"},
			{BIT8, "Motor Sensor Failure"},
			{BIT9, "Motor AUX Sensor"},
			{BIT10, "Encoder Misaligned"},
			{BIT11, "Anti-Runaway Engaged"},
			{BIT12, "Main Accelerator"},
			{BIT13, "AUX Accelerator"},
			{BIT14, "Pre-charge"},
			{BIT15, "DC Contactor"},
			{BIT16, "Power Valve"},
			{BIT17, "Current Sensor"},
			{BIT18, "Auto-tune"},
			{BIT19, "RS485"},
			{BIT20, "CAN"},
			{BIT21, "Software"}
		};
	
		for (const auto& pair : error_map) {
			if (error & pair.first) {
				Serial.printf("Error: %s\n", pair.second);
			}
		}
	}
};

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

