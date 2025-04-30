#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include <map>
#include <functional>
#include "ICANManager.h"

class MotorCANManager : public ICANManager {
public:
    struct MotorElectricalData {
        uint16_t bus_voltage_dV; // Bus voltage in 0.1V units
        int16_t bus_current_dA; // Bus current in 0.1A units (signed)
        int16_t phase_current_dA; // Phase current in 0.1A units (signed)
        int16_t rpm; // RPM per bit (signed, offset 32000)
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

    MotorCANManager();
    void handle_can_frame(const twai_message_t& message) override;

private:
    MotorData motor_data_left;
    MotorData motor_data_right;
    using CANHandler = std::function<void(const twai_message_t&)>;
    std::map<uint32_t, CANHandler> can_handlers;
    void initialize_can_handlers();
    void handle_electrical_data(const twai_message_t& message, MotorElectricalData& data);
    void handle_state_data(const twai_message_t& message, MotorStateData& data);
    void decode_motor_status(uint8_t status);
    void decode_motor_error(uint32_t error);
};
