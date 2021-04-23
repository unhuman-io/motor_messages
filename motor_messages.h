#pragma once

#define MOTOR_MESSAGES_VERSION  "1.0"

typedef struct {
    uint32_t mcu_timestamp;             // timestamp in microcontroller clock cycles
    uint32_t host_timestamp_received;   // return of host_timestamp from ReceiveData
    float motor_position;               // motor position in radians
    float joint_position;               // joint position in radians
    float iq;                           // Measured motor current in A line-line
    float torque;                       // measured torque in Nm
    int32_t motor_encoder;              // motor position in raw counts
    float reserved[3];
} MotorStatus;

typedef enum {OPEN, DAMPED, CURRENT, POSITION, TORQUE, IMPEDANCE, VELOCITY, 
    CURRENT_TUNING, POSITION_TUNING, VOLTAGE, PHASE_LOCK, STEPPER_TUNING, 
    STEPPER_VELOCITY, 
    CRASH=254, BOARD_RESET=255} MotorMode;

typedef enum {SINE, SQUARE, TRIANGLE, CHIRP} TuningMode;

typedef struct {
    uint32_t host_timestamp;            // Value from host
    uint8_t mode_desired;               // \sa ModeDesired
    union {
        // main control modes
        struct {                                
            float current_desired;              // motor current desired in A line-line
            float position_desired;             // motor position desired in rad
            float velocity_desired;             // motor velocity desired in rad/s
            float torque_desired;               // torque desired Nm
            float reserved;                     // reserved for strange uses
        };
        
        // debug/tuning modes
        struct {
            float voltage_desired;
            float reserved[4];
        } voltage;
        struct {
            uint32_t mode;
            float amplitude;
            float reserved;
            float bias;
            float frequency;
        } position_tuning;
        struct {
            float amplitude;
            uint32_t mode;
            float reserved;
            float bias;
            float frequency;
        } current_tuning;
        struct {
            uint32_t mode;
            float amplitude;
            float kv;
            float bias;
            float frequency;
        } stepper_tuning;
        struct {
            float voltage;
            float velocity;
            float reserved[3];
        } stepper_velocity;
    };
} MotorCommand;
