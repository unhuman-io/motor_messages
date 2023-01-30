#pragma once

#include <assert.h>
// The MOTOR_MESSAGES minor number will increment for non breaking changes (i.e. 
// only adding fields or context) and will increment the major number if there is 
// a struct reorganization
#define MOTOR_MESSAGES_VERSION  "3.3"

#ifdef __cplusplus
namespace obot {
#endif

// The structs below are used for direct communication with the STM32 microcontroller
// The STM32 is 32 bit little-endian so the packing of these structs will follow.
// 32 bit alignment by default will cause 32 bit fields to be aligned leaving 
// empty space after 8 or 16 bit fields. Fortunately this packing seems to work 
// natively without additional translahtion on current intel, arm, and esp32 processors.

// Each of the below errors can be individually mapped to either force the mode into
// safe_mode or be ignored and only reported. MotorErrors are latching and can only
// be cleared by a reset. More detailed error information may be available through
// the text api.
typedef struct {
    union {
        struct {
            uint8_t sequence:1;                 // if sequence error detection is on and
                                                // there is a discrepancy in host_timestamp_received
            uint8_t bus_voltage_low:1;
            uint8_t bus_voltage_high:1;
            uint8_t bus_current:1;
            uint8_t microcontroller_temperature:1;
            uint8_t board_temperature:1;
            uint8_t motor_temperature:1;
            uint8_t driver_fault:1;

            uint8_t motor_overcurent:1;
            uint8_t motor_phase_open:1;
            uint8_t motor_encoder:1;
            uint8_t motor_encoder_limit:1;
            uint8_t output_encoder:1;
            uint8_t output_encoder_limit:1;
            uint8_t torque_sensor:1;
            uint8_t controller_tracking:1;

            uint8_t host_fault:1;               // The host requested the system to fault
            uint8_t driver_not_enabled:1;
            uint8_t reserved1:6;

            uint8_t reserved2:7;
            uint8_t fault:1;
        };
        uint32_t all;
    };
} MotorError;

#define ERROR_MASK_ALL  0xFFFFFFFF
#define ERROR_MASK_NONE 0x80000000

typedef struct {
    uint8_t mode;                       // returns current mode, should be mode_desired
                                        // unless there is an error
    union {
        uint8_t byte;
        struct {
            uint8_t gpio:1;             // a gpio input
        };
    } misc;
    MotorError error;                   // \sa MotorError
} MotorFlags;

static_assert(sizeof(MotorFlags) == sizeof(uint32_t)*2);

typedef enum {FLOAT=0, UINT32_T=1, INT32_T=2} RoundRobinType;

typedef struct {
    uint8_t index;                  // Index to custom data field
    uint8_t type;                   // \sa RoundRobinType
    float data;                     // type defined by round_robin_type, default float
} RoundRobinData;

typedef struct {
    uint32_t mcu_timestamp;             // timestamp in microcontroller clock cycles
    uint32_t host_timestamp_received;   // return of host_timestamp from ReceiveData
    float motor_position;               // motor position in radians
    float joint_position;               // joint position in radians
    float iq;                           // Measured motor current in A line-line
    float torque;                       // measured torque in Nm
    int32_t motor_encoder;              // motor position in raw counts
    RoundRobinData rr_data;             // an index, type, and value that cycles in round robin
    float reserved;
    MotorFlags flags;                   // \sa MotorFlags
                                        // 48 bytes
} MotorStatus;

typedef enum {OPEN, DAMPED, CURRENT, POSITION, TORQUE, IMPEDANCE, VELOCITY, 
    STATE, 
    CURRENT_TUNING, POSITION_TUNING, VOLTAGE, PHASE_LOCK, STEPPER_TUNING, 
    STEPPER_VELOCITY, HARDWARE_BRAKE,
    DRIVER_ENABLE=248, DRIVER_DISABLE=249, 
    CLEAR_FAULTS=250, FAULT=251, NO_MODE=252,
    SLEEP=253, CRASH=254, BOARD_RESET=255} MotorMode;

typedef enum {SINE, SQUARE, TRIANGLE, CHIRP} TuningMode;

typedef struct {
    float current_desired;              // see above
    float position_desired;
    float velocity_desired;
    float torque_desired;
    float torque_dot_desired;           
    float kp, kd, kt, ks;               // position, velocity, torque, and torque_dot gains
} StateControllerCommand;

typedef struct {
    uint32_t host_timestamp;            // Value from host
    uint8_t mode_desired;               // \sa ModeDesired
    union {
        uint8_t byte;
        struct {
            uint8_t gpio:1;              // a gpio signal
        };
    } misc;
    union {
        // main control modes
        struct {                                
            float current_desired;              // motor current desired in A line-line
            float position_desired;             // motor position desired in rad
            float velocity_desired;             // motor velocity desired in rad/s
            float torque_desired;               // torque desired Nm
            float torque_dot_desired;           // torque_dot desired Nm/s
            float reserved;                     // reserved for strange uses
            float reserved2[3];                 // also reserved
        };

        // state controller
        StateControllerCommand state;
        
        // debug/tuning modes
        struct {
            float voltage_desired;              // motor voltage V line-line
        } voltage;                              // Close loop voltage mode, may overcurrent easily
        struct {
            uint32_t mode;                      // \sa TuningMode
            float amplitude;                    // amplitude rad
            float reserved;
            float bias;                         // bias rad
            float frequency;                    // frequency Hz
        } position_tuning;
        struct {
            float amplitude;                    // amplitude A
            uint32_t mode;                      // \sa TuningMode
            float reserved;
            float bias;                         // bias A
            float frequency;                    // frequency Hz
        } current_tuning;
        struct {
            uint32_t mode;                      // \sa TuningMode
            float amplitude;                    // amplitude in motor radians
            float kv;                           // motor kv Vs/rad
            float bias;                         // bias rad
            float frequency;                    // frequency Hz
        } stepper_tuning;                       // open loop mode, may skip in position and overcurrent easily
        struct {
            float voltage;                      // motor voltage V line-line
            float velocity;                     // motor velocity rad/s
        } stepper_velocity;                     // open loop mode, may skip in position and overcurrent easily
    };
                                            // 11*4 = 44 bytes
} MotorCommand;

#ifdef __cplusplus
}  // namespace obot
#endif

// USB is the current default physical communication protocol. MotorCommand and 
// MotorStatus structs are sent on one endpoint as defined below. A separate debugging
// and other communication channel called the text_api is available on on another
// endpoint. The text_api communicates in 8 bit char non-null terminated strings.
// The length of the string is provided by the USB protocol and has a max length 
// defined below 

#define USB_ENDPOINT_MOTOR      2
#define USB_ENDPOINT_TEXT_API   1
#define MAX_API_DATA_SIZE 1000      // refers to the TextAPI which is a separate
                                    // communication channel, both TX and RX
