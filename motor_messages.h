#pragma once

#include <assert.h>
// The MOTOR_MESSAGES minor number will increment for non breaking changes (i.e. 
// only adding fields or context) and will increment the major number if there is 
// a struct reorganization
#define MOTOR_MESSAGES_VERSION  "7.1"

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

            uint8_t motor_overcurrent:1;
            uint8_t motor_phase_open:1;
            uint8_t motor_encoder:1;
            uint8_t motor_encoder_limit:1;
            uint8_t output_encoder:1;
            uint8_t output_encoder_limit:1;
            uint8_t torque_sensor:1;
            uint8_t controller_tracking:1;

            uint8_t host_fault:1;               // The host requested the system to fault
            uint8_t driver_not_enabled:1;
            uint8_t encoder_disagreement:1;         // motor encoder vs output encoder disagreement
            uint8_t torque_sensor_disagreement:1;   // torque sensor vs current disagreement
            uint8_t init_failure:1;
            uint8_t reserved1:3;

            // warning flags only
            uint8_t reserved2:1;
            uint8_t motor_encoder_warning:1;
            uint8_t output_encoder_warning:1;
            uint8_t torque_sensor_warning:1;
            uint8_t motor_current_limit:1;
            uint8_t motor_voltage_limit:1;
            uint8_t motor_soft_limit:1;        // a warning only. Soft limit mode holds velocity

            // overall fault bit
            uint8_t fault:1;
        };
        uint32_t all;
    };
} MotorError;

#define ERROR_MASK_SEQUENCE                     (1<<0)
#define ERROR_MASK_BUS_VOLTAGE_LOW              (1<<1)
#define ERROR_MASK_BUS_VOLTAGE_HIGH             (1<<2)
#define ERROR_MASK_BUS_CURRENT                  (1<<3)
#define ERROR_MASK_MICROCONTROLLER_TEMPERATURE  (1<<4)
#define ERROR_MASK_BOARD_TEMPERATURE            (1<<5)
#define ERROR_MASK_MOTOR_TEMPERATURE            (1<<6)
#define ERROR_MASK_DRIVER_FAULT                 (1<<7)

#define ERROR_MASK_MOTOR_OVERCURRENT            (1<<8)
#define ERROR_MASK_MOTOR_PHASE_OPEN             (1<<9)
#define ERROR_MASK_MOTOR_ENCODER                (1<<10)
#define ERROR_MASK_MOTOR_ENCODER_LIMIT          (1<<11)
#define ERROR_MASK_OUTPUT_ENCODER               (1<<12)
#define ERROR_MASK_OUTPUT_ENCODER_LIMIT         (1<<13)
#define ERROR_MASK_TORQUE_SENSOR                (1<<14)
#define ERROR_MASK_CONTROLLER_TRACKING          (1<<15)

#define ERROR_MASK_HOST_FAULT                   (1<<16)
#define ERROR_MASK_DRIVER_NOT_ENABLED           (1<<17)
#define ERROR_MASK_ENCODER_DISAGREEMENT         (1<<18)
#define ERROR_MASK_TORQUE_SENSOR_DISAGREEMENT   (1<<19)
#define ERROR_MASK_INIT_FAILURE                 (1<<19)

#define ERROR_MASK_MOTOR_ENCODER_WARNING        (1<<25)
#define ERROR_MASK_OUTPUT_ENCODER_WARNING       (1<<26)
#define ERROR_MASK_TORQUE_SENSOR_WARNING        (1<<27)
#define ERROR_MASK_MOTOR_CURRENT_LIMIT          (1<<28)
#define ERROR_MASK_MOTOR_VOLTAGE_LIMIT          (1<<29)
#define ERROR_MASK_MOTOR_SOFT_LIMIT             (1<<30)
#define ERROR_MASK_FAULT                        (1<<31)

#define ERROR_MASK_ALL                          0x80FFFFFF
#define ERROR_MASK_NONE                         0x80000000

#define ERROR_BIT_STRINGS   {"sequence", "bus_voltage_low", "bus_voltage_high", "bus_current", \
  "microcontroller_temperature", "board_temperature", "motor_temperature", "driver_fault", \
  "motor_overcurrent", "motor_phase_open", "motor_encoder", "motor_encoder_limit", \
  "output_encoder", "output_encoder_limit", "torque_sensor", "controller_tracking", \
  "host_fault", "driver_not_enabled", "encoder_disagreement", "torque_sensor_disagreement", \
  "init_failure", "", "", "", \
  "", "motor_encoder_warning", "output_encoder_warning", "torque_sensor_warning", \
  "motor_current_limit", "motor_voltage_limit", "motor_soft_limit", "fault"}

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
    union {
        float data;                 // type defined by round_robin_type, default float
        uint32_t data_u32;          // uint32 alternative
        int32_t data_i32;           // int32 alternative
    };
} RoundRobinData;

// Default indexes for RoundRobinData
#define MOTOR_TEMPERATURE_INDEX     0
#define BOARD_TEMPERATURE_INDEX     1
#define BUS_VOLTAGE_INDEX           2
#define BUS_CURRENT_INDEX           3
#define MOTOR_POWER_INDEX           4
#define AMBIENT_TEMPERATURE_INDEX   5
#define MOSFET_TEMPERATURE_INDEX    6
#define VOLTAGE_3V3_INDEX           7
#define VOLTAGE_5V_INDEX            8
#define CURRENT_5V_INDEX            9
#define USB_ERROR_COUNT_INDEX       10
#define MOSFET2_TEMPERATURE_INDEX   11
#define OUTPUT_ENCODER_CRC_INDEX    12
#define MOTOR_ENCODER_CRC_INDEX     13
#define TORQUE_SENSOR_CRC_INDEX     14
#define AMBIENT_TEMPERATURE_2_INDEX 15
#define AMBIENT_TEMPERATURE_3_INDEX 16
#define AMBIENT_TEMPERATURE_4_INDEX 17
#define OUTPUT_ENCODER_ERROR_INDEX  18
#define MOTOR_ENCODER_ERROR_INDEX   19
#define TORQUE_SENSOR_ERROR_INDEX   20
#define JOINT_ENCODER_CRC_INDEX     21
#define JOINT_ENCODER_ERROR_INDEX   22
#define MICROCONTROLLER_TEMPERATURE_INDEX 23
#define UPTIME_INDEX                24
#define MOTOR_TEMPERATURE_ESTIMATE_INDEX  25
#define MOTOR_ENCODER_WARNING_INDEX 26
#define OUTPUT_ENCODER_WARNING_INDEX      27
#define JOINT_ENCODER_WARNING_INDEX 28
#define ROUND_ROBIN_LENGTH          30


typedef struct {
    uint32_t mcu_timestamp;             // timestamp in microcontroller clock cycles
    uint32_t host_timestamp_received;   // return of host_timestamp from ReceiveData
    float motor_position;               // motor position in radians
    float joint_position;               // joint position in radians
    float iq;                           // Measured motor current in A line-line
    float torque;                       // measured torque in Nm
    MotorFlags flags;                   // \sa MotorFlags
    RoundRobinData rr_data;             // an index, type, and value that cycles in round robin
    float reserved;
    float iq_desired;                   // desired input to the current controller
    float motor_velocity;               // motor velocity in rad/s
    float joint_velocity;               // joint velocity in rad/s

    int32_t motor_encoder;              // motor position in raw counts
    // 60 bytes
} MotorStatus;

typedef struct {
    uint32_t mcu_timestamp;
    uint32_t host_timestamp_received;
    float motor_position;
    float joint_position;
    float iq;
    float torque;
    MotorFlags flags;
    RoundRobinData rr_data;
    float joint_position2;
    float iq_desired;
    // 48 bytes
} MotorStatusLite;


typedef enum {OPEN, DAMPED, CURRENT, POSITION, TORQUE, IMPEDANCE, VELOCITY, 
    STATE, 
    CURRENT_TUNING, POSITION_TUNING, VOLTAGE, PHASE_LOCK, STEPPER_TUNING, 
    STEPPER_VELOCITY, HARDWARE_BRAKE, JOINT_POSITION, FIND_LIMITS, ADMITTANCE,
    TUNING,
    DRIVER_ENABLE=248, DRIVER_DISABLE=249, 
    CLEAR_FAULTS=250, FAULT=251, NO_MODE=252,
    SLEEP=253, CRASH=254, BOARD_RESET=255} MotorMode;

#define MOTOR_MODE_COLORS {"azure", "orange", "green", "blue", "salmon", "chartreuse", "blue", \
    "magenta", \
    "springgreen", "blue", "violet", "yellow", "cyan", \
    "cyan", "orange", "blue", "blue", "salmon", \
    "white"}
#define MOTOR_MODE_UPPER_COLORS {"azure", "white", \
    "azure", "red", "red", \
    "white", "red", "red"}

typedef enum {SINE, SQUARE, TRIANGLE, CHIRP} TuningMode;

typedef struct {
    float current_desired;              // see above
    float position_desired;
    float velocity_desired;
    float torque_desired;
    float torque_dot_desired;           
    float kp, kd, kt, ks;               // position, velocity, torque, and torque_dot gains
} StateControllerCommand;

// Debug and tuning command options
typedef struct {
    float voltage_desired;              // motor voltage V line-line
} VoltageCommand;                       // Closed loop voltage mode, may overcurrent easily

typedef struct {
    uint32_t mode;                      // \sa TuningMode
    float amplitude;                    // amplitude rad
    float reserved;
    float bias;                         // bias rad
    float frequency;                    // frequency Hz
} PositionTuningCommand;

typedef struct {
    float amplitude;                    // amplitude A
    uint32_t mode;                      // \sa TuningMode
    float reserved;
    float bias;                         // bias A
    float frequency;                    // frequency Hz
} CurrentTuningCommand;

typedef enum {STEPPER_CURRENT, STEPPER_VOLTAGE} StepperMode;

typedef struct {
    uint32_t mode;                      // \sa TuningMode
    float amplitude;                    // amplitude in motor radians
    float kv;                           // motor kv Vs/rad
    float bias;                         // bias rad
    float frequency;                    // frequency Hz
    StepperMode stepper_mode;           // \sa StepperMode
} StepperTuningCommand;                       // open loop mode, may skip in position and overcurrent easily

typedef struct {
    float voltage;                      // motor voltage V line-line
    float velocity;                     // motor velocity rad/s
    float current;                      // motor current in A
    StepperMode stepper_mode;           // \sa StepperMode
} StepperVelocityCommand;                     // open loop mode, may skip in position and overcurrent easily

typedef enum{FIND_LIMITS_OUTPUT, FIND_LIMITS_MOTOR} FindLimitsMode;
typedef enum{FIND_LIMITS_NEGATIVE, FIND_LIMITS_POSITIVE, FIND_LIMITS_BOTH, FIND_LIMITS_BOTH_AUTO} FindLimitsOptions;
typedef struct {
    float current_desired;              // motor current
    float position_desired;             // Position either in motor or output radians
    float velocity_desired;             // Velocity either in motor or output rad/s
    uint8_t mode;                       // \sa FindLimitsMode
    uint8_t options;                    // \sa FindLimitsOptions
} FindLimitsCommand;

typedef struct {
    uint32_t mode;                      // Supports POSITION, VELOCITY, TORQUE
    uint32_t tuning_mode;               // \sa TuningMode
    float amplitude;                    // amplitude generic
    float bias;                         // bias generic
    float frequency;                   // frequency Hz
} TuningCommand;

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
        VoltageCommand voltage;
        PositionTuningCommand position_tuning;
        CurrentTuningCommand current_tuning;
        StepperTuningCommand stepper_tuning;
        StepperVelocityCommand stepper_velocity;
        FindLimitsCommand find_limits;
        TuningCommand tuning_command;
    };
                                            // 13*4 = 52 bytes
} MotorCommand;

typedef struct {
    uint32_t timestamp;
    float measured_motor_position;
    float command_iq;
    float measured_iq;
    float measured_ia;
    float measured_ib;
    float measured_ic;
    float command_va;
    float command_vb;
    float command_vc;
    float vbus;
} FastLog; // A debug struct subject to change

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
