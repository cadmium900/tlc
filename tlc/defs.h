///
/// \file       defs.h
/// \brief      The Lung Carburetor Firmware definitions file
///
/// \author     Frederic Lauzon
#ifndef TLC_DEFS_H
#define TLC_DEFS_H

#include <stdint.h>

// Force error check at compile time for constants
#define HXCOMPILATIONASSERT(name, x) typedef char name[x ? 1 : -1]
// Force compilation error and trace the value in the error message
#define HXCOMPILATIONTRACE(name, x) char (*name)[x] = 1


/// \enum eConsts
/// \brief General constants used throughout the firmware
enum eConsts
{
    kSerialBaudRate             = 500000,   ///> Baud rate of serial port
    kSerialRxTimeOut            = 10,       ///> Maximum wait time in ms to wait for serial data
    kCommBufferSize             = 128,      ///> Maximum rx buffer size
    kRxBufferReserve            = 10,       ///> Reserve of data before we start discarding rx buffer
    kSerialDiscardTimeout       = 500,      ///> Discard rx buffer timeout
    kSerialCommandTimeout       = 2000,     ///> Restart comms if no commands received timeout
    kPeriodControl              = 5,        ///> Period to call control loop in milliseconds
    kPeriodCommunications       = 2,        ///> Period to call communications loop in milliseconds
    kPeriodLcdKeypad            = 250,      ///> Period to refresh Lcd and scan keypad in milliseconds
    kPeriodSensors              = 5,        ///> Period to call sensors loop in milliseconds
    kPeriodWarmup               = 1000,     ///> Period to warmup the system in milliseconds
    kPeriodStabilization        = 100,      ///> Stablization period between respiration cycles
    kEEPROM_Version             = 1,        ///> EEPROM version must match this version for compatibility
    kMaxCurveCount              = 8,        ///> Maximum respiration curve index count
    kPacketId                   = 0xFEED    ///> Packet Identifier for communications
};

HXCOMPILATIONASSERT(assertSensorPeriodCheck, (kPeriodSensors >= 1));
HXCOMPILATIONASSERT(assertRXBufferSizeCheck, (kCommBufferSize < 255));


/// \struct tPacketHeader
/// \brief Describe the communications packet header
///
/// Data prefix sent before payload over serial port from and to the firmware
struct tPacketHeader
{
    uint16_t id;
    uint8_t  size;
    uint16_t crc;
    uint16_t keyDataModel;
    uint16_t keyCfg;
    uint8_t  cmd;
} __attribute__((packed));

/// \struct tPacketReadData
/// \brief Describe the read data payload to firmware
///
/// Firmware will reply with tPacketReadData and an array of bytes that match the request amount of bytes
struct tPacketReadData
{
    uint8_t offset;
    uint8_t bytesToRead;
} __attribute__((packed));

/// \struct tPacketWriteData
/// \brief Describe the write data command to firmware
struct tPacketWriteData
{
    uint8_t offset;
    uint8_t bytesToWrite;
} __attribute__((packed));

/// \struct tPacketCurve
/// \brief Describe the curve parameters to set for respiration
struct tPacketCurve
{
    float breatheRate;
    float inhaleMmH2O;
    float exhaleMmH2O;
    float inhaleRatio;
    float exhaleRatio;
} __attribute__((packed));

/// \enum ePacketCommand
/// \brief Packet Command Id for serial communications
enum ePacketCommand
{
    kPacketCommand_ReadData             = 0,
    kPacketCommand_WriteData            = 1,
    kPacketCommand_ReadCfg              = 2,
    kPacketCommand_WriteCfg             = 3,
    kPacketCommand_WriteCfgToEeprom     = 4,
    kPacketCommand_LoadCfgFromEeprom    = 5,
    kPacketCommand_SetDefaultCfg        = 6,
    kPacketCommand_SetZeroPressure      = 7,
    kPacketCommand_ClearSafeties        = 8,
    kPacketCommand_DisableSafeties      = 9,
    kPacketCommand_EnableSafeties       = 10,
    kPacketCommand_SetCurve             = 11,

    // Firmware reply to received command with ack or nack set as bit 7 or 6
    kPacketCommand_MaskAck              = (1<<7),
    kPacketCommand_MaskNAck             = (1<<6)
};


/// \enum eState
/// \brief System state
enum eState
{
    kState_Init = 0,    ///> System initializing
    kState_Idle,        ///> System is idle
    kState_Warmup,      ///> System is warming up and sampling sensors
    kState_Process,     ///> System is processing respiration and sensors
    kState_Error,       ///> System raised an error

    kState_Count
};

enum eRqState
{
    kRqState_Stop = 0,
    kRqState_Start
};

/// \enum eState
/// \brief Respiration Cycle State
enum eCycleState
{
    kCycleState_WaitTrigger = 0,    ///> Respiration cycle waiting for trigger
    kCycleState_Inhale,             ///> Respiration cycle inhaling
    kCycleState_Exhale,             ///> Respiration cycle exnhaling
    kCycleState_Stabilization,      ///> Respiration cycle stabilization of pressure

    kCycleState_Count
};

/// \enum eControlMode
/// \brief Pump pressure control mode
enum eControlMode
{
    kControlMode_PID = 0,       ///> Pump is controlled by pressure feedback using a PID
    kControlMode_FeedForward,   ///> Feedforward is used to send requested pump pwm values from master controller

    kControlMode_Count
};

/// \enum eTriggerMode
/// \brief Respiration trigger mode
enum eTriggerMode
{
    kTriggerMode_Timed = 0,             ///> Machine Respiration timed, ignore patient respiration
    kTriggerMode_Patient,               ///> Machine Respiration triggered by patient respiration
    kTriggerMode_PatientSemiAutomatic,  ///> Machine Respiration triggered by patient respiration, or timed when patient is not triggering after a timeout

    kTriggerMode_Count
};

/// \enum eAlarm
/// \brief Alarms raised by safeties
enum eAlarm
{
    kAlarm_MaxPressureLimit             = (1<<0),   ///> System max pressure reached
    kAlarm_MinPressureLimit             = (1<<1),   ///> System min pressure reached
    kAlarm_PressureSensorRedudancyFail  = (1<<2),   ///> Both pressure sensors report different readings
    kAlarm_InvalidConfiguration         = (1<<3),   ///> Loaded configuration is invalid
    kAlarm_BatteryLow                   = (1<<4),   ///> Low battery voltage
};

const float kMPX5010_MaxPressure_mmH2O          = 1019.78f;
const float kMPX5010_MaxPressureDelta_mmH2O     = 90.0f;
const float kMPX5010_Accuracy                   = 0.5f;
const float kMPX5010_Sensitivity_mV_mmH2O       = 4.413f;
const float kBatteryLevelGain                   = 5.0f;

#define PIN_SERIAL_RX           0       // Serial port RX
#define PIN_SERIAL_TX           1       // Serial port TX

#define PIN_OUT_SERVO_EXHALE    2       // Servo exhale valve

// Timer0 used by millis
// timer1 used by TimerOne
// Timer2 used by ServoTimer2
// pins 3 and 11 analogWrite are disabled by the use of servotimer2 library

// this pin cannot be changes since we use the timer1 port
#define PIN_OUT_PUMP1_PWM       9       // Ambu pump Cam PWM output

#define PIN_OUT_BUZZER          5      // Buzzer signal output

#define PIN_PRESSURE0           A0      // Pressure readings from MPX pressure sensor
#define PIN_PRESSURE1           A1      // Pressure readings from MPX redundant pressure sensor
#define PIN_BATTERY             A2      // Battery voltage

#define PIN_LCD_KEYPAD_SDA      A4
#define PIN_LCD_KEYPAD_SCL      A5

#endif // TLC_DEFS_H
