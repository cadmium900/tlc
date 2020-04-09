///
/// \file       datamodel.h
/// \brief      The Lung Carburetor Firmware datamodel definitions file
///
/// \author     Frederic Lauzon
/// \defgroup   datamodel Data Model
#ifndef TLC_DATAMODEL_H
#define TLC_DATAMODEL_H

#include "common.h"

/// \struct tPressureCurve
/// \brief Describe a pressurecurve to execute
///
/// A collection of setpoints of pressure in time to execute for the respiration cycle
struct tPressureCurve 
{
    float           fSetPoint_mmH2O[kMaxCurveCount];    ///> Pressure for every point of the curve
    uint32_t        nSetPoint_TickMs[kMaxCurveCount];   ///> Number of millisecond to execute point of the curve
    uint8_t         nCount;                             ///> Number of active points in the setpoint curve
} __attribute__((packed));

/// \struct tDataModel
/// \brief Describe internal data
///
/// A centralised collection of information read and written by modules
/// The data model structure is used directly as a register file by communications
/// Changing this structure directly affects communications protocol serialization
struct tDataModel 
{    
    uint16_t        nSafetyFlags;           ///> Bitmask of safety alarm source eAlarm    
    uint8_t         nRqState;               ///> Requested state from eRqState
    eState          nState;                 ///> System state
    eControlMode    nControlMode;           ///> Control mode of the pump
    eTriggerMode    nTriggerMode;           ///> Respiration trigger mode
    eCycleState     nCycleState;            ///> Respiration cycle state
    uint8_t         nRawPressure[2];        ///> Raw read pressure from sensor
    float           fBatteryLevel;          ///> Battery voltage level
    
    tPressureCurve  pInhaleCurve;           ///> Inhale curve descriptor
    tPressureCurve  pExhaleCurve;           ///> Exhale curve descriptor
    uint8_t         nCurveIndex;            ///> Current executing curve setpoint index

    float           fRequestPressure_mmH2O; ///> Requested pressure set-point
    uint8_t         nRespirationPerMinute;  ///> Number of respiration per minute
    float           fInhalePressureTarget_mmH2O; ///> Inhale Pressure Target
    float           fExhalePressureTarget_mmH2O; ///> Exhale Pressure Target
    float           fInhaleRatio;           ///> Inhale Ratio
    float           fExhaleRatio;           ///> Exhale Ratio

    float           fPressure_mmH2O[2];     ///> Converted pressure, useable as cmH2O
    float           fPressureError;         ///> Pressure error: readings vs set-point
    float           fP;                     ///> Control Proportional
    float           fI;                     ///> Control Integral
    float           fD;                     ///> Control Derivative
    float           fPI;                    ///> Control Sum of Proportional and Integral errors
    uint16_t        nPWMPump;               ///> Pump PWM power output

    uint32_t        nTickControl;           ///> Last control tick
    uint32_t        nTickCommunications;    ///> Last communications tick
    uint32_t        nTickSensors;           ///> Last sensors tick
    uint32_t        nTickSetPoint;          ///> Current curve pressure set-point ticker
    uint32_t        nTickRespiration;       ///> Start of respiration tick
    uint32_t        nTickStabilization;     ///> Stabilization tick between respiration
    uint32_t        nTickWait;              ///> Wait tick after respiration
    uint32_t        nTickLcdKeypad;         ///> Lcd and Keypad update and scan rate

    uint8_t         nTerminator;            ///> Terminator check MUST BE LAST element of structure
} __attribute__((packed));
HXCOMPILATIONASSERT(assertDataModelSizeCheck,        (sizeof(tDataModel) == offsetof(tDataModel, nTerminator)+1));

// It is assumed that enums are 2 bytes in size
HXCOMPILATIONASSERT(assertEnumStateSizeCheck,        (sizeof(eState) == 2));
HXCOMPILATIONASSERT(assertEnumControlModeSizeCheck,  (sizeof(eControlMode) == 2));
HXCOMPILATIONASSERT(assertEnumTriggerModeSizeCheck,  (sizeof(eTriggerMode) == 2));
HXCOMPILATIONASSERT(assertEnumCycleStateSizeCheck,   (sizeof(eCycleState) == 2));

// The protocol assumes that there is maximum 8 points in a curve
HXCOMPILATIONASSERT(assertMaxCurveCountCheck,        (kMaxCurveCount == 8));
HXCOMPILATIONASSERT(assertCheckOffsetTerminator,     (offsetof(tDataModel, nTerminator) == 231));


#define PROTOCOL_KEY    (uint16_t)offsetof(tDataModel, nSafetyFlags) + \
                        (uint16_t)offsetof(tDataModel, nRqState) + \
						(uint16_t)offsetof(tDataModel, nState) + \
						(uint16_t)offsetof(tDataModel, nControlMode) + \
						(uint16_t)offsetof(tDataModel, nTriggerMode) + \
						(uint16_t)offsetof(tDataModel, nCycleState) + \
						(uint16_t)offsetof(tDataModel, nRawPressure[0]) + \
						(uint16_t)offsetof(tDataModel, nRawPressure[1]) + \
						(uint16_t)offsetof(tDataModel, fBatteryLevel) + \
						(uint16_t)offsetof(tDataModel, pInhaleCurve) + \
						(uint16_t)offsetof(tDataModel, pExhaleCurve) + \
						(uint16_t)offsetof(tDataModel, nCurveIndex) + \
						(uint16_t)offsetof(tDataModel, fRequestPressure_mmH2O) + \
						(uint16_t)offsetof(tDataModel, nRespirationPerMinute) + \
						(uint16_t)offsetof(tDataModel, fInhalePressureTarget_mmH2O) + \
						(uint16_t)offsetof(tDataModel, fExhalePressureTarget_mmH2O) + \
						(uint16_t)offsetof(tDataModel, fInhaleRatio) + \
						(uint16_t)offsetof(tDataModel, fExhaleRatio) + \
						(uint16_t)offsetof(tDataModel, fPressure_mmH2O[0]) + \
						(uint16_t)offsetof(tDataModel, fPressure_mmH2O[1]) + \
						(uint16_t)offsetof(tDataModel, fPressureError) + \
						(uint16_t)offsetof(tDataModel, fP) + \
						(uint16_t)offsetof(tDataModel, fI) + \
						(uint16_t)offsetof(tDataModel, fD) + \
						(uint16_t)offsetof(tDataModel, fPI) + \
						(uint16_t)offsetof(tDataModel, nPWMPump) + \
						(uint16_t)offsetof(tDataModel, nTickControl) + \
						(uint16_t)offsetof(tDataModel, nTickCommunications) + \
						(uint16_t)offsetof(tDataModel, nTickSensors) + \
						(uint16_t)offsetof(tDataModel, nTickSetPoint) + \
						(uint16_t)offsetof(tDataModel, nTickRespiration) + \
						(uint16_t)offsetof(tDataModel, nTickStabilization) + \
						(uint16_t)offsetof(tDataModel, nTickWait) + \
						(uint16_t)offsetof(tDataModel, nTickLcdKeypad) + \
						(uint16_t)offsetof(tDataModel, nTerminator)

HXCOMPILATIONASSERT(assertCheckProtocolKey, (PROTOCOL_KEY == 4643));

// Uncomment to trace the value of protocol_key at compile-time:    HXCOMPILATIONTRACE(stopCompileCheckSize, PROTOCOL_KEY);
// Uncomment to trace the size of tDataModel at compile-time:       HXCOMPILATIONTRACE(stopCompileCheckSize, sizeof(tDataModel));

extern tDataModel gDataModel;

/// \fn bool DataModel_Init()
/// \brief Initialize datamodel defaults
bool DataModel_Init();

#endif // TLC_DATAMODEL_H
