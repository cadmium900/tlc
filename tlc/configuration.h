///
/// \file       configuration.h
/// \brief      The Lung Carburetor Firmware configuration module
///
/// \author     Frederic Lauzon
/// \defgroup   configuration Configuration
#ifndef TLC_CONFIGURATION_H
#define TLC_CONFIGURATION_H

#include "common.h"

/// \struct tConfiguration
/// \brief NVM Configuration Stored and Loaded from EEPROM.
struct tConfiguration
{
    uint8_t     nVersion;                   ///> Configuration structure version
    uint16_t    nPressureSensorOffset[3];   ///> Offset when pressure sensor is at atmosphere readings
    float       fMinBatteryLevel;           ///> Minimum battery level for alarm
    float       fMaxPressureLimit_mmH2O;    ///> Max allowed pressure limit
    float       fMinPressureLimit_mmH2O;    ///> Min allowed pressure limit
    float       fMaxPressureDelta_mmH2O;    ///> Maximum allowed pressure delta between redundant readings
    float       fMinPressureLimit_Flow;     ///> Minimum pressure on pressure (flow) sensor to detect a problem
    float       fGainP;                     ///> Control gain P
    float       fGainI;                     ///> Control gain I
    float       fGainD;                     ///> Control gain D
    float       fILimit;                    ///> Integral error limit
    float       fPILimit;                   ///> Proportional+Integral error limit
    float       fControlTransfer;           ///> Control transfer from adjusted errors to pwm
    float       fPatientTrigger_mmH2O;      ///> Patient triggers respiration when this value is reached (In TriggerMode Patient or semi automatic)
    uint16_t    nServoExhaleOpenAngle;      ///> Angle in degree (0..180) when exhale servo valve is open
    uint16_t    nServoExhaleCloseAngle;     ///> Angle in degree (0..180) when exhale servo valve is close
    uint32_t    nCRC;                       ///> Configuration CRC check
} __attribute__((packed));
extern tConfiguration gConfiguration;

HXCOMPILATIONASSERT(assertConfigurationSizeCheck,        (sizeof(tConfiguration) == offsetof(tConfiguration, nCRC)+4));
#define CFGPROTOCOL_KEY (uint16_t)offsetof(tConfiguration, nVersion) + \
                        (uint16_t)offsetof(tConfiguration, nPressureSensorOffset) + \
                        (uint16_t)offsetof(tConfiguration, fMinBatteryLevel) + \
                        (uint16_t)offsetof(tConfiguration, fMaxPressureLimit_mmH2O) + \
                        (uint16_t)offsetof(tConfiguration, fMinPressureLimit_mmH2O) + \
                        (uint16_t)offsetof(tConfiguration, fMaxPressureDelta_mmH2O) + \
                        (uint16_t)offsetof(tConfiguration, fMinPressureLimit_Flow) + \
                        (uint16_t)offsetof(tConfiguration, fGainP) + \
                        (uint16_t)offsetof(tConfiguration, fGainI) + \
                        (uint16_t)offsetof(tConfiguration, fGainD) + \
                        (uint16_t)offsetof(tConfiguration, fILimit) + \
                        (uint16_t)offsetof(tConfiguration, fPILimit) + \
                        (uint16_t)offsetof(tConfiguration, fControlTransfer) + \
                        (uint16_t)offsetof(tConfiguration, fPatientTrigger_mmH2O) + \
                        (uint16_t)offsetof(tConfiguration, nServoExhaleOpenAngle) + \
                        (uint16_t)offsetof(tConfiguration, nServoExhaleCloseAngle) + \
                        (uint16_t)offsetof(tConfiguration, nCRC)

HXCOMPILATIONASSERT(assertCheckConfigurationProtocolKey, (CFGPROTOCOL_KEY == 520));
// Uncomment to trace the value of protocol_key at compile-time:    HXCOMPILATIONTRACE(stopCompileCheckSize, CFGPROTOCOL_KEY);
// Uncomment to trace the size of tConfiguration at compile-time:   HXCOMPILATIONTRACE(stopCompileCheckSize, sizeof(tConfiguration));


// Make sure that configuration structure can fit into the EEPROM
HXCOMPILATIONASSERT(assertEEPROMSizeCheck, (sizeof(tConfiguration) <= 512));

/// \fn bool Configuration_Init()
/// \brief Initialize configuration module
bool Configuration_Init();

/// \fn bool Configuration_SetDefaults()
/// \brief Set default configuration
bool Configuration_SetDefaults();

/// \fn bool Configuration_Read()
/// \brief Read configuration from eeprom
bool Configuration_Read();

/// \fn bool Configuration_Write()
/// \brief Write configuration to eeprom
bool Configuration_Write();

#endif // TLC_CONFIGURATION_H
