///
/// \file       configuration.cpp
/// \brief      The Lung Carburetor Firmware configuration module
///
/// \author     Frederic Lauzon
/// \ingroup    configuration Configuration
#include "configuration.h"
#include <EEPROM.h>
#include "lcd_keypad.h"

tConfiguration gConfiguration;

// Read configuration from eeprom
bool Configuration_Init()
{
    memset(&gConfiguration, 0, sizeof(tConfiguration));

#if 0 // Only use when debugging configuration
    bool bValid = true;
    Configuration_SetDefaults();

#else
    bool bValid = Configuration_Read();
    if (!bValid)
    {
#ifdef ENABLE_LCD
        sprintf(gLcdMsg, "NVM Fail");
#endif
        Serial.println("DEBUG: Invalid config, revert to default");
        Configuration_SetDefaults();
        Configuration_Write();
    }
    else
    {
#ifdef ENABLE_LCD
        sprintf(gLcdMsg, "NVM Success");
#endif
        Serial.println("DEBUG: Config loaded");
    }
#endif

    return bValid;
}

//
// Written by Christopher Andrews.
// CRC algorithm generated by pycrc, MIT licence ( https://github.com/tpircher/pycrc ).
//
static uint32_t CRC32(uint8_t* pBuffer, int len)
{
    const uint32_t crc_table[16] =
    {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
    };

    uint32_t crc = ~0L;
    for (int index = 0; index < len; ++index)
    {
        crc = crc_table[(crc ^ pBuffer[index]) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (pBuffer[index] >> 4)) & 0x0f] ^ (crc >> 4);
        crc = ~crc;
    }

    return crc;
}

bool Configuration_SetDefaults()
{
    gConfiguration.nVersion                 = kEEPROM_Version;
    gConfiguration.fMinBatteryLevel         = 11.0f;
    gConfiguration.nPressureSensorOffset[0] = 0;
    gConfiguration.nPressureSensorOffset[1] = 0;
    gConfiguration.fMaxPressureLimit_mmH2O  = kMPX5010_MaxPressure_mmH2O;
    gConfiguration.fMinPressureLimit_mmH2O  = -kMPX5010_MaxPressure_mmH2O;
    gConfiguration.fMaxPressureDelta_mmH2O  = kMPX5010_MaxPressureDelta_mmH2O;
    gConfiguration.fGainP                   = 50.0f;
    gConfiguration.fGainI                   = 0.00f;
    gConfiguration.fGainD                   = 0.0000f;
    gConfiguration.fILimit                  = 5.0f;
    gConfiguration.fPILimit                 = 1000.0f;
    gConfiguration.fControlTransfer         = 1.5f;
    gConfiguration.fPatientTrigger_mmH2O    = 40.0f;
    gConfiguration.nServoExhaleOpenAngle    = 1500;
    gConfiguration.nServoExhaleCloseAngle   = 750;
    gConfiguration.nCRC                     = 0; // Clear CRC for computation


    return true;
}

// Read configuration from EEPROM, returns false if bad CRC or eeprom version
bool Configuration_Read()
{
    // Read eeprom and check CRC
    uint8_t* pConfiguration = (uint8_t*)&gConfiguration;
    for (size_t a = 0; a < sizeof(tConfiguration); ++a)
    {
        pConfiguration[a] = EEPROM[a];
    }

    uint32_t oemCRC = gConfiguration.nCRC;
    gConfiguration.nCRC = 0; // Clear CRC for computation
    uint32_t cmpCRC = CRC32(pConfiguration, sizeof(tConfiguration));

    bool ret = (oemCRC == cmpCRC) && (gConfiguration.nVersion == kEEPROM_Version);
    if(ret){
        Serial.println("DEBUG: Configuration_Read success");
    }else{
        Serial.println("DEBUG: Configuration_Read failed");
    }   

    return ret;
}

// Write configuration to EEPROM, returns true if successful
bool Configuration_Write()
{
    // Read eeprom and check CRC
    uint8_t* pConfiguration = (uint8_t*)&gConfiguration;
    gConfiguration.nVersion = kEEPROM_Version;
    gConfiguration.nCRC     = 0; // Clear CRC for computation

    uint32_t newCRC         = CRC32(pConfiguration, sizeof(tConfiguration));
    gConfiguration.nCRC     = newCRC;
    for (size_t a = 0; a < sizeof(tConfiguration); ++a)
    {
        EEPROM[a] = pConfiguration[a];
    }

    Serial.println("DEBUG: Configuration_Write success (?)");
    
    return true;
}