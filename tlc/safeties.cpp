///
/// \file       safeties.cpp
/// \brief      The Lung Carburetor Firmware safeties module
///
/// \author     Frederic Lauzon
/// \ingroup    safeties
#include "safeties.h"
#include "configuration.h"
#include "datamodel.h"

tSafeties gSafeties;
static uint32_t nMinPressureOkTick = 0;
static uint32_t nMaxPressureOkTick = 0;
enum eSafetiesConsts
{
    kMaxPressureTimeout = 300
};

// Initialize safeties
bool Safeties_Init()
{
    gSafeties.bEnabled              = true;
    gSafeties.bCritical             = false;
    gSafeties.bConfigurationInvalid = false;

    nMinPressureOkTick              = millis();
    nMaxPressureOkTick              = millis();

    return true;
}

void Safeties_Clear()
{
    gSafeties.bCritical             = false;
    gDataModel.nSafetyFlags         = 0;
}

bool Safeties_Enable()
{
    gSafeties.bEnabled = true;
    return true;
}

bool Safeties_Disable()
{
    gSafeties.bEnabled = false;
    return true;
}

// Process safeties checks
void Safeties_Process()
{
    if (gDataModel.nState != kState_Process)
    {
        nMinPressureOkTick = millis();
        nMaxPressureOkTick = millis();
        return;
    }

    if (gDataModel.nRqState != kRqState_Start)
    {
        nMinPressureOkTick = millis();
        nMaxPressureOkTick = millis();
    }


    // If any safety issue, set bCritical in global safeties structure
    if (gSafeties.bEnabled)
    {
        float fPressureDelta = gDataModel.fPressure_mmH2O[0] - gDataModel.fPressure_mmH2O[1];

        gDataModel.nSafetyFlags &= ~kAlarm_CriticalMask;
        if (gDataModel.fPressure_mmH2O[0] >= gConfiguration.fMaxPressureLimit_mmH2O)
        {
            if (gDataModel.nRqState == kRqState_Start)
            {
                if ((millis() - nMaxPressureOkTick) > kMaxPressureTimeout)
                {
                    gDataModel.nSafetyFlags |= kAlarm_MaxPressureLimit;
                    if (gDataModel.fPressure_Flow <= gConfiguration.fMinPressureLimit_Flow)
                    {
                        gDataModel.nSafetyFlags |= kAlarm_CloggedTube;
                    }
                }
            }
        }
        else
        {
            nMaxPressureOkTick = millis();
        }

        if (gDataModel.fPressure_mmH2O[0] <= gConfiguration.fMinPressureLimit_mmH2O)
        {
            if (gDataModel.nRqState == kRqState_Start)
            {
                uint32_t nCycleTimeMs = 10000;
                if (gDataModel.nRespirationPerMinute > 0)
                {
                    nCycleTimeMs = (uint32_t)(60000.0f / (float)gDataModel.nRespirationPerMinute);
                    nCycleTimeMs *= 2; // Check low pressure for 2 respiration cycles
                }
                if ((millis() - nMinPressureOkTick) > nCycleTimeMs)
                {
                    gDataModel.nSafetyFlags |= kAlarm_MinPressureLimit;
                    gDataModel.nSafetyFlags |= kAlarm_DisconnectedTube;
                }
            }
        }
        else
        {
            nMinPressureOkTick = millis();
        }


        if (fabs(fPressureDelta) >= gConfiguration.fMaxPressureDelta_mmH2O)
        {
            gDataModel.nSafetyFlags |= kAlarm_PressureSensorRedudancyFail;
        }

        if (gSafeties.bConfigurationInvalid)
        {
            gDataModel.nSafetyFlags |= kAlarm_InvalidConfiguration;
        }

        if (gDataModel.fBatteryLevel < gConfiguration.fMinBatteryLevel)
        {
            gDataModel.nSafetyFlags |= kAlarm_BatteryLow;
        }

        if ((gDataModel.nSafetyFlags & kAlarm_CriticalMask) != 0)
        {
            gSafeties.bCritical     = true;
            gDataModel.nState       = kState_Error;
        }
    }
}
