///
/// \file       control.cpp
/// \brief      The Lung Carburetor Firmware control module
///
/// \author     Frederic Lauzon
/// \ingroup    control
#include "control.h"
#include "datamodel.h"
#include "configuration.h"
#include "safeties.h"
#include "TimerOne.h"
#include "lcd_keypad.h"

#define USE_RAMPCURVE 0

ServoTimer2 exhaleValveServo;
ServoTimer2 pumpServo;
uint32_t gTickStart = 0;

// Last Error for derivative computation
static float    gLastPressureError = 0.0f;
static uint16_t gMinimumPWM = 0;
const uint16_t  kBlowerPrimeTimeMs = 300;
const uint16_t  kESCPrimePWM = 100;
const float     kCmdFilterGain = 0.5f;
static float    fFilteredCmd = 0.0f;
static float    fTargetCmd   = 0.0f;

bool Control_Init()
{
    gDataModel.nTickRespiration = millis(); // Respiration cycle start tick. Used to compute respiration per minutes
    gDataModel.nTickFromStart = millis();
    //exhaleValveServo.write(gConfiguration.nServoExhaleOpenAngle);

    pumpServo.write(1000);  // DE 1000 a 2000

    gDataModel.nRqState = kRqState_Stop;
    gLastPressureError  = 0.0f;
    gMinimumPWM         = 0;

    return true;
}

bool Control_SetCurveFromDataModel()
{
    if (gDataModel.nRespirationPerMinute > 0)
    {
        float fInhaleTime = gDataModel.fInhaleTime;
        float fExhaleTime = gDataModel.fExhaleTime;

#if USE_RAMPCURVE == 0
        // Last inhale point will last for the remaining inhaleTime
        gDataModel.pInhaleCurve.nCount = 1;
        gDataModel.pInhaleCurve.fSetPoint_mmH2O[gDataModel.pInhaleCurve.nCount-1]  = gDataModel.fInhalePressureTarget_mmH2O;
        gDataModel.pInhaleCurve.nSetPoint_TickMs[gDataModel.pInhaleCurve.nCount-1] = (uint32_t)((fInhaleTime) * 1000.0f);

        gDataModel.pExhaleCurve.nCount = 1;
        gDataModel.pExhaleCurve.fSetPoint_mmH2O[gDataModel.pExhaleCurve.nCount-1]  = gDataModel.fExhalePressureTarget_mmH2O;
        gDataModel.pExhaleCurve.nSetPoint_TickMs[gDataModel.pExhaleCurve.nCount-1] = (uint32_t)((fExhaleTime) * 1000.0f);
        return true;
#else
        // Inhale curve
        uint8_t  rampPointCount = kMaxCurveCount-1;
        uint32_t nPointTickMs   = (uint32_t)((gDataModel.fInhaleRampTime * 1000.0f) / (float)rampPointCount);
        gDataModel.pInhaleCurve.nCount = kMaxCurveCount;

        // Generate inhale ramp time (all points minus last target point)
        float accumulatorRatio = (1.0f / (float)rampPointCount) * gDataModel.fInhalePressureTarget_mmH2O;
        float acc = 0.0f;
        for (int a = 0; a < rampPointCount; ++a)
        {
            gDataModel.pInhaleCurve.fSetPoint_mmH2O[a] = gDataModel.fInhalePressureTarget_mmH2O;//*** temporarry hack acc;
            gDataModel.pInhaleCurve.nSetPoint_TickMs[a] = nPointTickMs;
            acc += accumulatorRatio;

            // Clamp to maximum inhale pressure target
            if (acc > gDataModel.fInhalePressureTarget_mmH2O)
            {
                acc = gDataModel.fInhalePressureTarget_mmH2O;
            }
        }

        // Last inhale point will last for the remaining inhaleTime
        gDataModel.pInhaleCurve.fSetPoint_mmH2O[gDataModel.pInhaleCurve.nCount-1]  = gDataModel.fInhalePressureTarget_mmH2O;
        gDataModel.pInhaleCurve.nSetPoint_TickMs[gDataModel.pInhaleCurve.nCount-1] = (uint32_t)((fInhaleTime - gDataModel.fInhaleRampTime) * 1000.0f);

        // Compute exhale ramp time (all points minus last target point)
        rampPointCount = kMaxCurveCount-1;
        nPointTickMs   = (uint16_t)((gDataModel.fExhaleRampTime * 1000.0f) / (float)rampPointCount);
        gDataModel.pExhaleCurve.nCount = kMaxCurveCount;
        accumulatorRatio = (1.0f / (float)rampPointCount) * (gDataModel.fInhalePressureTarget_mmH2O-gDataModel.fExhalePressureTarget_mmH2O);
        acc = gDataModel.fInhalePressureTarget_mmH2O;
        for (int a = 0; a < rampPointCount; ++a)
        {
            gDataModel.pExhaleCurve.fSetPoint_mmH2O[a] = gDataModel.fExhalePressureTarget_mmH2O; //*** tmeporary hackeacc;
            gDataModel.pExhaleCurve.nSetPoint_TickMs[a] = nPointTickMs;
            acc -= accumulatorRatio;

            // Clamp to maximum inhale pressure target
            if (acc < gDataModel.fExhalePressureTarget_mmH2O)
            {
                acc = gDataModel.fExhalePressureTarget_mmH2O;
            }
        }

        // Last exhale point will last for the remaining exhaleTime
        gDataModel.pExhaleCurve.fSetPoint_mmH2O[gDataModel.pExhaleCurve.nCount-1]  = gDataModel.fExhalePressureTarget_mmH2O;
        gDataModel.pExhaleCurve.nSetPoint_TickMs[gDataModel.pExhaleCurve.nCount-1] = (uint32_t)((fExhaleTime - gDataModel.fExhaleRampTime) * 1000.0f);

        return true;
#endif        
    }

    gSafeties.bConfigurationInvalid = true;
    return false;
 }

// Control using a PID with pressure feedback
void Control_PID()
{ 
    gDataModel.fPressureError = gDataModel.fRequestPressure_mmH2O - gDataModel.fPressure_mmH2O[0];
    gDataModel.fP = gDataModel.fPressureError * gConfiguration.fGainP;

    fTargetCmd += (gDataModel.fPressureError * gConfiguration.fGainI);
  
  fFilteredCmd = fFilteredCmd + (fTargetCmd - fFilteredCmd) * kCmdFilterGain;
  gDataModel.fI = fFilteredCmd;
  
    if (gDataModel.fI > gConfiguration.fILimit)
    {
        gDataModel.fI = gConfiguration.fILimit;
    }
    else if (gDataModel.fI < -gConfiguration.fILimit)
    {
        gDataModel.fI = -gConfiguration.fILimit;
    }

    gDataModel.fD = (gDataModel.fPressureError - gLastPressureError) * gConfiguration.fGainD;
    gLastPressureError = gDataModel.fPressureError;

    gDataModel.fPID = gDataModel.fP + gDataModel.fI + gDataModel.fD;
    if (gDataModel.fPID > gConfiguration.fPIDLimit)
    {
        gDataModel.fPID = gConfiguration.fPIDLimit;
    }
    else if (gDataModel.fPID < -gConfiguration.fPIDLimit)
    {
        gDataModel.fPID = -gConfiguration.fPIDLimit;
    }

    if (gDataModel.fPID < 0)
    {
        gDataModel.fPID = 0;
    }

    gDataModel.nPWMPump = (uint16_t)(gDataModel.fPID * gConfiguration.fControlTransfer);
}

// Return true when condition for respiration has been triggered
static bool CheckTrigger()
{
    bool trigger = false;
    switch (gDataModel.nTriggerMode)
    {
    case kTriggerMode_Timed:
        trigger = ((gDataModel.nRespirationPerMinute > 0) && (millis() - gDataModel.nTickRespiration) >= (60000 / gDataModel.nRespirationPerMinute));

        if ((gDataModel.nRespirationPerMinute > 0) && (millis() - gDataModel.nTickRespiration) >= (uint32_t)((60000 / gDataModel.nRespirationPerMinute) - kBlowerPrimeTimeMs))
        {
            gMinimumPWM = kESCPrimePWM;
        }
        break;

    case kTriggerMode_Patient:
        // If patient triggers respiration
        if (gDataModel.fPressure_mmH2O[0] < gConfiguration.fPatientTrigger_mmH2O)
        {
            trigger = true;
        }
        break;

    case kTriggerMode_PatientSemiAutomatic:
        // If patient triggers respiration or if time for next respiration cycle
        if (gDataModel.fPressure_mmH2O[0] < gConfiguration.fPatientTrigger_mmH2O)
        {
            trigger = true;
        }
        trigger |= ((gDataModel.nRespirationPerMinute > 0) && (millis() - gDataModel.nTickRespiration) >= (60000 / gDataModel.nRespirationPerMinute));
        if ((gDataModel.nRespirationPerMinute > 0) && (millis() - gDataModel.nTickRespiration) >= (uint32_t)((60000 / gDataModel.nRespirationPerMinute) - kBlowerPrimeTimeMs))
        {
            gMinimumPWM = kESCPrimePWM;
        }
        break;

    default:
        // Invalid setting
        gSafeties.bConfigurationInvalid = true;
        gDataModel.nPWMPump             = 0;
        return false;
    };

    return trigger;
}

static bool BeginRespirationCycle()
{
    gDataModel.nTickRespiration = millis(); // Respiration cycle start tick. Used to compute respiration per minutes

    return true;
}

static bool EndRespirationCycle()
{
    return true;
}

static bool StartInhaleCycle()
{
    gMinimumPWM = kESCPrimePWM;
    if (gDataModel.pInhaleCurve.nCount <= 0)
    {
        return false;
    }

    // Make sure exhale valve is closed
    //exhaleValveServo.write(gConfiguration.nServoExhaleCloseAngle);

    // Start a new inhale cycle
    gDataModel.nCurveIndex              = 0;
    gDataModel.nTickSetPoint            = millis();
    gDataModel.fRequestPressure_mmH2O   = gDataModel.pInhaleCurve.fSetPoint_mmH2O[0];

    return true;
}

// Inhale cycle set point, returns true when cycle is finished
static bool Inhale()
{
    // Check for overflow of allowed maximum curve setpoint count
    if (gDataModel.nCurveIndex > kMaxCurveCount)
    {
        // Raise a safety issue
        gSafeties.bCritical     = true;
        gDataModel.nCurveIndex  = gDataModel.pInhaleCurve.nCount;
    }

    if (gDataModel.nCurveIndex >= gDataModel.pInhaleCurve.nCount)
    {
        return true;
    }

    gDataModel.fRequestPressure_mmH2O = gDataModel.pInhaleCurve.fSetPoint_mmH2O[gDataModel.nCurveIndex];
    if ((millis() - gDataModel.nTickSetPoint) >= gDataModel.pInhaleCurve.nSetPoint_TickMs[gDataModel.nCurveIndex])
    {
        gDataModel.nTickSetPoint = millis();
        ++gDataModel.nCurveIndex;
    }

    return false;
}

static bool StopInhaleCycle()
{
    return true;
}

static bool StartExhaleCycle()
{
    gMinimumPWM = 0; // Set minimum pwm to 0 to be able to activate brakes
    if (gDataModel.pExhaleCurve.nCount <= 0)
    {
        return false;
    }

    // Start a new inhale cycle
    gDataModel.nCurveIndex              = 0;
    gDataModel.nTickStartExhale         = millis();
    gDataModel.nTickSetPoint            = millis();
    gDataModel.fRequestPressure_mmH2O   = gDataModel.pExhaleCurve.fSetPoint_mmH2O[0];

    //exhaleValveServo.write(gConfiguration.nServoExhaleOpenAngle);

    return true;
}

static bool Exhale()
{
    // Check for overflow of allowed maximum curve setpoint count
    if (gDataModel.nCurveIndex >= kMaxCurveCount)
    {
        // Raise a safety issue
        gSafeties.bCritical     = true;
        gDataModel.nCurveIndex  = gDataModel.pExhaleCurve.nCount;
    }

    if (gDataModel.nCurveIndex >= gDataModel.pExhaleCurve.nCount)
    {
        return true;
    }

    // During flat part of exhale curve, check for PeepLow and PeepHigh warnings
#if 0 //*** To configure correctly
    if ((millis() - gDataModel.nTickStartExhale) >= (uint32_t)(gDataModel.fExhaleCheckPeepTime * 1000.0f))
    {
        if (gDataModel.fPressure_mmH2O[0] < gDataModel.fPeepLowLimit_mmH2O)
        {
            gDataModel.nSafetyFlags |= kAlarm_PeepLowWarning;
        }
        else
        {
            gDataModel.nSafetyFlags &= ~kAlarm_PeepLowWarning;
        }

        if (gDataModel.fPressure_mmH2O[0] > gDataModel.fPeepHighLimit_mmH2O)
        {
            gDataModel.nSafetyFlags |= kAlarm_PeepHighWarning;
        }
        else
        {
            gDataModel.nSafetyFlags &= ~kAlarm_PeepHighWarning;
        }
    }
#endif

    gDataModel.fRequestPressure_mmH2O = gDataModel.pExhaleCurve.fSetPoint_mmH2O[gDataModel.nCurveIndex];
    if ((millis() - gDataModel.nTickSetPoint) >= gDataModel.pExhaleCurve.nSetPoint_TickMs[gDataModel.nCurveIndex])
    {
        gDataModel.nTickSetPoint = millis();
        ++gDataModel.nCurveIndex;
    }

    return false;
}

static bool StopExhaleCycle()
{
    //exhaleValveServo.write(gConfiguration.nServoExhaleCloseAngle);

    return true;
}

static bool ComputeRespirationSetPoint()
{
    // Process proper part of the respiration cycle: Trigger, Inhale or Exhale
    switch (gDataModel.nCycleState)
    {
    case kCycleState_WaitTrigger:
#if ENABLE_LCD
        sprintf(gLcdDetail, "Trigger   ");
#endif
        if (CheckTrigger())
        {
            BeginRespirationCycle();
            if (StartInhaleCycle())
            {
                // Force accumulated PID errors to zero to force the system to react faster
                gDataModel.fI                       = 0.0f;
                gDataModel.fD                       = 0.0f;
                gDataModel.fPID                     = 0.0f;
                gLastPressureError                  = 0.0f;
        fFilteredCmd            = 0.0f;
        fTargetCmd              = 0.0f;

                gDataModel.nCycleState = kCycleState_Inhale;
            }
            else
            {
                // No inhale curve set
                gSafeties.bCritical = true;
            }
        }
        break;

    case kCycleState_Inhale:
        {
#if ENABLE_LCD
            sprintf(gLcdDetail, "Inhale  ");
#endif
            bool inhaleFinished = Inhale();
            if (inhaleFinished)
            {
                StopInhaleCycle();
                if (StartExhaleCycle())
                {
                    gDataModel.nCycleState = kCycleState_Exhale;
                }
            }
            else
            {
                // No exhale curve set
                gSafeties.bCritical = true;
            }
        }
        break;

    case kCycleState_Exhale:
        {
#if ENABLE_LCD
            sprintf(gLcdDetail, "Exhale   ");
#endif
            bool exhaleFinished = Exhale();
            if (exhaleFinished)
            {
                StopExhaleCycle();
                EndRespirationCycle();
                gDataModel.nTickStabilization = millis();
                gDataModel.nCycleState = kCycleState_Stabilization;
            }
        }
        break;

    case kCycleState_Stabilization:
#if ENABLE_LCD
        sprintf(gLcdDetail, "Stabil   ");
#endif
        // Pressure Stabilization between cycles
        if ((millis() - gDataModel.nTickStabilization) >= kPeriodStabilization)
        {
            gDataModel.nCycleState = kCycleState_WaitTrigger;
        }
        break;

    default:
#if ENABLE_LCD
        sprintf(gLcdDetail, "N/A   ");
#endif
        // Invalid setting
        gSafeties.bConfigurationInvalid = true;
        gDataModel.nPWMPump             = 0;
        gDataModel.nCycleState          = kCycleState_WaitTrigger;
        return false;
    };

    return true;
}

static void ResetRespirationState()
{
    gDataModel.nTickRespiration         = millis(); // Respiration cycle start tick. Used to compute
    gDataModel.nCycleState              = kCycleState_WaitTrigger;
    gDataModel.fI                       = 0.0f;
    gDataModel.fD                       = 0.0f;
    gDataModel.fPID                     = 0.0f;
    gLastPressureError                  = 0.0f;

    gDataModel.nCurveIndex              = 0;
    gDataModel.nTickSetPoint            = 0;
    gDataModel.nPWMPump                 = 0;
    gDataModel.fRequestPressure_mmH2O   = 0.0f;
}

void Control_Process()
{
    if (gDataModel.nRqState != kRqState_Start || gDataModel.nState != kState_Process)
    {
        exhaleValveServo.write(gConfiguration.nServoExhaleOpenAngle);
        gDataModel.nTickFromStart = 0;
        gTickStart = millis();

        pumpServo.write(1000);

        ResetRespirationState();
        return;
    }

    gDataModel.nTickFromStart = millis() - gTickStart;

    // Process pressure feedback to match current pressure set point
    switch (gDataModel.nControlMode)
    {
    case kControlMode_PID:
        // It is assumed that the last pressure setpoint in the exhale curve is kept between respiration
        if (ComputeRespirationSetPoint())
        {
            Control_PID();
        }
        break;

    case kControlMode_FeedForward:
        // In feedforward, the master controls the gDataModel.nPWMPump value through the serial port
        break;

    default:
        // Unknown control mode, raise error.
        gSafeties.bConfigurationInvalid = true;
        gDataModel.nPWMPump             = 0;
        break;
    };

    // Pump power to output, do not allow zero pwm iun inhale    
    if (gDataModel.nPWMPump <= gMinimumPWM)
    {
        gDataModel.nPWMPump = gMinimumPWM;
    }
    
    uint16_t pwm = gDataModel.nPWMPump + 1000;
    if (pwm > 2000)
    {
        pwm = 2000;
    }
    pumpServo.write(pwm);
}
