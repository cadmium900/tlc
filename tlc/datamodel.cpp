///
/// \file       datamodel.cpp
/// \brief      The Lung Carburetor Firmware datamodel module
///
/// \author     Frederic Lauzon
/// \ingroup    datamodel
#include "datamodel.h"
#include "control.h"

tDataModel gDataModel;

bool DataModel_Init()
{
    memset(&gDataModel, 0, sizeof(tDataModel));

    gDataModel.pInhaleCurve.nCount = 8;
    for (int a = 0; a < 8; ++a)
    {
        gDataModel.pInhaleCurve.nSetPoint_TickMs[a] = 100;
        gDataModel.pInhaleCurve.fSetPoint_mmH2O[a]  = 250.0f;
    }

    gDataModel.pExhaleCurve.nCount = 8;
    for (int a = 0; a < 8; ++a)
    {
        gDataModel.pExhaleCurve.nSetPoint_TickMs[a] = 100;
        gDataModel.pExhaleCurve.fSetPoint_mmH2O[a]  = 80.0f;
    }
    gDataModel.pExhaleCurve.fSetPoint_mmH2O[7]  = 0.0f;

    gDataModel.nRespirationPerMinute    = 12;
    gDataModel.nControlMode             = kControlMode_PID;
    gDataModel.nTriggerMode             = kTriggerMode_Timed;

    gDataModel.fInhalePressureTarget_mmH2O  = 250.0f; // 25 cmH2O
    gDataModel.fExhalePressureTarget_mmH2O  = 50.0f;  // 5 cmH2O
    gDataModel.fInhaleTime                  = 1.0f;
    gDataModel.fExhaleTime                  = 1.0f;
    gDataModel.fInhaleRampTime              = 0.1f;
    gDataModel.fExhaleRampTime              = 0.1f;
    gDataModel.fExhaleCheckPeepTime         = 0.75f;
    gDataModel.fPeepLowLimit_mmH2O          = 2.0f;
    gDataModel.fPeepHighLimit_mmH2O         = 25.0f;
    Control_SetCurveFromDataModel();

    gDataModel.nState = kState_Init;
    return true;
}
