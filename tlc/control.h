///
/// \file       control.h
/// \brief      The Lung Carburetor Firmware control module
///
/// \author     Frederic Lauzon
/// \defgroup   control Respiration control
#ifndef TLC_CONTROL_H
#define TLC_CONTROL_H

#include "common.h"
#include <ServoTimer2.h>
extern ServoTimer2 exhaleValveServo;
extern ServoTimer2 pumpServo;

/// \fn bool Control_Init()
/// \brief Initialize control
bool Control_Init();

/// \fn bool Control_SetCurveFromDataModel()
/// \brief Refresh the respiration curve from stored values in the data model
bool Control_SetCurveFromDataModel();

/// \fn bool Control_Process()
/// \brief Process control
void Control_Process();

#endif // TLC_CONTROL_H
