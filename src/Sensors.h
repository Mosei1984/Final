#ifndef SENSOR_CORRECTIONS_H
#define SENSOR_CORRECTIONS_H

#include <Arduino.h>
#include "Init_System.h"
#include "DH_Parameter.h"
#include "PoseEKF.h"

void getSensorBasedZAndTilt(double outTiltRollPitch[2], double& outCorrectedZ);

void applySensorCorrections(double& zKorrigiert,
                            double& rollKorrigiert,
                            double& pitchKorrigiert);

void sensorsEkfInit(float z0, float pitch0);
void sensorsEkfUpdate(double& zFiltered, double& pitchFiltered);

#endif // SENSOR_CORRECTIONS_H
