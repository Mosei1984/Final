// ForwardKinematics.h

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Arduino.h>
#include "DH_Parameter.h"      // enthält DHParams robotDHParams[6]
/**
 * @brief Berechnet die Vorwärtskinematik für den 6-DOF-Roboterarm.
 *        Aus den aktuellen Gelenkwinkeln (currentJointAngles[]) wird die
 *        homogene Transformationsmatrix T_0_6 ermittelt. Anschließend
 *        werden Endeffektor-Position (X, Y, Z) und Orientierung (Roll, Pitch, Yaw)
 *        ausgegeben.
 *
 * @param jointAngles  Array von 6 Gelenkwinkeln (in Radiant). Die Reihenfolge:
 *                     [0]=Basis, [1]=Schulter, [2]=Ellbogen, [3]=Wrist-Pitch,
 *                     [4]=Wrist-Roll, [5]=Tool-Roll (falls vorhanden).
 *
 * @param endPos       Ausgabeparameter: Endeffektor-Position [3] (in Metern)
 * @param endOri       Ausgabeparameter: Endeffektor-Orientierung [3] (Roll, Pitch, Yaw in Radiant)
 */
void computeForwardKinematics(const double jointAngles[6],
                              double endPos[3],
                              double endOri[3]);

#endif // FORWARD_KINEMATICS_H
