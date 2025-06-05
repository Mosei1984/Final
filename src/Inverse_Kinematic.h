// InverseKinematics.h

#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Arduino.h>
#include "DH_Parameter.h"        // DHParams, jointLimits, currentJointAngles, toolLength
#include "Forward_Kinematic.h"   // computeForwardKinematics()
#include <ArduinoEigen.h>           // Für Matrix- und SVD-Operationen

/**
 * @brief Struktur zur Einstellung der IK-Iteration.
 */
struct IKSettings {
    double positionTolerance;   // Toleranz für Positionsfehler (in Metern)
    double orientationTolerance;// Toleranz für Orientierungsfehler (in Radiant)
    double stepSize;            // Skalierungsfaktor für Δθ im Update
    int maxIterations;          // Maximale Anzahl an Iterationen
    IKSettings() : positionTolerance(1e-4),
                   orientationTolerance(1e-3),
                   stepSize(0.5),
                   maxIterations(100) {}
};

/**
 * @brief Führt eine iterative inverse Kinematik mittels Jacobian-Pseudoinverser durch.
 *
 * @param targetPos    Gewünschte Endeffektor-Position [X, Y, Z] in Metern
 * @param targetOri    Gewünschte Endeffektor-Orientierung [Roll, Pitch, Yaw] in Radiant (ZYX-Konvention)
 * @param initialGuess Startwerte für Gelenkwinkel (länge 6, Radiant). Wird in place aktualisiert.
 * @param solution     Ausgabe: Gefundene Gelenkwinkel (länge 6, Radiant). Gültig, wenn Funktion true zurückgibt.
 * @param settings     IKSettings mit Toleranzen, Schrittweite, maxIter.
 * @return true, wenn IK konvergiert (Fehler < Toleranzen), sonst false.
 */
bool computeInverseKinematics(const double targetPos[3],
                              const double targetOri[3],
                              double initialGuess[6],
                              double solution[6],
                              const IKSettings& settings = IKSettings());

#endif // INVERSE_KINEMATICS_H
