// SensorCorrections.h

#ifndef SENSOR_CORRECTIONS_H
#define SENSOR_CORRECTIONS_H

#include <Arduino.h>
#include "Init_System.h"        // enthält getTiltAngleRad(), getCorrectedLaserHeight()
#include "DH_Parameter.h"      // currentJointAngles[], robotDHParams[], toolLength

/**
 * @brief Liest aus den Sensoren (ADXL345 + VL53L0X) die aktuelle Neigung (Roll, Pitch)
 *        und die gemessene Höhe (Z), korrigiert um Neigung. Gibt beides zurück.
 *
 * @param outTiltRollPitch   Ausgabe: [0]=Roll (Rad), [1]=Pitch (Rad)
 * @param outCorrectedZ      Ausgabe: Vertikale Höhe (Meter), basierend auf Laser + Tilt
 */
void getSensorBasedZAndTilt(double outTiltRollPitch[2], double& outCorrectedZ);

/**
 * @brief  Führt eine laufende Sensorfusion aus, um:
 *         1) die aktuelle Z-Position (in m) des Endeffektors zu korrigieren,
 *            basierend auf Laser + Neigung
 *         2) die aktuellen Roll- und Pitch-Winkel (in Radiant) zu korrigieren,
 *            basierend auf den ADXL-Messwerten
 *
 *         Diese Funktion sollte idealerweise im Loop() in regelmäßigen Abständen
 *         aufgerufen werden, um die kinematisch errechneten Werte zu justieren.
 *
 * @param zKorrigiert        Ausgabe: Korrigierte Endeffektor-Höhe (m)
 * @param rollKorrigiert     Ausgabe: Korrigierter Roll-Winkel (Rad)
 * @param pitchKorrigiert    Ausgabe: Korrigierter Pitch-Winkel (Rad)
 */
void applySensorCorrections(double& zKorrigiert, double& rollKorrigiert, double& pitchKorrigiert);

#endif // SENSOR_CORRECTIONS_H
