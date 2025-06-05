#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "Robo_Config_V1.h"
#include "DH_Parameter.h"  // für currentJointAngles[], ggf. jointLimits[]
#include <math.h>
//
// --- Homing-Konstanten pro Achse ---
// Wir definieren hier, in welchem Winkel (Joint‐Frame) der Endschalter liegt.
// Direkt nach Homing soll AccelStepper::currentPosition() so eingestellt sein,
// dass currentJointAngles[i] diesen Winkel repräsentiert.
//

// 1) Winkel (in Grad) des Endschalters relativ zur „Null‐Stellung“ jeder Achse:
constexpr float BASE_HOMEPOS_DEG      = -15.0f;  // Basis‐Endschalter liegt bei –15°
constexpr float SHOULDER_HOMEPOS_DEG  = -30.0f;  // Schulter‐Endschalter bei –30°
constexpr float ELBOW_HOMEPOS_DEG     =  -60.0f; // Ellbogen‐Endschalter bei –60°
constexpr float WRIST_PITCH_HOMEPOS_DEG = -125.0f; // Wrist‐Pitch‐Endschalter bei –125°
constexpr float WRIST_ROLL_HOMEPOS_DEG    = -125.0f; // Wrist‐Roll‐Endschalter bei –125°
constexpr float TOOL_ROLL_HOMEPOS_DEG     = -125.0f; // Tool‐Roll‐Endschalter bei –125°
// (Bei den letzten beiden Achsen ggf. anders, falls Greifer‐Servo oder Wrist‐Roll nicht gehomed wird)

// 2) Mechanische Maximalbereiche (für Kinematik‐Grenzen, in Grad):
constexpr float BASE_MAX_ANGLE_DEG     =  180.0f; 
constexpr float SHOULDER_MAX_ANGLE_DEG =  150.0f; 
constexpr float ELBOW_MAX_ANGLE_DEG    =  120.0f; 
constexpr float WRIST_PITCH_MAX_DEG    =  180.0f; 


// --- Backoff- und Timeout-Parameter ---
// Wie viele Schritte sollen wir nach Endschalter-Erkennung zurueckfahren, damit
// wir einen definierten Wiederanlauf-Punkt haben.
// Der konkrete Wert ist in Robo_Config_V1.h als HOMING_BACKOFF_STEPS festgelegt.
//

// --- Backoff- und Timeout-Parameter ---
// Wie viele Schritte sollen wir nach Endschalter-Erkennung zurueckfahren, damit
// wir einen definierten Wiederanlauf-Punkt haben.
// Der konkrete Wert ist in Robo_Config_V1.h als HOMING_BACKOFF_STEPS festgelegt.
//
// Zusätzliche Sicherheit: Maximale Zeit für eine Homingfahrt, um Endlosschleifen
// bei defekten Endschaltern zu vermeiden (in Millisekunden).
constexpr unsigned long MAX_HOMING_TIME_MS = 60000; // 1 Minute


//
// --- Funktionen und globale Flags ---
//

/**
 * @brief  Homingt jede einzelne Achse: fahrt langsam in Richtung Endschalter, bis dieser
 *         auslöst, backoff, Position geringfügig korrigieren und interne Step-Position
 *         auf den Soll‐Offset (in Steps) setzen.
 *
 * @param  axis  Index der Achse (0 = Basis, 1 = Schulter, 2 = Ellbogen, 3 = Wrist Pitch, …).
 */

// Rueckgabe: true bei erfolgreichem Homing, false bei Timeout
bool homeAxis(uint8_t axis);


/**
 * @brief  Hintereinander alle Axes 0..3 (bzw. 0..5, je nach Mechanik) homen. 
 *         Nach jeder erfolgreichen Achse speichert es currentJointAngles[axis] = HOMEPOS_RAD.
 *         Am Ende fährt er in die vorgegebenen Kalibrierwinkel (siehe Homing.cpp).
 */

// Homt alle definierten Achsen hintereinander. Liefert false, wenn eine Achse
// nicht innerhalb des Zeitlimits homed.
bool homeAllAxes();

/**
 * @brief  Checkt für eine Achse, ob deren Endschalter ausgelöst ist.
 *         Bei NC-Schaltern mit Pull-Up bedeutet HIGH = gedrückt. Rückgabe: true bei HIGH.

 *         Implementierung siehe Homing.cpp.
 */
bool isEndstopPressed(uint8_t axis);

/**
 * @brief  Nach dem Homing: Fährt alle Achsen in die vordefinierten Kalibrierwinkel,
 *         kalibriert ggf. die Sensoren (ADXL, VL53L0X) und schreibt
 *         currentJointAngles[] in die Initialposition (in Radiant).
 *
 *         Diese Funktion wird von homeAllAxes() aufgerufen.
 */
void moveToCalibrationPose();

#endif // HOMING_H
