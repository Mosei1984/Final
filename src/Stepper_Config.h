// Stepper_Config.h

#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Robo_Config_V1.h"

// =====================
// Funktionalprototypen
// =====================

/**
 * @brief  Initialisiert alle 6 Stepper-Motoren: GPIOs, Enable-Pin, MaxSpeed, Acceleration.
 */
void configureSteppers();

/**
 * @brief  Fügt alle motors[i] zur multiStepperGroup hinzu (organisiert koordinierte Bewegungen).
 */
void setupMultiStepper();

/**
 * @brief  Startet eine asynchrone, koordinierte Bewegung aller Achsen zu targetSteps (in Steps).
 *         Bewegungsfortschritt muss anschließend regelmäßig via updateAllSteppers() getaktet werden.
 *
 * @param targetSteps  Länge-6-Array mit Zielpositionen in Steps für Achsen 0..5.
 */
void moveToPositionsAsync(const long targetSteps[6]);

/**
 * @brief  Prüft, ob aktuell noch eine koordinierte MultiStepper-Bewegung läuft.
 * @return true, wenn mindestens eine Achse Ziel nicht erreicht hat.
 */
bool isMultiMoveActive();

/**
 * @brief  Stoppt koordinierte MultiStepper-Bewegung und setzt alle Einzelachsen-Variablen zurück.
 */
void stopAllSteppers();

/**
 * @brief  Interrupt-Service-Routine für STEP-Puls-Generierung (Multi & JointMode).
 *
 *   - Koordiniert via multiStepperGroup.runSpeedToPosition()
 *   - Erzeugt Einzel-Achsen Pulse (JointMode) anhand targetSpeeds[] & stepInterval[].
 */
void stepperISR();

/**
 * @brief  Setzt die Drehgeschwindigkeit (Steps/s) für eine Einzelachse (JointMode).
 *
 * @param axis  Achsenindex 0..5
 * @param speed Gewünschte Schrittgeschwindigkeit in Steps/s (positiv <=> Richtung).
 *              Wenn speed == 0.0, stoppt die Achse.
 */
void setStepperSpeed(uint8_t axis, float speed);

/**
 * @brief  Aktiviert den Motor der gewählten Achse (Enable-Pin aktiv = LOW bei aktiv-low).
 *
 * @param axis  Achsenindex 0..5
 */
void enableStepper(uint8_t axis);

/**
 * @brief  Deaktiviert den Motor der gewählten Achse (Enable-Pin HIGH bei aktiv-low).
 *
 * @param axis  Achsenindex 0..5
 */
void disableStepper(uint8_t axis);

/**
 * @brief  Ruft AccelStepper::run() für alle Einzelachsen auf und
 *         bearbeitet gleichzeitig die aktuelle MultiStepper-Bewegung.
 *
 *  Diese Funktion muss regelmäßig im Haupt-Loop aufgerufen werden,
 *  um:
 *   - JointMode-Bewegung (motors[i].run())
 *   - asynchrone MultiStepper (- runSpeedToPosition() mit multiMoveActive)
 *   am Laufen zu halten.
 */
void updateAllSteppers();

#endif // STEPPER_CONFIG_H
