// JointMode.h

#ifndef JOINT_MODE_H
#define JOINT_MODE_H

#include <Arduino.h>
#include "Remote.h"
#include "Stepper_Config.h"
#include "Robo_Config_V1.h"

// =====================
// Funktionen zum Joint Mode
// =====================

/**
 * @brief Initialisiert Joint Mode: setzt Auswahl auf Achse 0 und stoppt alle Achsen.
 *        Muss in setup() aufgerufen werden, bevor jointModeUpdate() verwendet wird.
 */
void jointModeInit();

/**
 * @brief  Muss in loop() regelmäßig aufgerufen werden, wenn Joint Mode aktiv ist.
 *         Liest Joystick-Werte, wechselt ausgewählte Achse via rechter Joystick Y-Achse
 *         und steuert Geschwindigkeit der aktuell selektierten Achse anhand linker Joystick Y-Achse.
 */
void jointModeUpdate();

/**
 * @brief  Beendet Joint Mode: stoppt aktuell selektierte Achse (setzt alle Achsen auf Geschwindigkeit 0).
 *         Kann vor Moduswechsel aufgerufen werden.
 */
void jointModeStop();

#endif // JOINT_MODE_H
