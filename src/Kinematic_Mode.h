// KinematicMode.h

#ifndef KINEMATIC_MODE_H
#define KINEMATIC_MODE_H

#include <Arduino.h>
#include "Remote.h"
#include "Inverse_Kinematic.h"
#include "Forward_Kinematic.h"
#include "Stepper_Config.h"
#include "Robo_Config_V1.h"
#include "Menu.h"  // Für KS_COUNT, KS_…-Symbole

/**
 * @brief Initialisiert Kinematic Mode: setzt interne Variablen zurück.
 *        Muss einmalig in setup() aufgerufen werden, bevor kinematicModeUpdate() verwendet wird.
 */
void kinematicModeInit();

/**
 * @brief  Muss in loop() regelmäßig aufgerufen werden, wenn Kinematic Mode aktiv ist.
 *         - Navigiert im Untermenü (Sensor Toggle, Set Position, Go To Position) via rechter Joystick Y.
 *         - Bestätigt Auswahl via Button1.
 *         - In "Set Position" passt man Zielkoordinaten (X/Y via linker Joystick; Z via rechter Joystick X) an.
 *         - In "Go To Position" versucht man, über Inverse-Kinematik zu den Zielkoordinaten zu fahren.
 *         - Button2 kehrt jeweils eine Ebene zurück.
 */
void kinematicModeUpdate();

/**
 * @brief  Beendet Kinematic Mode: stoppt alle Achsen und setzt eventuell laufende IK-Wünsche zurück.
 */
void kinematicModeStop();

/**
 * @brief  Gibt zurück, ob die Sensoren (Laser+ADXL) aktuell im Kinematic Mode aktiviert sind.
 */
bool areSensorsEnabled();

#endif // KINEMATIC_MODE_H
