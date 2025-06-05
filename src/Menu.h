// Menu.h

#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include "Remote.h"     // Für Zugriffe auf Joystick & Buttons
#include "Robo_Config_V1.h"    // Für Display-Pointer (displayPtr), Deadzone etc.

/**
 * @brief Hauptmenüeinträge.
 */
enum MainMenuItem {
    MM_HOMING = 0,
    MM_JOINT,
    MM_KINEMATIC,
    MM_GCODE,
    MM_TEACH_PLAY,
    MM_COUNT
};

/**
 * @brief Untermenü für Homing-Optionen.
 */
enum HomingSubItem {
    HS_SINGLE_AXIS = 0,
    HS_ALL_AXES,
    HS_MOVE_INIT_POS,
    HS_AUTO_HOMING,
    HS_HOMING_BACK,   // "Zurück"
    HS_COUNT
};

/**
 * @brief Untermenü für Kinematic-Optionen.
 */
enum KinematicSubItem {
    KS_SENSORS_TOGGLE = 0,
    KS_SET_POSITION,
    KS_GOTO_POSITION,
    KS_KIN_BACK,      // "Zurück"
    KS_COUNT
};

/**
 * @brief Die ausgewählte Aktion nach Verlassen des Menüs.
 *        mainIndex gibt den Hauptmodus (0..MM_COUNT-1) an.
 *        subIndex ist -1, wenn kein Untermenü (z.B. Joint/Gcode/TeachPlay) gewählt wurde,
 *        ansonsten < HS_COUNT oder KS_COUNT.
 */
struct MenuSelection {
    int8_t mainIndex;   // Index in MainMenuItem (0..MM_COUNT-1)
    int8_t subIndex;    // -1 oder Index in jeweiligem Untermenü
};

/**
 * @brief Initialisiert das Menü-System. Muss einmalig in setup() aufgerufen werden.
 */
void menuInit();

/**
 * @brief Muss in loop() regelmäßig aufgerufen werden.
 *        Liest Joystick/Buttons, navigiert im Menü, zeichnet das Display,
 *        und legt bei Auswahl in selection fest, dass eine Wahl getroffen wurde.
 */
void menuUpdate();

/**
 * @brief Prüft, ob der Benutzer aktuell eine Auswahl bestätigt hat (Haupt- oder Untermenü).
 * @return true, wenn eine Auswahl vorliegt (einmalig), sonst false.
 */
bool menuSelectionAvailable();

/**
 * @brief Liefert den aktuellen Menü-Wahl-Fortschritt. Gültig, wenn menuSelectionAvailable() true lieferte.
 *        Nach dem Auslesen sollte menuResetSelection() aufgerufen werden.
 */
MenuSelection menuGetSelection();

/**
 * @brief Setzt den internen Auswahl-Status zurück, damit menuSelectionAvailable() erneut auf false geht.
 */
void menuResetSelection();

#endif // MENU_H
