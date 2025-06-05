// RemoteControl.h

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <Arduino.h>
#include "Robo_Config_V1.h"  // Enthält joyLXCenter, joyLYCenter, joyRZCenter, joyRYawCenter, DEADZONE, SERVO_POT_PIN

// =====================
// Externe Pin-Definitionen (vom Benutzer in Robo_Config_V1.cpp setzen)
// =====================

extern const uint8_t JOY_LX_PIN;   // Analog-Pin für linken Joystick X-Achse
extern const uint8_t JOY_LY_PIN;   // Analog-Pin für linken Joystick Y-Achse
extern const uint8_t JOY_RZ_PIN;   // Analog-Pin für rechten Joystick Z-Achse
extern const uint8_t JOY_RY_PIN;   // Analog-Pin für rechten Joystick Yaw-Achse

extern const uint8_t BUTTON1_PIN;  // Digital-Pin für Button 1
extern const uint8_t BUTTON2_PIN;  // Digital-Pin für Button 2

// =====================
// Strukturen & Datentypen
// =====================

/**
 * @brief Enthält die aktuellen, berechneten Joystick-Werte (-1.0 bis +1.0) und Button-Zustände.
 */
struct RemoteState {
    float leftX;    // Linker Joystick X-Wert (–1.0 … +1.0)
    float leftY;    // Linker Joystick Y-Wert (–1.0 … +1.0)
    float rightZ;   // Rechter Joystick Z-Wert (–1.0 … +1.0)
    float rightY;   // Rechter Joystick Yaw-Wert (–1.0 … +1.0)

    bool button1;   // true = gedrückt
    bool button2;   // true = gedrückt

    int16_t servoPotRaw;  // Roher ADC-Wert des Servo-Potentiometers (0 … 1023)
};

/**
 * @brief Initialisiert RemoteControl: PinMode für Buttons und interne Variablen.
 *        Muss einmalig in setup() aufgerufen werden.
 */
void remoteInit();

/**
 * @brief  Führt eine Kalibrierung der Joysticks durch, um joyCXCenter usw. zu setzen.
 *         Liest `samples`-mal den Rohwert von jeder Achse und berechnet den Mittelwert.
 *
 * @param samples  Anzahl der Messungen pro Achse (Standard 100)
 */
void calibrateJoysticks(uint16_t samples = 100);

/**
 * @brief  Muss in loop() regelmäßig aufgerufen werden. Liest alle Eingänge (Joysticks,
 *         Buttons, Servo-Potentiometer), wendet Deadzone und Normalisierung an und
 *         speichert in einer internen RemoteState-Instanz.
 */
void updateRemoteInputs();

/**
 * @brief  Gibt einen Zeiger auf die aktuelle RemoteState-Struktur.  
 *         Enthält alle verarbeiteten Joystick-Werte und Button-Zustände.
 */
const RemoteState* getRemoteStatePointer();

#endif // REMOTE_CONTROL_H
