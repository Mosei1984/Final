// RemoteControl.cpp

#include "Remote.h"

// =====================
// Interne Variablen
// =====================

// Aktueller Remote-Zustand (wird in updateRemoteInputs() aktualisiert)
static RemoteState currentState = {
    0.0f, 0.0f, 0.0f, 0.0f,
    false, false,
    0
};

// =============================================================================
// remoteInit()
//
// Setzt PinModes für Buttons und initialisiert Joystick-Center.
// =============================================================================
void remoteInit() {
    // Buttons als Input mit Pullup (angenommen aktive LOW-Schaltung)
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    // Joystick-Pins: Analog, keine explizite pinMode nötig auf ARM/Teensy

    // Servo-Potentiometer: Analog, ebenfalls keine pinMode nötig

    // Initiale Deadzone-Mittelwerte aus Robo_Config_V1 (joyLXCenter etc.) bleiben.
}

// =============================================================================
// calibrateJoysticks(samples)
//
// Liest `samples`-mal pro Achse und setzt joy*Center auf den Durchschnitt.
// =============================================================================
void calibrateJoysticks(uint16_t samples) {
    uint32_t sumLX = 0, sumLY = 0, sumRZ = 0, sumRY = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sumLX += analogRead(JOY_LX_PIN);
        sumLY += analogRead(JOY_LY_PIN);
        sumRZ += analogRead(JOY_RZ_PIN);
        sumRY += analogRead(JOY_RYAW_PIN);
        delay(5);
    }
    joyLXCenter   = sumLX / samples;
    joyLYCenter   = sumLY / samples;
    joyRZCenter   = sumRZ / samples;
    joyRYawCenter = sumRY / samples;
}

// =============================================================================
// Intern: Wendet Deadzone auf einen rohen ADC-Wert an
//
// @param raw    Roher ADC-Wert (0..1023)
// @param center Center-Wert (joy*Center), ermittelt durch Kalibrierung
// @return Normierter Wert in [-1.0, +1.0] mit Deadzone
// =============================================================================
static float applyJoystickDeadzone(int16_t raw, int16_t center) {
    int16_t delta = raw - center;
    // Falls im Deadzone-Bereich, zurück 0.0
    if (abs(delta) <= DEADZONE) {
        return 0.0f;
    }
    // Normiere – wenn delta positiv, Max = (1023-center); wenn negativ, Min = (center-0)
    if (delta > 0) {
        return (float)(delta - DEADZONE) / (float)(1023 - center - DEADZONE);
    } else {
        return (float)(delta + DEADZONE) / (float)(center - DEADZONE);
    }
}

// =============================================================================
// updateRemoteInputs()
//
// Liest alle Joysticks, Buttons, Servo-Pot; wendet Deadzone und speichert im currentState.
// =============================================================================
void updateRemoteInputs() {
    // 1) Rohwerte von Joysticks lesen
    int16_t rawLX = analogRead(JOY_LX_PIN);
    int16_t rawLY = analogRead(JOY_LY_PIN);
    int16_t rawRZ = analogRead(JOY_RZ_PIN);
    int16_t rawRY = analogRead(JOY_RYAW_PIN);

    // 2) Deadzone und Normierung anwenden
    currentState.leftX  = applyJoystickDeadzone(rawLX, joyLXCenter);
    currentState.leftY  = applyJoystickDeadzone(rawLY, joyLYCenter);
    currentState.rightZ = applyJoystickDeadzone(rawRZ, joyRZCenter);
    currentState.rightY = applyJoystickDeadzone(rawRY, joyRYawCenter);

    // 3) Buttons lesen (aktives LOW: pressed == LOW)
    currentState.button1 = (digitalRead(BUTTON1_PIN) == LOW);
    currentState.button2 = (digitalRead(BUTTON2_PIN) == LOW);

    // 4) Servo-Potentiometer lesen (roher ADC 0..1023)
    currentState.servoPotRaw = analogRead(SERVO_POT_PIN);
}

// =============================================================================
// getRemoteStatePointer()
//
// Gibt Zeiger auf aktuelle RemoteState-Struktur.
// =============================================================================
const RemoteState* getRemoteStatePointer() {
    return &currentState;
}
