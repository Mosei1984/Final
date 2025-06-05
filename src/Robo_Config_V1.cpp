// Robo_Config_V1.cpp

#include "Robo_Config_V1.h"

// =====================
// Joystick-Mittelwerte (wird in InitSystem oder main gesetzt)
// =====================
int16_t joyLXCenter  = 0;
int16_t joyLYCenter  = 0;
int16_t joyRZCenter  = 0;
int16_t joyRYawCenter = 0;

// =====================
// Homing- und Motor-Richtungen
// =====================
// HOMING_ENABLED: true = Homing für diese Achse aktivieren
const bool HOMING_ENABLED[6] = {
    true,  // Achse 0 (Basis)
    true,  // Achse 1 (Schulter)
    true,  // Achse 2 (Ellbogen)
    true,  // Achse 3 (Wrist Pitch)
    false, // Achse 4 (Wrist Roll, falls nicht gehomed)
    false  // Achse 5 (Tool Roll/Greifer, falls nicht gehomed)
};

// MOTOR_DIRECTION: true = invertierte Drehrichtung (positive logische Richtung)
// Wird von configureSteppers() verwendet, um DIR-Pin richtig zu setzen.
bool MOTOR_DIRECTION[6] = {
    false, // Achse 0: Basis (keine Inversion)
    false, // Achse 1: Schulter
    true,  // Achse 2: Ellbogen (invertiert)
    false, // Achse 3: Wrist Pitch
    true,  // Achse 4: Wrist Roll (invertiert)
    false  // Achse 5: Tool Roll/Greifer
};

// HOMING_DIRECTION: true = positive logische Richtung beim Homing
// Wird in Homing.cpp verwendet, um softwareseitig in Richtung Endstop zu fahren.
bool HOMING_DIRECTION[6] = {
    false, // Achse 0: Basis fährt in negative Richtung zum Endschalter
    true,  // Achse 1: Schulter fährt in positive Richtung
    false, // Achse 2: Ellbogen fährt in negative Richtung
    true,  // Achse 3: Wrist Pitch fährt in positive Richtung
    false, // Achse 4: Wrist Roll (wenn homing nötig)
    false  // Achse 5: Tool Roll/Greifer (wenn homing nötig)
};

// =====================
// Display-Pointer (wird in InitSystem.cpp initialisiert)
// =====================
U8G2* displayPtr = nullptr;

// =====================
// Globale Zeitbasis (z. B. für Joystick-Abfrage, Debouncing)
// =====================
unsigned long lastJoystickCheck = 0;

// =====================
// AccelStepper-Objekte für jede der 6 Achsen
//    Konstruktor: (StepperType, stepPin, dirPin)
//    STEP_PINS[] und DIR_PINS[] kommen aus Robo_Config_V1.h
// =====================
AccelStepper motors[6] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[4], DIR_PINS[4]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[5], DIR_PINS[5])
};

// =====================
// MultiStepper-Gruppe für koordinierte Bewegungen
// =====================
MultiStepper multiStepperGroup;

// =====================
// Ende Robo_Config_V1.cpp
// =====================
