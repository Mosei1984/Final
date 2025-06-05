// Stepper_Config.cpp

#include "Stepper_Config.h"
#include "Robo_Config_V1.h"

// =====================
// Definition globaler Variablen
// =====================

// MultiStepper-Zielpositionen (in Steps) für alle 6 Achsen
static long multiTargetSteps[6] = { 0, 0, 0, 0, 0, 0 };

// Flag, ob aktuell eine koordinierte Bewegung aktiv ist
static volatile bool multiMoveActive = false;

// Gewünschte Schrittgeschwindigkeiten für Einzelachsen (JointMode), in Steps/s.
static volatile float targetSpeeds[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// Intervall (in µs) zwischen zwei STEP-Impulsen für jede Achse (6).
static volatile unsigned long stepInterval[6] = { 0, 0, 0, 0, 0, 0 };

// Letzte Zeit (in µs) eines STEP-Impulses für jede Achse (6).
static volatile unsigned long lastStepMicros[6] = { 0, 0, 0, 0, 0, 0 };

// Aktueller Zustand des STEP-Pins (HIGH oder LOW) für jede Achse (6)
static volatile bool stepPinState[6] = { false, false, false, false, false, false };

// =====================
// Funktionale Implementierungen
// =====================

void configureSteppers() {
    for (uint8_t i = 0; i < 6; i++) {
        // Enable-Pin (aktiv-low): HIGH → Motor inaktiv
        pinMode(ENABLE_PINS[i], OUTPUT);
        digitalWrite(ENABLE_PINS[i], HIGH);

        // STEP- & DIR-Pin
        pinMode(STEP_PINS[i], OUTPUT);
        digitalWrite(STEP_PINS[i], LOW);
        pinMode(DIR_PINS[i], OUTPUT);
        // Standard-Drehrichtung basierend auf MOTOR_DIRECTION[]
        digitalWrite(DIR_PINS[i], MOTOR_DIRECTION[i] ? LOW : HIGH);

        // AccelStepper-Objekte konfigurieren
        motors[i].setEnablePin(ENABLE_PINS[i]);
        motors[i].setPinsInverted(false, false, true); // ENABLE invertiert (aktiv-low)
        motors[i].setMaxSpeed(DEFAULT_MAX_SPEED);
        motors[i].setAcceleration(DEFAULT_ACCELERATION);
    }
}

void setupMultiStepper() {
    for (uint8_t i = 0; i < 6; i++) {
        multiStepperGroup.addStepper(motors[i]);
    }
}

void moveToPositionsAsync(const long targetSteps[6]) {
    for (uint8_t i = 0; i < 6; i++) {
        multiTargetSteps[i] = targetSteps[i];
    }
    multiStepperGroup.moveTo(multiTargetSteps);
    multiMoveActive = true;
}

bool isMultiMoveActive() {
    return multiMoveActive;
}

void stopAllSteppers() {
    // Stoppe koordinierte Bewegung
    multiMoveActive = false;
    // Stoppe jeden Motor und deaktiviere
    for (uint8_t i = 0; i < 6; i++) {
        motors[i].stop();
        digitalWrite(ENABLE_PINS[i], HIGH);
        targetSpeeds[i]   = 0.0f;
        stepInterval[i]   = 0;
        stepPinState[i]   = false;
        digitalWrite(STEP_PINS[i], LOW);
    }
}

void stepperISR() {
    // ===== Einzelachsen (JointMode) – STEP-Pulse =====
    unsigned long now = micros();
    for (uint8_t i = 0; i < 6; i++) {
        if (targetSpeeds[i] != 0.0f && stepInterval[i] > 0) {
            if ((now - lastStepMicros[i]) >= stepInterval[i]) {
                stepPinState[i] = !stepPinState[i];
                digitalWrite(STEP_PINS[i], stepPinState[i] ? HIGH : LOW);
                lastStepMicros[i] = now;
            }
        }
    }
}

void setStepperSpeed(uint8_t axis, float speed) {
    if (axis >= 6) return;
    noInterrupts();
    if (speed == 0.0f) {
        targetSpeeds[axis] = 0.0f;
        stepInterval[axis] = 0;
        stepPinState[axis] = false;
        digitalWrite(STEP_PINS[axis], LOW);
        interrupts();
        return;
    }
    // DIR-Pin setzen basierend auf Vorzeichen & MOTOR_DIRECTION
    if (speed > 0.0f) {
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? LOW : HIGH);
    } else {
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? HIGH : LOW);
    }
    targetSpeeds[axis] = speed;
    float absSpeed = fabsf(speed);
    stepInterval[axis] = (absSpeed < 1e-6f) ? 0 : (unsigned long)(1000000.0f / absSpeed);
    lastStepMicros[axis] = micros();
    stepPinState[axis]   = false;
    digitalWrite(STEP_PINS[axis], LOW);
    interrupts();
}

void enableStepper(uint8_t axis) {
    if (axis >= 6) return;
    digitalWrite(ENABLE_PINS[axis], LOW); // aktiv-low
}

void disableStepper(uint8_t axis) {
    if (axis >= 6) return;
    digitalWrite(ENABLE_PINS[axis], HIGH);
}

void updateAllSteppers() {
    // 1) Einzelachsen laufen lassen
    for (uint8_t i = 0; i < 6; i++) {
        motors[i].run();
    }
    // 2) Koordinierte MultiStepper-Bewegung vorantreiben
    if (multiMoveActive) {
        multiStepperGroup.runSpeedToPosition();
        bool stillMoving = false;
        for (uint8_t i = 0; i < 6; i++) {
            if (motors[i].distanceToGo() != 0) {
                stillMoving = true;
                break;
            }
        }
        if (!stillMoving) {
            multiMoveActive = false;
        }
    }
}
