// Homing.cpp

#include "Homing.h"
 // Enthält MICROSTEPPING, STEPS_PER_REVOLUTION, ENDSTOP_PINS[], motors[], DIR_PINS[], ENABLE_PINS[], MOTOR_DIRECTION[], HOMING_DIRECTION[], multiStepperGroup


// Hilfsfunktion: Grad → Radiant
static inline float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

// ----------------------------------------------------------------------------
// Prüft, ob Endstop (INPUT_PULLUP) der Achse gedrückt ist (LOW = gedrückt)
// ----------------------------------------------------------------------------
bool isEndstopPressed(uint8_t axis) {
    return (digitalRead(ENDSTOP_PINS[axis]) == LOW);
}

// ----------------------------------------------------------------------------
// Setzt AccelStepper-Positionszähler auf offsetSteps
// ----------------------------------------------------------------------------
static void setStepperPositionToOffset(uint8_t axis, long offsetSteps) {
    motors[axis].setCurrentPosition(offsetSteps);
}

// ----------------------------------------------------------------------------
// Homing einer einzelnen Achse
// 1) Fahrt mit HOMING_FAST_SPEED in Richtung HOMING_DIRECTION[axis], bis Endstop auslöst
// 2) Backoff um BACKOFF_STEPS
// 3) Interne Position auf HOMEPOS_DEG-Offset setzen und currentJointAngles aktualisieren
// ----------------------------------------------------------------------------
void homeAxis(uint8_t axis) {
    Serial.print("Homing axis ");
    Serial.println(axis);
    // 1) Homingfahrt starten
    float fastSpeed = HOMING_FAST_SPEED;
    if (HOMING_DIRECTION[axis]) {
        // Positive logische Richtung
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? LOW : HIGH);
        motors[axis].setSpeed(+fastSpeed);
    } else {
        // Negative logische Richtung
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? HIGH : LOW);
        motors[axis].setSpeed(-fastSpeed);
    }
    // Motor aktivieren (Enable LOW)
    digitalWrite(ENABLE_PINS[axis], LOW);

    // Solange fahren, bis Endstop auslöst
    while (!isEndstopPressed(axis)) {
        motors[axis].runSpeed();
    }

    // 2) Endstop erkannt: anhalten, kurze Pause, dann Backoff
    motors[axis].stop();
    delay(10);

    // Backoff in Gegenrichtung
    long backoff = BACKOFF_STEPS;
    if (HOMING_DIRECTION[axis]) {
        // Umkehr auf negative logische Richtung
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? HIGH : LOW);
    } else {
        // Umkehr auf positive logische Richtung
        digitalWrite(DIR_PINS[axis], MOTOR_DIRECTION[axis] ? LOW : HIGH);
    }
    motors[axis].setSpeed(+fastSpeed);
    long count = 0;
    while (count < backoff) {
        motors[axis].runSpeed();
        count++;
    }
    motors[axis].stop();
    delay(5);

    // 3) Interne Steps-Position auf HOMEPOS_DEG setzen und Winkel speichern
    float stepsPerDeg = (float)(STEPS_PER_REVOLUTION * MICROSTEPPING) / 360.0f;
    long offsetSteps = 0;
    switch (axis) {
        case 0:
            offsetSteps = (long)(BASE_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[0] = deg2rad(BASE_HOMEPOS_DEG);
            break;
        case 1:
            offsetSteps = (long)(SHOULDER_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[1] = deg2rad(SHOULDER_HOMEPOS_DEG);
            break;
        case 2:
            offsetSteps = (long)(ELBOW_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[2] = deg2rad(ELBOW_HOMEPOS_DEG);
            break;
        case 3:
            offsetSteps = (long)(WRIST_PITCH_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[3] = deg2rad(WRIST_PITCH_HOMEPOS_DEG);
            break;
        case 4:
            offsetSteps = (long)(WRIST_ROLL_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[4] = deg2rad(WRIST_ROLL_HOMEPOS_DEG);
            break;
        case 5:
            offsetSteps = (long)(TOOL_ROLL_HOMEPOS_DEG * stepsPerDeg);
            currentJointAngles[5] = deg2rad(TOOL_ROLL_HOMEPOS_DEG);
            break;
        default:
            offsetSteps = 0;
            currentJointAngles[axis] = 0.0f;
            break;
    }
    setStepperPositionToOffset(axis, offsetSteps);

    // Motor deaktivieren (Enable HIGH)
    digitalWrite(ENABLE_PINS[axis], HIGH);
    delay(10);
    Serial.print("Axis ");
    Serial.print(axis);
    Serial.println(" homed");
}

// ----------------------------------------------------------------------------
// Homing aller Achsen (0..3) und anschließende Kalibrierpose
// ----------------------------------------------------------------------------
void homeAllAxes() {
    Serial.println("Starting homing sequence");
    // Endstop-Pins auf INPUT_PULLUP
    for (uint8_t i = 0; i < 6; i++) {
        pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
    }

    // Homing Reihenfolge
    homeAxis(0);
    homeAxis(1);
    homeAxis(2);
    homeAxis(3);
    // Optional: homing für 4, 5, falls benötigt:
    // homeAxis(4);
    // homeAxis(5);

    // Anschließend in Kalibrierpose fahren
    moveToCalibrationPose();
    Serial.println("Homing sequence done");
}

// ----------------------------------------------------------------------------
// Fahrt in Kalibrierpose nach Homing:
//   - Koordinierte Bewegung zu definierten 0°-Winkeln
//   - Sensor-Kalibrierung (ADXL, VL53L0X)
//   - Setze currentJointAngles auf 0
// ----------------------------------------------------------------------------
void moveToCalibrationPose() {
    // Kalibrierwinkel (Grad)
    constexpr float BASE_CALIB_DEG        = 0.0f;
    constexpr float SHOULDER_CALIB_DEG    = 0.0f;
    constexpr float ELBOW_CALIB_DEG       = 0.0f;
    constexpr float WRIST_PITCH_CALIB_DEG = 0.0f;
    constexpr float WRIST_ROLL_CALIB_DEG  = 0.0f;
    constexpr float TOOL_ROLL_CALIB_DEG   = 0.0f;

    // In Schritte umrechnen
    float stepsPerDeg = (float)(STEPS_PER_REVOLUTION * MICROSTEPPING) / 360.0f;
    long targetSteps[6];
    targetSteps[0] = (long)(BASE_CALIB_DEG        * stepsPerDeg);
    targetSteps[1] = (long)(SHOULDER_CALIB_DEG    * stepsPerDeg);
    targetSteps[2] = (long)(ELBOW_CALIB_DEG       * stepsPerDeg);
    targetSteps[3] = (long)(WRIST_PITCH_CALIB_DEG * stepsPerDeg);
    targetSteps[4] = (long)(WRIST_ROLL_CALIB_DEG  * stepsPerDeg);
    targetSteps[5] = (long)(TOOL_ROLL_CALIB_DEG   * stepsPerDeg);

    // Motoren aktivieren (Enable LOW)
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(ENABLE_PINS[i], LOW);
    }

    // MultiStepper-Koordination
    multiStepperGroup.moveTo(targetSteps);
    bool stillMoving = true;
    while (stillMoving) {
        multiStepperGroup.runSpeedToPosition();
        stillMoving = false;
        for (uint8_t i = 0; i < 6; i++) {
            if (motors[i].distanceToGo() != 0) {
                stillMoving = true;
                break;
            }
        }
    }

    // Sensor-Kalibrierung hier einfügen
    // z.B. InitSystem::calibrateAccelerometer();
    //     InitSystem::calibrateDistanceSensor();

    // Setze alle Gelenkwinkel auf 0 (Radiant)
    for (uint8_t i = 0; i < 6; i++) {
        currentJointAngles[i] = 0.0;
    }

    // Motoren deaktivieren (Enable HIGH)
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(ENABLE_PINS[i], HIGH);
    }
}
