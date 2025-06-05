// KinematicMode.cpp


#include "Kinematic_Mode.h"
#include "Robo_Config_V1.h" // displayPtr


// =============================================================================
// Interne State-Variablen
// =============================================================================

// Untermenü-Index (0..KS_COUNT-1, aus Menu.h)
static int8_t currentSub = 0;
// Flags, ob im Set-Position- oder Go-To-Position-Dialog
static bool inSetPosition = false;
static bool inGoToPosition = false;

// Navigationsvorher (damit nur Flankenbewegungen zählen)

static int8_t prevNavY = 0;
static bool   prevButton1 = false;
static bool   prevButton2 = false;


// Aktuelle Zielkoordinaten in Metern (X, Y, Z)
static double targetPos[3] = {0.0, 0.0, 0.0};

// Sensors Enabled Flag
static bool sensorsEnabled = false;

// =============================================================================
// Konstanten für Schrittumrechnung
// =============================================================================
// Schritte pro Radiant: (STEPS_PER_REVOLUTION * MICROSTEPPING) / (2π)
static const double STEPS_PER_RAD = ((double)BASE_STEPS) / (2.0 * M_PI);

// =============================================================================
// kinematicModeInit()
// =============================================================================
void kinematicModeInit() {

    currentSub       = 0;
    prevNavY         = 0;
    prevButton1      = false;
    prevButton2      = false;
    inSetPosition    = false;

    inGoToPosition   = false;
    sensorsEnabled   = false;

    // Setze Standardziel: aktuelle Endeffektor-Position (aus FK)
    double currAngles[6];
    for (uint8_t i = 0; i < 6; i++) {
        currAngles[i] = currentJointAngles[i];
    }
    double currPos[3], currOri[3];
    computeForwardKinematics(currAngles, currPos, currOri);

    targetPos[0] = currPos[0];
    targetPos[1] = currPos[1];
    targetPos[2] = currPos[2];

    // Stoppe alle Achsen
    for (uint8_t i = 0; i < 6; i++) {
        setStepperSpeed(i, 0.0f);
    }
}

// =============================================================================
// kinematicModeStop()
// =============================================================================
void kinematicModeStop() {
    // Stoppe alle Achsen
    for (uint8_t i = 0; i < 6; i++) {
        setStepperSpeed(i, 0.0f);
    }
    // Reset Dialog-Flags
    inSetPosition  = false;
    inGoToPosition = false;
}

// =============================================================================
// areSensorsEnabled()
// =============================================================================
bool areSensorsEnabled() {
    return sensorsEnabled;
}

// =============================================================================
// kinematicModeUpdate()
// =============================================================================

void kinematicModeUpdate() {
    // 1) Eingänge aktualisieren
    updateRemoteInputs();
    const RemoteState* rs = getRemoteStatePointer();
    bool pressed1 = rs->button1 && !prevButton1;
    bool pressed2 = rs->button2 && !prevButton2;
    prevButton1 = rs->button1;
    prevButton2 = rs->button2;

    // 2) Wenn man gerade Ziel eingibt (Set Position)
    if (inSetPosition) {
        static double prevPos[3] = {0.0, 0.0, 0.0};
        // Anpassung:
        //   - Linker Joystick X  steuert ΔX (–0.01 … +0.01 m)
        //   - Linker Joystick Y  steuert ΔY (–0.01 … +0.01 m)
        //   - Rechter Joystick X steuert ΔZ (–0.01 … +0.01 m)
        const double stepIncrement = 0.01; // 1 cm pro vollen Ausschlag
        targetPos[0] += rs->leftX * stepIncrement;
        targetPos[1] += rs->leftY * stepIncrement;
        targetPos[2] += rs->rightZ * stepIncrement;


        // Begrenze Zielkoordinaten z.B. [–0.5m..+0.5m]
        for (int i = 0; i < 3; i++) {
            if (targetPos[i] < -0.5) targetPos[i] = -0.5;
            if (targetPos[i] > +0.5) targetPos[i] = +0.5;
        }


        // Gebe neue Position nur aus, wenn sie sich spürbar geändert hat
        if (fabs(prevPos[0] - targetPos[0]) > 0.005 ||
            fabs(prevPos[1] - targetPos[1]) > 0.005 ||
            fabs(prevPos[2] - targetPos[2]) > 0.005) {
            Serial.print("Target X:"); Serial.print(targetPos[0]);
            Serial.print(" Y:"); Serial.print(targetPos[1]);
            Serial.print(" Z:"); Serial.println(targetPos[2]);
            if (displayPtr) {
                displayPtr->clearBuffer();
                displayPtr->setFont(u8g2_font_ncenB08_tr);
                displayPtr->setCursor(0, 16);
                displayPtr->print("X:");
                displayPtr->print(targetPos[0], 2);
                displayPtr->setCursor(0, 32);
                displayPtr->print("Y:");
                displayPtr->print(targetPos[1], 2);
                displayPtr->setCursor(0, 48);
                displayPtr->print("Z:");
                displayPtr->print(targetPos[2], 2);
                displayPtr->sendBuffer();
            }
            prevPos[0] = targetPos[0];
            prevPos[1] = targetPos[1];
            prevPos[2] = targetPos[2];
        }

        // Bestätigen mit Button1: wechsle zu Go-To-Position
        if (pressed1) {
            inSetPosition  = false;
            inGoToPosition = true;
            Serial.println("Set complete -> GoTo");
        }
        // Abbrechen mit Button2: zurück ins Untermenü
        if (pressed2) {
            inSetPosition = false;
            Serial.println("Set cancelled");
        }

        return;
    }

    // 3) Wenn man gerade Go-To-Position aktiv hat
    if (inGoToPosition) {
        // Lokales Array für Ziel-Orientierung (hier 0,0,0)
        double zeroOri[3] = { 0.0, 0.0, 0.0 };

        // Versuche IK zu berechnen
        double initialGuess[6];
        for (uint8_t i = 0; i < 6; i++) {
            initialGuess[i] = currentJointAngles[i];
        }
        double solAngles[6];
        IKSettings settings;




        bool ok = computeInverseKinematics(targetPos, zeroOri,
                                           initialGuess, solAngles, settings);
        if (ok) {
            Serial.println("IK solution found");
            // Wandle Gelenkwinkel in Schritte: θ [rad] → steps
            long stepTargets[6];
            for (uint8_t i = 0; i < 6; i++) {
                stepTargets[i] = (long)round(solAngles[i] * STEPS_PER_RAD);
            }
            moveToPositionsAsync(stepTargets);
        } else {
            Serial.println("IK failed");
        }
        // Zurück ins Untermenü
        inGoToPosition = false;
        Serial.println("Move command sent");
        return;
    }

    // 4) Normaler Untermenü-Navigationsmodus
    // Rechter Joystick Y steuert Untermenü-Auswahl
    int8_t navY = 0;
    if (rs->rightY > 0.5f) {
        navY = +1;  // nach unten
    } else if (rs->rightY < -0.5f) {
        navY = -1;  // nach oben

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }
        Serial.print("Kinematic menu sub: ");
        Serial.println(currentSub);

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }
        Serial.print("Kinematic menu sub: ");
        Serial.println(currentSub);

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }
        Serial.print("Kinematic menu sub: ");
        Serial.println(currentSub);

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }
        Serial.print("Kinematic menu sub: ");
        Serial.println(currentSub);

    }
    if (navY != prevNavY) {
        if (navY == -1) {
            currentSub = (currentSub - 1 + KS_COUNT) % KS_COUNT;
        } else if (navY == +1) {
            currentSub = (currentSub + 1) % KS_COUNT;
        }
        Serial.print("Kinematic menu sub: ");
        Serial.println(currentSub);
    }
    prevNavY = navY;



    // Auswahl mit Button1
    if (pressed1) {
        switch (currentSub) {
            case KS_SENSORS_TOGGLE:
                sensorsEnabled = !sensorsEnabled;
                Serial.print("Sensors ");
                Serial.println(sensorsEnabled ? "on" : "off");
                if (displayPtr) {
                    displayPtr->clearBuffer();
                    displayPtr->setFont(u8g2_font_ncenB08_tr);
                    displayPtr->setCursor(0, 20);
                    displayPtr->print("Sensors:");
                    displayPtr->print(sensorsEnabled ? "ON" : "OFF");
                    displayPtr->sendBuffer();
                }
                break;
            case KS_SET_POSITION:
                inSetPosition = true;
                Serial.println("Set target position");
                break;
            case KS_GOTO_POSITION:
                inGoToPosition = true;
                Serial.println("Execute IK move");
                break;


            case KS_KIN_BACK:
                // Beende Kinematic Mode
                kinematicModeStop();
                break;
            default:
                break;
        }
    }
}
