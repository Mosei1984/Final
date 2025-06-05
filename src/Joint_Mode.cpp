// JointMode.cpp

#include "Joint_Mode.h"
#include "Robo_Config_V1.h" // fuer displayPtr
#include "Debug.h"


// =====================
// Interne State-Variablen
// =====================

// Index der aktuell ausgewählten Achse (0..5)
static int8_t selectedAxis = 0;

// Vorheriger Wert der Joystick-Navigation (–1, 0 oder +1)
// für rechten Joystick Y-Achse (ändert Auswahl nur bei Flanke)
static int8_t prevSelectNavY = 0;

// =====================
// jointModeInit()
// =====================
void jointModeInit() {
    // Reset Auswahl auf Achse 0
    selectedAxis = 0;
    prevSelectNavY = 0;

    // Stoppe alle Achsen
    for (uint8_t i = 0; i < 6; i++) {
        setStepperSpeed(i, 0.0f);
    }
}

// =====================
// jointModeStop()
// =====================
void jointModeStop() {
    // Stoppe alle Achsen beim Verlassen des Modus
    for (uint8_t i = 0; i < 6; i++) {
        setStepperSpeed(i, 0.0f);
    }
}

// =====================
// jointModeUpdate()
// =====================
void jointModeUpdate() {
    // 1) Eingänge aktualisieren
    updateRemoteInputs();
    const RemoteState* rs = getRemoteStatePointer();

    // 2) Achsenauswahl über rechten Joystick Y-Achse
    //    readNavDirectionY analog: über 0.5 → –1 (oben), unter –0.5 → +1 (unten)
    int8_t navY = 0;
    if (rs->rightY > 0.5f) {
        navY = +1;  // joystick nach unten → Auswahl nach unten
    } else if (rs->rightY < -0.5f) {
        navY = -1;  // joystick nach oben → Auswahl nach oben
    }

    if (navY != prevSelectNavY) {
        if (navY == -1) {
            selectedAxis = (selectedAxis - 1 + 6) % 6;
        } else if (navY == +1) {
            selectedAxis = (selectedAxis + 1) % 6;
        }

        DEBUG_PRINT("Select axis: ");
        DEBUG_PRINTLN(selectedAxis);

    }
    prevSelectNavY = navY;

    // 3) Geschwindigkeit der ausgewählten Achse steuern via linker Joystick Y-Achse
    //    raw-Wert in [-1.0 .. +1.0], Deadzone bereits in updateRemoteInputs() angewandt
    float speedNorm = rs->leftY;  // –1.0 (Joystick oben) .. +1.0 (Joystick unten)
    // Mappe auf Steps/s: volle Auslenkung = ±MAX_SPEED
    // (MAX_SPEED aus Robo_Config_V1.h definiert, z.B. 2000.0f)
    float targetSpeed = speedNorm * MAX_SPEED;

    // Stoppe alle anderen Achsen außer der selektierten
    for (uint8_t i = 0; i < 6; i++) {
        if (i != selectedAxis) {
            setStepperSpeed(i, 0.0f);
        }
    }
    // Setze Geschwindigkeit für selektierte Achse
    setStepperSpeed((uint8_t)selectedAxis, targetSpeed);

    // 4) Einfache Anzeige der aktuell selektierten Achse & Geschwindigkeit
    if (displayPtr) {
        displayPtr->clearBuffer();
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        displayPtr->setCursor(0, 16);
        displayPtr->print("Axis: ");
        displayPtr->print(selectedAxis);
        displayPtr->setCursor(0, 32);
        displayPtr->print("Speed: ");
        displayPtr->print(targetSpeed, 0);
        displayPtr->sendBuffer();
    }
}
