// Menu.cpp

#include "Menu.h"

// =============================================================================
// Interne State-Variablen
// =============================================================================

// Aktueller Hauptmenü-Index (0..MM_COUNT-1)
static int8_t currentMain = 0;

// Untermenü-Flags und Indizes
static bool inHomingSub   = false;
static int8_t currentHomingSub = 0;

static bool inKinematicSub   = false;
static int8_t currentKinematicSub = 0;

// Joystick-Vorzustände für Auf-/Ab-Bewegung (–1,0,+1)


static int8_t prevMainNavY = 0;
static int8_t prevSubNavY  = 0;
static bool   prevButton1  = false;
static bool   prevButton2  = false;



// Wahl abgeschlossen?
static bool choiceMade = false;
static MenuSelection finalSelection = { -1, -1 };

// =============================================================================
// Hilfsfunktion: Liest Joystick-Vertical und gibt –1, 0 oder +1 zurück.
//                Verwendet DEADZONE aus Robo_Config_V1.h.
// =============================================================================


static int8_t readNavDirectionY(float rawValue) {
    // Positive Werte entsprechen Joystick nach unten,
    // negative Werte Joystick nach oben. Wir geben
    // +1 fuer "nach unten" und -1 fuer "nach oben" zurueck.
    if (rawValue > 0.5f)  return +1; // nach unten
    if (rawValue < -0.5f) return -1; // nach oben
    return 0;
}



// =============================================================================
// menuInit()
// =============================================================================
void menuInit() {
    currentMain = 0;
    inHomingSub = false;
    currentHomingSub = 0;


    inKinematicSub = false;
    currentKinematicSub = 0;
    prevMainNavY = 0;
    prevSubNavY = 0;
    prevButton1  = false;
    prevButton2  = false;
    choiceMade = false;
    finalSelection = { -1, -1 };
}



// =============================================================================
// menuSelectionAvailable()
// =============================================================================
bool menuSelectionAvailable() {
    return choiceMade;
}

// =============================================================================
// menuGetSelection()
// =============================================================================
MenuSelection menuGetSelection() {
    return finalSelection;
}

// =============================================================================
// menuResetSelection()
// =============================================================================
void menuResetSelection() {
    choiceMade = false;
    finalSelection = { -1, -1 };
}

// =============================================================================
// Zeichnet das Hauptmenü auf displayPtr
// =============================================================================
static void drawMainMenu() {
    if (!displayPtr) return;
    displayPtr->firstPage();
    do {
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        // Menütitel
        displayPtr->setCursor(0, 12);
        displayPtr->print("== Hauptmenü ==");


        const char* items[MM_COUNT] = {
            "Homing",
            "Joint Mode",
            "Kinematic Mode",
            "G-Code Mode",
            "Teach/Play Mode"
        };

        for (int8_t i = 0; i < MM_COUNT; i++) {
            int y = 16 + i * 10;
            if (i == currentMain) {
                displayPtr->drawStr(0, y, ">");
                displayPtr->setCursor(8, y);
            } else {
                displayPtr->setCursor(8, y);
            }
            displayPtr->print(items[i]);
        }

    } while (displayPtr->nextPage());
}

// =============================================================================
// Zeichnet das Homing-Untermenü
// =============================================================================
static void drawHomingSubMenu() {
    if (!displayPtr) return;
    displayPtr->firstPage();
    do {
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        displayPtr->setCursor(0, 12);
        displayPtr->print("== Homing Menu ==");

        const char* items[HS_COUNT] = {
            "1. Einzelachse",
            "2. Alle Achsen",
            "3. Init Position",
            "4. Auto Homing",
            "5. Zurueck"
        };


        for (int8_t i = 0; i < HS_COUNT; i++) {
            int y = 16 + i * 10;
            if (i == currentHomingSub) {
                displayPtr->drawStr(0, y, ">");
                displayPtr->setCursor(8, y);
            } else {
                displayPtr->setCursor(8, y);

            }
            displayPtr->print(items[i]);
        }
    } while (displayPtr->nextPage());
}

// =============================================================================
// Zeichnet das Kinematic-Untermenü
// =============================================================================
static void drawKinematicSubMenu() {
    if (!displayPtr) return;
    displayPtr->firstPage();
    do {
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        displayPtr->setCursor(0, 12);
        displayPtr->print("== Kinematic Menu ==");

        const char* items[KS_COUNT] = {
            "1. Sensoren Ein/Aus",
            "2. Position Setzen",
            "3. Fahre Position",
            "4. Zurueck"
        };


        for (int8_t i = 0; i < KS_COUNT; i++) {
            int y = 16 + i * 10;
            if (i == currentKinematicSub) {
                displayPtr->drawStr(0, y, ">");
                displayPtr->setCursor(8, y);
            } else {
                displayPtr->setCursor(8, y);

            }
            displayPtr->print(items[i]);
        }
    } while (displayPtr->nextPage());
}

// =============================================================================
// menuUpdate()
// =============================================================================
void menuUpdate() {
    // Wenn bereits gewählt, nichts mehr tun
    if (choiceMade) return;

    // 1) Eingänge aktualisieren
    updateRemoteInputs();

    const RemoteState* rs = getRemoteStatePointer();
    bool pressed1 = rs->button1 && !prevButton1;
    bool pressed2 = rs->button2 && !prevButton2;
    prevButton1 = rs->button1;
    prevButton2 = rs->button2;


    // 2) Navigation im Menü
    // Hauptmenü vs. Untermenüs:
    if (!inHomingSub && !inKinematicSub) {
        // Hauptmenü-Navigation
        int8_t dirY = readNavDirectionY(rs->leftY);
        if (dirY != prevMainNavY) {
            // Auf-/Ab-Bewegung erkannt
            if (dirY == -1) {
                // nach oben
                currentMain = (currentMain - 1 + MM_COUNT) % MM_COUNT;
            } else if (dirY == +1) {
                // nach unten
                currentMain = (currentMain + 1) % MM_COUNT;
            }
        }
        prevMainNavY = dirY;

        // Auswahl per Button1 (Flanke)
        if (pressed1) {

            switch (currentMain) {
                case MM_HOMING:
                    inHomingSub = true;
                    currentHomingSub = 0;
                    break;
                case MM_KINEMATIC:
                    inKinematicSub = true;
                    currentKinematicSub = 0;
                    break;
                default:
                    // Für andere Modi direkt als Wahl beenden (subIndex = -1)


                    finalSelection.mainIndex = currentMain;
                    finalSelection.subIndex = -1;
                    choiceMade = true;
                    Serial.print("Menu select main=");
                    Serial.println(currentMain);
                    break;
            }


        }

        // Zeichne Hauptmenü
        drawMainMenu();
        return;
    }

    // 3) Homing-Untermenü
    if (inHomingSub) {
        int8_t dirY = readNavDirectionY(rs->leftY);
        if (dirY != prevSubNavY) {
            if (dirY == -1) {
                currentHomingSub = (currentHomingSub - 1 + HS_COUNT) % HS_COUNT;
            } else if (dirY == +1) {
                currentHomingSub = (currentHomingSub + 1) % HS_COUNT;
            }
        }
        prevSubNavY = dirY;




        // Auswahl mit Button1 oder sofort zurück mit Button2
        if (pressed1) {
            if (currentHomingSub == HS_HOMING_BACK) {
                // Zurück ins Hauptmenü
                inHomingSub = false;
                currentHomingSub = 0;
            } else {
                // Auswahl getroffen -> mainIndex=MM_HOMING, subIndex=currentHomingSub

                finalSelection.mainIndex = MM_HOMING;
                finalSelection.subIndex = currentHomingSub;
                choiceMade = true;
                Serial.print("Homing select sub=");
                Serial.println(currentHomingSub);
            }

        }
        if (pressed2) {
            inHomingSub = false;
            currentHomingSub = 0;
            Serial.println("Homing menu exit");
        }

        }

        drawHomingSubMenu();
        return;
    }

    // 4) Kinematic-Untermenü
    if (inKinematicSub) {
        int8_t dirY = readNavDirectionY(rs->leftY);
        if (dirY != prevSubNavY) {
            if (dirY == -1) {
                currentKinematicSub = (currentKinematicSub - 1 + KS_COUNT) % KS_COUNT;
            } else if (dirY == +1) {
                currentKinematicSub = (currentKinematicSub + 1) % KS_COUNT;
            }
        }
        prevSubNavY = dirY;



        // Auswahl mit Button1 oder Zurück mit Button2
        if (pressed1) {
            if (currentKinematicSub == KS_KIN_BACK) {
                // Zurück ins Hauptmenü
                inKinematicSub = false;
                currentKinematicSub = 0;
            } else {
                // Auswahl getroffen -> mainIndex=MM_KINEMATIC, subIndex=currentKinematicSub
                finalSelection.mainIndex = MM_KINEMATIC;
                finalSelection.subIndex = currentKinematicSub;
                choiceMade = true;
                Serial.print("Kinematic select sub=");
                Serial.println(currentKinematicSub);
            }
        }
        if (pressed2) {
            inKinematicSub = false;
            currentKinematicSub = 0;
            Serial.println("Kinematic menu exit");
        }


        }



        drawKinematicSubMenu();
        return;
    }
}
