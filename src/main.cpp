// main.cpp

#include <Arduino.h>

#include <U8g2lib.h>
#include <IntervalTimer.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>


#include "Robo_Config_V1.h"
#include "Remote.h"
#include "Menu.h"
#include "Joint_Mode.h"

#include "Kinematic_Mode.h"
#include "Homing.h"
#include "Stepper_Config.h"
#include "Init_System.h"
#include "SystemStatus.h"


// -----------------------------------------------------------------------------
// Globale Objekte
// -----------------------------------------------------------------------------

// OLED-Display (SSD1306 I²C, Full-Buffer)
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// Timer für die STEP-ISR (2 kHz)
IntervalTimer stepTimer;
static bool stepTimerRunning = false;

static void startStepTimer() {
  if (!stepTimerRunning) {
    stepTimer.begin(stepperISR, 500);  // 2 kHz
    stepTimerRunning = true;
  }
}

static void stopStepTimer() {
  if (stepTimerRunning) {
    stepTimer.end();
    stepTimerRunning = false;
  }
}


// NeoPixel-Status-LEDs
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Flag, um nach einem Modus zurück ins Menü zu springen
static bool returnToMenu = false;


// Aktueller System-Status (für LEDs)
static SystemStatus currentStatus = STATUS_IDLE;


// -----------------------------------------------------------------------------
// LED-Status-Funktion
// -----------------------------------------------------------------------------
// Setzt alle NeoPixels auf eine Farbe passend zum SystemStatus.
// -----------------------------------------------------------------------------

void setStatusLED(SystemStatus s) {

  uint32_t color;
  switch (s) {
    case STATUS_MENU:
      color = pixels.Color(0, 0, 150);   // Blau
      break;
    case STATUS_HOMING:
      color = pixels.Color(200, 150, 0); // Gelb
      break;
    case STATUS_JOINT:
      color = pixels.Color(0, 200, 0);   // Grün
      break;

    case STATUS_KINEMATIC:
      if (areSensorsEnabled()) {
        color = pixels.Color(150, 0, 150); // Magenta wenn Sensoren aktiv
      } else {
        color = pixels.Color(0, 150, 150); // Cyan
      }
      break;

    case STATUS_IDLE:
      color = pixels.Color(0, 50, 0);    // Dunkelgrün (ruhig)
      break;
    case STATUS_ERROR:
      color = pixels.Color(200, 0, 0);   // Rot
      break;
    default:
      color = pixels.Color(50, 50, 50);  // Grau (Fallback)
      break;
  }

  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// Kleine Hilfsfunktion, um eine zweizeilige Meldung auf dem Display anzuzeigen
static void showMessage(const char* line1, const char* line2) {
  if (!displayPtr) return;
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_ncenB08_tr);
  displayPtr->setCursor(0, 20);
  displayPtr->print(line1);
  displayPtr->setCursor(0, 40);
  displayPtr->print(line2);
  displayPtr->sendBuffer();
}

// -----------------------------------------------------------------------------
// Wrapper für Homing-Untermenüaktionen
// -----------------------------------------------------------------------------

static void handleHomingSub(int8_t subIndex) {
  currentStatus = STATUS_HOMING;
  setStatusLED(currentStatus);
  stopStepTimer();
  showMessage("Homing...", "");
  bool success = true;

  switch (subIndex) {
    case HS_SINGLE_AXIS:
      // Homing jeder Achse einzeln (0..5)
      for (uint8_t i = 0; i < 6; i++) {
        if (!homeAxis(i)) {
          showMessage("Homing", "timeout");
          success = false;
          break;
        }
      }
      break;

    case HS_ALL_AXES:
      // Homing aller Achsen (0..3, optional 4/5) und Kalibrierpose
      if (!homeAllAxes()) {
        showMessage("Homing", "timeout");
        success = false;
        break;
      }
      break;

    case HS_MOVE_INIT_POS:
      // Homing + Initialposition anfahren
      if (!homeAllAxes()) {
        showMessage("Homing", "timeout");
        success = false;
        break;
      }
      {
        long initSteps[6] = {0, 0, 0, 0, 0, 0};
        moveToPositionsAsync(initSteps);
        while (isMultiMoveActive()) {

          // ISR übernimmt das Taktieren
        }
      }
      break;

    case HS_AUTO_HOMING:

      // Auto-Homing und Kalibrierpose (moveToCalibrationPose integriert)
      if (!homeAllAxes()) {
        showMessage("Homing", "timeout");
        success = false;
        break;
      }
      break;

    default:
      break;
  }

  if (success) {
    showMessage("Homing", "done");
  }
  startStepTimer();
  currentStatus = STATUS_IDLE;
  setStatusLED(currentStatus);
}


// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(200);

  // I2C-Bus starten
  Wire.begin();

  // --- 1) Display initialisieren ---
  oled.setI2CAddress(DISPLAY_I2C_ADDR << 1);
  oled.begin();
  displayPtr = &oled;
  DEBUG_PRINTLN("OLED ready");


  // --- 2) NeoPixel initialisieren ---
  pixels.begin();
  pixels.setBrightness(PIXEL_BRIGHTNESS);
  currentStatus = STATUS_IDLE;
  setStatusLED(currentStatus);

  // --- 3) RemoteControl (Joystick & Buttons) ---
  remoteInit();
  calibrateJoysticks(100);


  // --- 4) Sensoren initialisieren ---
  InitSystem::initializeSensorsAndFilters();

  // --- 5) Stepper initialisieren ---
  configureSteppers();
  setupMultiStepper();

  // --- 6) STEP-Timer (2 kHz) ---
  // Hoehere Frequenz erlaubt schnellere Schrittgeschwindigkeiten
  startStepTimer();

  // --- 7) Menü initialisieren ---

  currentStatus = STATUS_MENU;
  setStatusLED(currentStatus);
  menuInit();
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop() {

  // 1) Menü-Update (updateRemoteInputs wird in menuUpdate aufgerufen)
  menuUpdate();





  // 2) Wenn eine Menü-Auswahl vorliegt, handle sie
  if (menuSelectionAvailable()) {
    MenuSelection sel = menuGetSelection();
    menuResetSelection();

    // Homing-Modus
    if (sel.mainIndex == MM_HOMING) {
      handleHomingSub(sel.subIndex);
      returnToMenu = true;
    }
    // JointMode
    else if (sel.mainIndex == MM_JOINT) {
      currentStatus = STATUS_JOINT;
      setStatusLED(currentStatus);


      showMessage("Joint Mode", "Button2=Back");
      jointModeInit();
      returnToMenu = false;
      while (!returnToMenu) {
        jointModeUpdate();
        updateAllSteppers();
        if (getRemoteStatePointer()->button2) {
          returnToMenu = true;
        }
      }
      showMessage("Joint Mode", "done");
      jointModeStop();

      currentStatus = STATUS_IDLE;
      setStatusLED(currentStatus);

    }
    // KinematicMode
    else if (sel.mainIndex == MM_KINEMATIC) {
      currentStatus = STATUS_KINEMATIC;
      setStatusLED(currentStatus);


      showMessage("Kinematic", "Button2=Back");
      kinematicModeInit();
      returnToMenu = false;
      while (!returnToMenu) {
        kinematicModeUpdate();
        updateAllSteppers();
        if (getRemoteStatePointer()->button2) {
          returnToMenu = true;
        }
      }
      showMessage("Kinematic", "done");
      kinematicModeStop();

      currentStatus = STATUS_IDLE;
      setStatusLED(currentStatus);

    }
    // G-Code Mode (Platzhalter)
    else if (sel.mainIndex == MM_GCODE) {
      // TODO: G-Code-Modul integrieren
      returnToMenu = true;
    }
    // Teach/Play Mode (Platzhalter)
    else if (sel.mainIndex == MM_TEACH_PLAY) {
      // TODO: Teach/Play-Modul integrieren
      returnToMenu = true;
    }

    // 3) Nach Abschluss eines Modus zurück ins Hauptmenü
    if (returnToMenu) {
      currentStatus = STATUS_MENU;
      setStatusLED(currentStatus);
      menuInit();
    }
  }

  // 4) Während im Menü nichts gewählt wurde, weiter STEP-Impulse generieren
  updateAllSteppers();
  delay(10);
}
