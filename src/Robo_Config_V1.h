#ifndef ROBO_CONFIG_V1_H
#define ROBO_CONFIG_V1_H

#include <Arduino.h>
#include <U8g2lib.h>           // OLED-Display
#include <AccelStepper.h>      // AccelStepper- und MultiStepper-Klassen
#include <Adafruit_NeoPixel.h> // NeoPixel-LED-Streifen
#include <Servo.h>             // Servo-Objekt fürs Greifen
#include "Dh_Parameter.h"      // D-H-Parameter und Joint-Limits
#include <MultiStepper.h>

//
// =====================
// Makros & Konstanten
// =====================

// --- Microstepping & Schritte ---
#define MICROSTEPPING        32
#define STEPS_PER_REVOLUTION 200
constexpr uint32_t BASE_STEPS = STEPS_PER_REVOLUTION * MICROSTEPPING;

// --- Default-Werte für Motion (können in RoboConfig_v1.cpp überschrieben werden) ---
constexpr float DEFAULT_MAX_SPEED    = 1000.0f;
constexpr float DEFAULT_ACCELERATION = 2000.0f;

// --- I2C-Adressen (7-bit, ohne R/W-Bit) ---
constexpr uint8_t DISPLAY_I2C_ADDR  = 0x3C; // SSD1306 OLED
constexpr uint8_t ADXL345_I2C_ADDR  = 0x53; // Beschleunigungssensor
constexpr uint8_t VL53L0X_I2C_ADDR  = 0x29; // Laser-Distanzsensor

// --- PIN-Zuweisungen für Stepper (jeweils DRIVER-Modus) ---
// Jeder Achse hat einen eindeutigen STEP-Pin.
constexpr uint8_t STEP_PINS[6]   = {  2,  5,  8,  18, 11, 14 };
constexpr uint8_t DIR_PINS[6]    = {  3,  6,  9,  12,  15, 20 };
constexpr uint8_t ENABLE_PINS[6] = { 4, 7, 10, 13, 16, 21 };

// --- Endstop-Pins für Homing ---
constexpr uint8_t ENDSTOP_PINS[6] = { 22, 23, 24, 25, 29, 28 };

// --- Joystick-Pins (Analog-Eingänge) ---
constexpr uint8_t JOY_LX_PIN    = 41;   // Linker Joystick X-Achse
constexpr uint8_t JOY_LY_PIN    = 40;   // Linker Joystick Y-Achse
constexpr uint8_t JOY_RZ_PIN    = 39;   // Rechter Joystick Z-Achse (Vertikal)
constexpr uint8_t JOY_RYAW_PIN  = 38;   // Rechter Joystick Yaw-/Dreh-Achse

// --- Button-Pins (Digital-Eingänge) ---
// (in Remote.h sind bereits `extern const uint8_t BUTTON1_PIN;` und `BUTTON2_PIN;` deklariert,
//  deshalb hier ohne Makro, sondern als `constexpr` bzw. später in Robo_Config_V1.cpp definiert)
constexpr uint8_t BUTTON1_PIN   = 26;   // Button 1 (z.B. Bestätigen)
constexpr uint8_t BUTTON2_PIN   = 34;   // Button 2 (z.B. Zurück)

// --- Joystick-Kalibrierung (werden in InitSystem oder main neu gesetzt) ---
extern int16_t joyLXCenter;
extern int16_t joyLYCenter;
extern int16_t joyRZCenter;
extern int16_t joyRYawCenter;

// --- Joystick-Smoothing & Deadzone ---
#define JOYSTICK_SMOOTHING 8
#define DEADZONE           20

// --- Servo-Greifer & Potentiometer ---
constexpr uint8_t SERVO_PIN        = 33;   // PWM-Pin für den Greifer-Servo
constexpr uint8_t SERVO_POT_PIN    = 27;   // Analog-Pin zur Poti-Abfrage (Servo-Position)
constexpr uint8_t SERVO_DEFAULT_POS = 165;  // Startwinkel für den Greifer
constexpr uint8_t SERVO_MIN_POS     = 40;   // Min. Winkel (Greifer zu)
constexpr uint8_t SERVO_MAX_POS     = 180;  // Max. Winkel (Greifer auf)

// --- Geschwindigkeits-Limits (Systemweit) ---
constexpr float MAX_SPEED           = 2000.0f;  // Max. Geschwindigkeit für Bewegungen
constexpr float MIN_PRACTICAL_SPEED =   20.0f;  // Min. praktikable Geschwindigkeit
constexpr float MAX_ACCEL           = 1000.0f;  // Max. Beschleunigung

// --- Homing-Geschwindigkeiten & Backoff ---
constexpr float HOMING_FAST_SPEED      = 100.0f;   // Schnelle Annäherung beim Homing
constexpr float HOMING_SLOW_SPEED      =  50.0f;   // Langsame Annäherung beim Feinschritt
constexpr long  HOMING_BACKOFF_STEPS   = 200;      // Schritte zum Zurücksetzen nach Endstop-Hit

// --- Homing-Konfiguration (aktiviert/deaktiviert pro Achse; Definition in .cpp) ---
extern const bool HOMING_ENABLED[6];
extern bool MOTOR_DIRECTION[6];   // true = positive Richtung
extern bool HOMING_DIRECTION[6];  // true = positive Richtung beim Homing

// --- Timeout für Homing (in ms; definiert in Homing-Modul) ---
// #define MAX_HOMING_TIME 1200000  // 20 Minuten (bereits in Homing.h definiert)

// --- LED-Streifen (NeoPixel) ---
constexpr uint8_t NUM_PIXELS       = 5;   // Anzahl der LEDs im Streifen
constexpr uint8_t NEOPIXEL_PIN      = 6;   // Daten-Pin für NeoPixel
constexpr uint8_t PIXEL_BRIGHTNESS  = 200;  // Helligkeit der LEDs (0–255)

// --- Display-Pointer (wird in InitSystem.cpp initialisiert) ---
extern U8G2* displayPtr;

// --- Globale Zeitbasis (z. B. für Joystick-Abfrage, Debouncing usw.) ---
extern unsigned long lastJoystickCheck;

//
// =====================
// Globale Objekte
// =====================

// --- AccelStepper-Objekte für jede der 6 Achsen (Definition in RoboConfig_v1.cpp) ---
extern AccelStepper motors[6];

// --- MultiStepper-Gruppe für koordinierte Bewegungen (Definition in RoboConfig_v1.cpp) ---
extern MultiStepper multiStepperGroup;

#endif // ROBO_CONFIG_V1_H
