#include "Init_System.h"
#include "Robo_Config_V1.h"
#include "Debug.h"
#include "Sensors.h"
#include <Wire.h>

// =====================
// Globale Objekte & Variablen
// =====================

// ADXL345: ID 12345 (Arbitrary)
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// VL53L0X
Adafruit_VL53L0X lox;

// Kalman‐Filter (1D) – Zustand und Kovarianz
float kalmanState      = 0.0f;            // initiale Höhe
float kalmanCovariance = 1.0f;            // initiale Varianz (z.B. 1 mm²)
const float kalmanQ    = 0.01f;           // Prozessrauschen (mm²)
const float kalmanR    = 10.0f;           // Messrauschen (mm²)

// Kalibrier‐Offsets
float accelOffsetX     = 0.0f;
float accelOffsetY     = 0.0f;
float accelOffsetZ     = 0.0f;
float distanceOffset   = 0.0f;

// =====================
// Hilfs‐Funktion: Grad in Radiant
// =====================
static inline float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

// =====================
// InitSystem::initializeSensorsAndFilters()
// =====================

void InitSystem::initializeSensorsAndFilters() {
    // --- 1) ADXL345 initialisieren ---
    if (!adxl.begin(ADXL345_I2C_ADDR)) {
        DEBUG_PRINTLN("Fehler: ADXL345 nicht gefunden!");
        while (1) { delay(1000); }
    }
    adxl.setRange(ADXL345_RANGE_4_G);
    delay(50);

    // --- 2) VL53L0X initialisieren ---
    if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire)) {
        DEBUG_PRINTLN("Fehler: VL53L0X nicht gefunden!");
        while (1) { delay(1000); }
    }
    // setze Messmodus auf Einzelmessung (wird bei Bedarf aufgerufen)
    delay(50);

    // --- 3) ADXL‐Offset‐Kalibrierung ---
    InitSystem::calibrateAccelerometer();
    DEBUG_PRINT("ADXL Offsets: X="); DEBUG_PRINT(accelOffsetX);
    DEBUG_PRINT("  Y="); DEBUG_PRINT(accelOffsetY);
    DEBUG_PRINT("  Z="); DEBUG_PRINTLN(accelOffsetZ);

    // --- 4) VL53L0X‐Offset‐Kalibrierung ---
    InitSystem::calibrateDistanceSensor();
    DEBUG_PRINT("Laser Offset (mm): "); DEBUG_PRINTLN(distanceOffset);

    // --- 5) Erster Höhenmess‐Schritt & init Kalman / EKF --
    float tiltRad = InitSystem::getTiltAngleRad();
    // Einzelmessung vom Laser abrufen
    uint16_t rawDist = lox.readRange();
    float height0 = ((float)rawDist - distanceOffset) * cosf(tiltRad);
    kalmanState      = height0;
    kalmanCovariance = 1.0f;
    sensorsEkfInit(height0 / 1000.0f, tiltRad);
    DEBUG_PRINT("Initial Height (mm): "); DEBUG_PRINTLN(height0);
}

// =====================
// InitSystem::calibrateAccelerometer()
// =====================

void InitSystem::calibrateAccelerometer() {
    const int N = 200;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    sensors_event_t event;

    for (int i = 0; i < N; i++) {
        adxl.getEvent(&event);
        sumX += event.acceleration.x;
        sumY += event.acceleration.y;
        sumZ += event.acceleration.z;
        delay(5);
    }
    accelOffsetX = sumX / (float)N;
    accelOffsetY = sumY / (float)N;
    accelOffsetZ = sumZ / (float)N;
}

// =====================
// InitSystem::calibrateDistanceSensor()
// =====================

void InitSystem::calibrateDistanceSensor() {
    const int N = 100;
    uint32_t sumDist = 0;
    for (int i = 0; i < N; i++) {
        uint16_t raw = lox.readRange();
        sumDist += raw;
        delay(5);
    }
    distanceOffset = (float)sumDist / (float)N;
}

// =====================
// InitSystem::getTiltAngleRad()
// =====================

float InitSystem::getTiltAngleRad() {
    sensors_event_t event;
    adxl.getEvent(&event);

    float ax = event.acceleration.x - accelOffsetX;
    float ay = event.acceleration.y - accelOffsetY;
    float az = event.acceleration.z - accelOffsetZ;

    float denom = sqrtf(ay * ay + az * az);
    if (denom < 1e-6f) denom = 1e-6f;
    float theta = atan2f(ax, denom);

    return theta;
}

// =====================
// InitSystem::getCorrectedLaserHeight()
// =====================

float InitSystem::getCorrectedLaserHeight(float tiltRad) {
    uint16_t rawDist = lox.readRange();
    float d = (float)rawDist - distanceOffset;
    if (d < 0.0f) d = 0.0f;
    float h = d * cosf(tiltRad);
    return h;
}

// =====================
// InitSystem::isLaserReady()
// =====================

bool InitSystem::isLaserReady() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    return measure.RangeStatus != 4; // 4 indicates out of range
}

// =====================
// InitSystem::kalmanUpdate()
// =====================

float InitSystem::kalmanUpdate(float z) {
    float x_pred = kalmanState;
    float P_pred = kalmanCovariance + kalmanQ;

    float K = P_pred / (P_pred + kalmanR);
    float x_new = x_pred + K * (z - x_pred);
    float P_new = (1.0f - K) * P_pred;

    kalmanState      = x_new;
    kalmanCovariance = P_new;

    return kalmanState;
}
