#ifndef INITSYSTEM_H
#define INITSYSTEM_H

#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_VL53L0X.h>

// =====================
// Externe Objekte (in InitSystem.cpp definiert)
// =====================

extern Adafruit_ADXL345_Unified adxl;  // Beschleunigungssensor‐Objekt
extern Adafruit_VL53L0X         lox;   // Laser‐Distanzsensor‐Objekt

// Kalman‐Filter‐Zustand & Kovarianz
extern float kalmanState;      // aktuelle Höhe (mm)
extern float kalmanCovariance; // Kovarianz (Varianz) der Schätzung

// Prozess‐ und Messrausch‐Parameter
extern const float kalmanQ;  // Prozess‐Rauschkovarianz
extern const float kalmanR;  // Messrauschkovarianz

// Kalibrier‐Offsets
extern float accelOffsetX;
extern float accelOffsetY;
extern float accelOffsetZ;
extern float distanceOffset;

// =====================
// Prototypen
// =====================

namespace InitSystem {
    /**
     * @brief  Initialisiert ADXL345 und VL53L0X, kalibriert sie (Offsets),
     *         und initialisiert den Kalman‐Filter mit dem ersten richtigen Höhen­wert.
     */
    void initializeSensorsAndFilters();

    /**
     * @brief  Liest ADXL aus (x, y, z), subtrahiert Offsets und
     *         berechnet den Neigungswinkel theta (in Radiant) relativ zur Vertikalen.
     *
     * @return Neigungswinkel theta in Radiant (z.B. Pitch).
     */
    float getTiltAngleRad();

    /**
     * @brief  Liest Rohdistanz vom VL53L0X (mm), subtrahiert distanceOffset
     *         und berechnet die korrigierte vertikale Höhe.
     *
     * @param tiltRad  Neigungswinkel (Rad) aus ADXL
     * @return         vertikale Höhe (mm) = (rohdist - distanceOffset)*cos(tiltRad)
     */
    float getCorrectedLaserHeight(float tiltRad);

    /**
     * @brief  Prüft, ob VL53L0X bereit ist (Sensor‐Check).
     */
    bool isLaserReady();

    /**
     * @brief  Führt einmalig eine ADXL‐Offsetkalibrierung durch (Dauer: ca. 100 ms).
     */
    void calibrateAccelerometer();

    /**
     * @brief  Führt einmalig eine VL53L0X‐Offsetkalibrierung durch (Dauer: ca. 50 ms).
     */
    void calibrateDistanceSensor();

    /**
     * @brief  Führt einen Kalman‐Filterschritt durch:
     *         1. Prädiktions­update (state = state, covariance += Q)
     *         2. Messupdate (K = P/(P+R), state += K*(z - state), P *= (1-K))
     *
     * @param z  Messwert (korrigierte vertikale Höhe, mm) aus Laser
     * @return   Gefilterte aktuelle Höhe (mm)
     */
    float kalmanUpdate(float z);
}

#endif // INITSYSTEM_H
