// SensorCorrections.cpp

#include "Sensors.h"
#include "Init_System.h"
#include <math.h>

// =====================
// getSensorBasedZAndTilt
// =====================
void getSensorBasedZAndTilt(double outTiltRollPitch[2], double& outCorrectedZ) {
    // 1) Neigungswinkel (Pitch) und Roll aus ADXL berechnen:
    float tiltRad = InitSystem::getTiltAngleRad();
    // Standard ADXL liefert nur eine Achse Neigung (Pitch), wir setzen Roll=0
    // Wenn du Roll und Pitch getrennt messen möchtest, benötigst du einen 6DOF IMU.
    double rollRad  = 0.0;
    double pitchRad = (double) tiltRad;

    // 2) Korrigierte Höhe aus Laser + Tilt:
    float h_mm = InitSystem::getCorrectedLaserHeight(tiltRad);
    // Konvertiere Millimeter in Meter
    double h_m = (double) h_mm / 1000.0;

    // 3) Speichern
    outTiltRollPitch[0] = rollRad;
    outTiltRollPitch[1] = pitchRad;
    outCorrectedZ       = h_m;
}

// =====================
// applySensorCorrections
// =====================
void applySensorCorrections(double& zKorrigiert, double& rollKorrigiert, double& pitchKorrigiert) {
    // 1) Hole Sensor-basiertes Z & Neigung
    double tiltRP[2], zSensor;
    getSensorBasedZAndTilt(tiltRP, zSensor);

    // 2) Korrigiere kinematisch errechnete Z-Position:
    //    - Wenn du in deiner Applikation bereits eine kinematisch berechnete Z_habe,
    //      kannst du hier Mischfaktor (z.B. α) verwenden: zCorr = α * zKine + (1 - α) * zSensor.
    //    - Für Einfachheit setzen wir alpha=0.0 (voll auf Sensor verlassen), oder passe an.
    const double alpha = 0.0;
    zKorrigiert = alpha * zKorrigiert + (1.0 - alpha) * zSensor;

    // 3) Korrigiere Roll & Pitch:
    //    - Roll vom ADXL (hier standardmäßig 0) und Pitch (tiltRP[1]) setzen
    //    - Wenn kinematisch berechnete Roll/Pitch vorliegen, mische auch hier
    const double beta = 0.5; // 0.5 = halber Weg: 50% Kinematik, 50% Sensor
    rollKorrigiert  = beta * rollKorrigiert  + (1.0 - beta) * tiltRP[0];
    pitchKorrigiert = beta * pitchKorrigiert + (1.0 - beta) * tiltRP[1];
}
