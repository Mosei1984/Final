#ifndef DH_PARAMETER_H
#define DH_PARAMETER_H

#include <Arduino.h>

// =====================
// Strukturdefinitionen
// =====================

// D-H-Parameter-Struktur: a, alpha, d, theta
struct DHParams {
    double a;       // Gelenkabstand entlang der X-Achse
    double alpha;   // Twist-Winkel zwischen Z-Achsen (in Radiant)
    double d;       // Versatz entlang der Z-Achse
    double theta;   // Gelenkwinkel um die Z-Achse (in Radiant)
};

// Joint-Limits-Struktur: min und max Winkel (in Radiant)
struct JointLimits {
    double minAngle;
    double maxAngle;
};

// =====================
// Externe Variablen
// =====================

// D-H-Parameter für die 6 Gelenke
// (Initialisierung in Dh_Parameter.cpp)
extern DHParams robotDHParams[6];

// Winkelgrenzen für jedes Gelenk (in Radiant)
// (Initialisierung in Dh_Parameter.cpp)
extern JointLimits jointLimits[6];

// Aktuelle Gelenkwinkel (in Radiant), initial alle auf 0.0 setzen
// (Initialisierung in Dh_Parameter.cpp)
extern double currentJointAngles[6];

// Tool-Länge (Offset vom letzten Gelenk zum Werkzeugende), in Metern
// (z. B. 0.01 m), Definition in Dh_Parameter.cpp
extern const double toolLength;

#endif // DH_PARAMETER_H
