#include "DH_Parameter.h"
#include <cmath>

// =====================
// Definition der D-H-Parameter für 6 Gelenke
// =====================
// Beispielwerte: Bitte an die tatsächlichen Robotermaße anpassen.
DHParams robotDHParams[6] = {
    //    a       alpha       d         theta
    { 0.0,     M_PI_2,    0.1,      0.0 },   // Gelenk 1
    { 0.0,     0.0,       0.0,      0.0 },   // Gelenk 2
    { 0.0,     0.0,       0.0,      0.0 },   // Gelenk 3
    { 0.0,     M_PI_2,    0.0,      0.0 },   // Gelenk 4
    { 0.0,    -M_PI_2,    0.0,      0.0 },   // Gelenk 5
    { 0.0,     0.0,       0.0,      0.0 }    // Gelenk 6
};

// =====================
// Definition der Gelenk-Limits (in Radiant)
// =====================
// Beispielwerte: Untere/Obere Winkelgrenzen pro Gelenk anpassen
JointLimits jointLimits[6] = {
    { -M_PI,  M_PI   },   // Gelenk 1
    { -M_PI/2, M_PI/2 },   // Gelenk 2
    { -M_PI/2, M_PI/2 },   // Gelenk 3
    { -M_PI,  M_PI   },   // Gelenk 4
    { -M_PI,  M_PI   },   // Gelenk 5
    { -M_PI,  M_PI   }    // Gelenk 6
};

// =====================
// Aktuelle Gelenkwinkel (in Radiant), initial alle 0.0
// =====================
double currentJointAngles[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

// =====================
// Tool-Länge (Offset vom letzten Gelenk zum Ende), in Metern
// =====================
// Beispiel: 1 cm
const double toolLength = 0.01;
