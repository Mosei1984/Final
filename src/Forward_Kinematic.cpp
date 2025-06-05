// ForwardKinematics.cpp

#include "Forward_Kinematic.h"
#include "DH_Parameter.h"
#include <math.h>

// =====================
// Hilfsfunktionen zur Matrizenrechnung
// =====================

/**
 * @brief Baut aus einem einzelnen Satz D-H-Parameter eine 4×4-Transformationsmatrix.
 *
 * @param a      Gliedabstand a_i (Meter)
 * @param alpha  Twist-Winkel α_i (Radiant)
 * @param d      Versatz d_i (Meter)
 * @param theta  Gelenkwinkel θ_i (Radiant)
 * @param M      Ausgabe: 4×4-Array, das die Homogene Transformationsmatrix speichert.
 */
static void dhToMatrix(double a, double alpha, double d, double theta, double M[4][4]) {
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    // Zeile 0
    M[0][0] = ct;
    M[0][1] = -st * ca;
    M[0][2] = st * sa;
    M[0][3] = a * ct;
    // Zeile 1
    M[1][0] = st;
    M[1][1] = ct * ca;
    M[1][2] = -ct * sa;
    M[1][3] = a * st;
    // Zeile 2
    M[2][0] = 0.0;
    M[2][1] = sa;
    M[2][2] = ca;
    M[2][3] = d;
    // Zeile 3
    M[3][0] = 0.0;
    M[3][1] = 0.0;
    M[3][2] = 0.0;
    M[3][3] = 1.0;
}

/**
 * @brief Multipliziert zwei 4×4-Matrizen: C = A × B.
 *
 * @param A   Linke Matrix (4×4)
 * @param B   Rechte Matrix (4×4)
 * @param C   Ausgabe: Produktmatrix (4×4)
 */
static void matMul4x4(const double A[4][4], const double B[4][4], double C[4][4]) {
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            C[i][j] = 0.0;
            for (uint8_t k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

/**
 * @brief Extrahiert Roll, Pitch, Yaw (ZYX-Konvention) aus einer 3×3-Rotationsmatrix.
 *
 * @param R       Rotationsmatrix: obere 3×3 eines 4×4-Arrays
 * @param roll    Ausgabe: Rotation um X-Achse (Radiant)
 * @param pitch   Ausgabe: Rotation um Y-Achse (Radiant)
 * @param yaw     Ausgabe: Rotation um Z-Achse (Radiant)
 */
static void rotationMatrixToEulerZYX(const double R[3][3],
                                     double* roll,
                                     double* pitch,
                                     double* yaw) {
    // ZYX-Extraktion:
    // pitch = atan2(-R[2][0], sqrt(R[0][0]^2 + R[1][0]^2))
    // yaw   = atan2(R[1][0], R[0][0])
    // roll  = atan2(R[2][1], R[2][2])
    double sy = -R[2][0];
    double cy = sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);
    *pitch = atan2(sy, cy);

    double sx = R[2][1];
    double cx = R[2][2];
    *roll = atan2(sx, cx);

    double sz = R[1][0];
    double cz = R[0][0];
    *yaw = atan2(sz, cz);
}

// =====================
// computeForwardKinematics
// =====================

void computeForwardKinematics(const double jointAngles[6],
                              double endPos[3],
                              double endOri[3]) {
    // 1) Initialisiere T_result als 4×4-Einheitsmatrix
    double T_result[4][4];
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            T_result[r][c] = (r == c) ? 1.0 : 0.0;
        }
    }

    // 2) Iteriere über alle 6 Gelenke und multipliziere sukzessive
    for (uint8_t i = 0; i < 6; i++) {
        // Lade die D-H-Parameter für Gelenk i
        double a     = robotDHParams[i].a;
        double alpha = robotDHParams[i].alpha;
        double d     = robotDHParams[i].d;
        double theta = jointAngles[i];  // Gelenkwinkel in Radiant

        // Baue Transformationsmatrix für Gelenk i
        double T_curr[4][4];
        dhToMatrix(a, alpha, d, theta, T_curr);

        // Multipliziere T_result × T_curr → neues T_result
        double T_next[4][4];
        matMul4x4(T_result, T_curr, T_next);

        // Kopiere T_next zurück in T_result
        for (uint8_t r = 0; r < 4; r++) {
            for (uint8_t c = 0; c < 4; c++) {
                T_result[r][c] = T_next[r][c];
            }
        }
    }

    // 3) Extrahiere Endeffektor-Position (X, Y, Z) aus T_result
    endPos[0] = T_result[0][3];
    endPos[1] = T_result[1][3];
    endPos[2] = T_result[2][3];

    // 4) Extrahiere Rotationsmatrix (obere 3×3 von T_result) und konvertiere in Euler-ZYX
    double R[3][3];
    for (uint8_t r = 0; r < 3; r++) {
        for (uint8_t c = 0; c < 3; c++) {
            R[r][c] = T_result[r][c];
        }
    }
    double roll, pitch, yaw;
    rotationMatrixToEulerZYX(R, &roll, &pitch, &yaw);
    endOri[0] = roll;
    endOri[1] = pitch;
    endOri[2] = yaw;
}
