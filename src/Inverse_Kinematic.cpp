// InverseKinematics.cpp

#include "Inverse_Kinematic.h"
#include <math.h>

using namespace Eigen;

static void computeJacobianNumeric(const double jointAngles[6], Matrix<double,6,6>& J) {
    const double delta = 1e-6; // kleiner Winkelperturbationswert (Rad)
    double baseJoint[6];
    for (int i = 0; i < 6; i++) baseJoint[i] = jointAngles[i];

    // Berechne Basis-Forward-Kinematics
    double pos0[3], ori0[3];
    computeForwardKinematics(jointAngles, pos0, ori0);
    Vector3d vPos0(pos0[0], pos0[1], pos0[2]);
    Vector3d vOri0(ori0[0], ori0[1], ori0[2]);

    // Für jede Gelenkvariable j eine kleine Veränderung durchführen
    for (int j = 0; j < 6; j++) {
        double perturbed[6];
        memcpy(perturbed, baseJoint, 6 * sizeof(double));
        perturbed[j] += delta;

        double posD[3], oriD[3];
        computeForwardKinematics(perturbed, posD, oriD);
        Vector3d vPosD(posD[0], posD[1], posD[2]);
        Vector3d vOriD(oriD[0], oriD[1], oriD[2]);

        // Numerischer Gradient: Δpos/Δθ, Δori/Δθ
        Vector3d dPos = (vPosD - vPos0) / delta;
        Vector3d dOri = (vOriD - vOri0) / delta;

        // Setze Jacobianspalte j
        J(0,j) = dPos[0];
        J(1,j) = dPos[1];
        J(2,j) = dPos[2];
        J(3,j) = dOri[0];
        J(4,j) = dOri[1];
        J(5,j) = dOri[2];
    }
}

/**
 * @brief Berechnet die Pseudoinverse einer 6×6-Matrix mittels SVD.
 *
 * @param A       Eingangs-Jacobian (6×6)
 * @param A_pinv  Ausgabe: Pseudoinverse (6×6)
 */
static void computePseudoInverse(const Matrix<double,6,6>& A, Matrix<double,6,6>& A_pinv) {
    JacobiSVD<Matrix<double,6,6>> svd(A, ComputeFullU | ComputeFullV);
    const double tol = 1e-6; // Toleranz für singuläre Werte
    VectorXd singularValues = svd.singularValues();
    Matrix<double,6,6> S_pinv = Matrix<double,6,6>::Zero();

    for (int i = 0; i < 6; i++) {
        if (singularValues[i] > tol) {
            S_pinv(i,i) = 1.0 / singularValues[i];
        }
    }
    A_pinv = svd.matrixV() * S_pinv * svd.matrixU().transpose();
}

bool computeInverseKinematics(const double targetPos[3],
                              const double targetOri[3],
                              double initialGuess[6],
                              double solution[6],
                              const IKSettings& settings) {
    // Kopiere initialGuess in working array
    double theta[6];
    for (int i = 0; i < 6; i++) theta[i] = initialGuess[i];

    for (int iter = 0; iter < settings.maxIterations; iter++) {
        // 1) Aktuelle FK aus theta berechnen
        double currPosArr[3], currOriArr[3];
        computeForwardKinematics(theta, currPosArr, currOriArr);
        Vector3d currPos(currPosArr[0], currPosArr[1], currPosArr[2]);
        Vector3d currOri(currOriArr[0], currOriArr[1], currOriArr[2]);

        Vector3d desiredPos(targetPos[0], targetPos[1], targetPos[2]);
        Vector3d desiredOri(targetOri[0], targetOri[1], targetOri[2]);

        // 2) Fehlervektor e (6×1): Δpos und Δori
        Vector3d ePos = desiredPos - currPos;
        Vector3d eOri = desiredOri - currOri;

        // Reduziere Orientierungsfehler auf [-π, π]
        for (int k = 0; k < 3; k++) {
            while (eOri[k] > M_PI)  eOri[k] -= 2.0 * M_PI;
            while (eOri[k] < -M_PI) eOri[k] += 2.0 * M_PI;
        }

        double errPosNorm = ePos.norm();
        double errOriNorm = eOri.norm();

        // Prüfe Toleranzen
        if (errPosNorm < settings.positionTolerance && errOriNorm < settings.orientationTolerance) {
            // Konvergenz erreicht
            for (int i = 0; i < 6; i++) solution[i] = theta[i];
            return true;
        }

        // 3) Jacobian numerisch berechnen
        Matrix<double,6,6> J;
        computeJacobianNumeric(theta, J);

        // 4) Pseudoinverse von J
        Matrix<double,6,6> J_pinv;
        computePseudoInverse(J, J_pinv);

        // 5) Δθ = stepSize * J_pinv * [ePos; eOri]
        Vector<double,6> e;
        e << ePos[0], ePos[1], ePos[2], eOri[0], eOri[1], eOri[2];
        Vector<double,6> deltaTheta = settings.stepSize * (J_pinv * e);

        // 6) Update θ
        for (int i = 0; i < 6; i++) {
            theta[i] += deltaTheta[i];
            // Enforce joint limits:
            double minA = jointLimits[i].minAngle;
            double maxA = jointLimits[i].maxAngle;
            if (theta[i] < minA) theta[i] = minA;
            if (theta[i] > maxA) theta[i] = maxA;
        }
    }

    // Maximale Iterationen erreicht, keine Konvergenz
    return false;
}
