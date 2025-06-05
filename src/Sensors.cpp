#include "Sensors.h"
#include <math.h>

// =====================
// getSensorBasedZAndTilt
// =====================
void getSensorBasedZAndTilt(double outTiltRollPitch[2], double& outCorrectedZ) {
    float tiltRad = InitSystem::getTiltAngleRad();
    double rollRad  = 0.0;
    double pitchRad = (double)tiltRad;

    float h_mm = InitSystem::getCorrectedLaserHeight(tiltRad);
    double h_m  = (double)h_mm / 1000.0;

    outTiltRollPitch[0] = rollRad;
    outTiltRollPitch[1] = pitchRad;
    outCorrectedZ       = h_m;
}

// =====================
// Extended Kalman Pose Filter
// =====================
void sensorsEkfInit(float z0, float pitch0) {
    initPoseEkf(z0, pitch0);
}

void sensorsEkfUpdate(double& zFiltered, double& pitchFiltered) {
    double tiltRP[2];
    double zMeas;
    getSensorBasedZAndTilt(tiltRP, zMeas);
    predictPoseEkf();
    updatePoseEkf((float)zMeas, (float)tiltRP[1]);
    zFiltered     = poseEkf.state[0];
    pitchFiltered = poseEkf.state[1];
}

// =====================
// applySensorCorrections
// =====================
void applySensorCorrections(double& zKorrigiert,
                            double& rollKorrigiert,
                            double& pitchKorrigiert) {
    double zF, pitchF;
    sensorsEkfUpdate(zF, pitchF);
    zKorrigiert    = zF;
    pitchKorrigiert = pitchF;
    rollKorrigiert  = 0.0; // no roll sensor available
}
