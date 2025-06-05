#include "PoseEKF.h"

PoseEKF poseEkf;

void initPoseEkf(float z0, float pitch0) {
    poseEkf.init(z0, pitch0);
}

void predictPoseEkf() {
    poseEkf.predict();
}

void updatePoseEkf(float zMeas, float pitchMeas) {
    poseEkf.update(zMeas, pitchMeas);
}
