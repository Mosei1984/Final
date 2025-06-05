#ifndef POSE_EKF_H
#define POSE_EKF_H

struct PoseEKF {
    float state[2];     // [z (m), pitch (rad)]
    float P[2][2];      // covariance matrix
    float Q[2];         // process noise
    float R[2];         // measurement noise

    void init(float z0, float pitch0) {
        state[0] = z0;
        state[1] = pitch0;
        P[0][0] = 1.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 0.1f;
        Q[0] = 0.0001f; Q[1] = 0.001f;
        R[0] = 0.01f;   R[1] = 0.05f;
    }

    void predict() {
        P[0][0] += Q[0];
        P[1][1] += Q[1];
    }

    void update(float zMeas, float pitchMeas) {
        float y0 = zMeas - state[0];
        float S0 = P[0][0] + R[0];
        float K0 = P[0][0] / S0;
        state[0] += K0 * y0;
        P[0][0] *= (1.0f - K0);

        float y1 = pitchMeas - state[1];
        float S1 = P[1][1] + R[1];
        float K1 = P[1][1] / S1;
        state[1] += K1 * y1;
        P[1][1] *= (1.0f - K1);
    }
};

extern PoseEKF poseEkf;

void initPoseEkf(float z0, float pitch0);
void predictPoseEkf();
void updatePoseEkf(float zMeas, float pitchMeas);

#endif // POSE_EKF_H
