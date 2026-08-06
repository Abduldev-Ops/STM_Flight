/*
 * ekf.c
 *
 *  Created on: Jul 17, 2026
 *      Author: akanb
 */


#include "flight/ekf.h"
#include "string.h"
#include "math.h"

void EKF_Init(EKF_t *ekf)
{
	memset(ekf, 0, sizeof(EKF_t));

	//Initial covariance

	ekf->P[0][0] = 1.0f;
	ekf->P[1][1] = 1.0f;
	ekf->P[2][2] = 1.0f;
	ekf->P[3][3] = 1.0f;

	//Process noise

	ekf->Q[0][0] = 0.001f;
	ekf->Q[1][1] = 0.001f;
	ekf->Q[2][2] = 0.003f;
	ekf->Q[3][3] = 0.003f;

	//measurement noise

	ekf->R[0][0] = 0.003f;
	ekf->R[1][1] = 0.003f;
}

void EKF_Predict(EKF_t *ekf, float gx, float gy, float dt)
{
	//bias corrected  gyro rates
	float roll_rate = gx - ekf->x[2];
	float pitch_rate = gy - ekf->x[3];

	// State prediction
	ekf->x[0] += roll_rate  * dt;
    ekf->x[1] += pitch_rate * dt;
    // Biases modeled as constant — x[2] and x[3] unchanged

    // F matrix (state transition Jacobian)
    // F = [1, 0, -dt, 0 ]
    //     [0, 1,  0, -dt]
    //     [0, 0,  1,  0 ]
    //     [0, 0,  0,  1 ]

    // P = F*P*F^T + Q
    // Expanded manually for 4x4 (avoids dynamic matrix library)
    float P00 = ekf->P[0][0] - dt*ekf->P[2][0] - dt*ekf->P[0][2] + dt*dt*ekf->P[2][2] + ekf->Q[0][0];
    float P01 = ekf->P[0][1] - dt*ekf->P[2][1] - dt*ekf->P[0][3] + dt*dt*ekf->P[2][3];
    float P02 = ekf->P[0][2] - dt*ekf->P[2][2];
    float P03 = ekf->P[0][3] - dt*ekf->P[2][3];

    float P10 = ekf->P[1][0] - dt*ekf->P[3][0] - dt*ekf->P[1][2] + dt*dt*ekf->P[3][2];
    float P11 = ekf->P[1][1] - dt*ekf->P[3][1] - dt*ekf->P[1][3] + dt*dt*ekf->P[3][3] + ekf->Q[1][1];
    float P12 = ekf->P[1][2] - dt*ekf->P[3][2];
    float P13 = ekf->P[1][3] - dt*ekf->P[3][3];

    float P20 = ekf->P[2][0] - dt*ekf->P[2][2];
    float P21 = ekf->P[2][1] - dt*ekf->P[2][3];
    float P22 = ekf->P[2][2] + ekf->Q[2][2];
    float P23 = ekf->P[2][3];

    float P30 = ekf->P[3][0] - dt*ekf->P[3][2];
    float P31 = ekf->P[3][1] - dt*ekf->P[3][3];
    float P32 = ekf->P[3][2];
    float P33 = ekf->P[3][3] + ekf->Q[3][3];

    ekf->P[0][0]=P00; ekf->P[0][1]=P01; ekf->P[0][2]=P02; ekf->P[0][3]=P03;
    ekf->P[1][0]=P10; ekf->P[1][1]=P11; ekf->P[1][2]=P12; ekf->P[1][3]=P13;
    ekf->P[2][0]=P20; ekf->P[2][1]=P21; ekf->P[2][2]=P22; ekf->P[2][3]=P23;
    ekf->P[3][0]=P30; ekf->P[3][1]=P31; ekf->P[3][2]=P32; ekf->P[3][3]=P33;
}

void EKF_Update(EKF_t *ekf, float roll_meas, float pitch_meas)
{
    // Innovation (measurement residual)
    float y0 = roll_meas  - ekf->x[0];
    float y1 = pitch_meas - ekf->x[1];

    // S = H*P*H^T + R
    // H extracts rows 0 and 1, so S is just the top-left 2x2 of P plus R
    float S00 = ekf->P[0][0] + ekf->R[0][0];
    float S01 = ekf->P[0][1];
    float S10 = ekf->P[1][0];
    float S11 = ekf->P[1][1] + ekf->R[1][1];

    // S^-1 (2x2 matrix inverse)
    float det = S00*S11 - S01*S10;
    if (fabsf(det) < 1e-10f) return;  // singular — skip update

    float Si00 =  S11 / det;
    float Si01 = -S01 / det;
    float Si10 = -S10 / det;
    float Si11 =  S00 / det;

    // K = P*H^T*S^-1
    // P*H^T is just columns 0 and 1 of P
    float K00 = ekf->P[0][0]*Si00 + ekf->P[0][1]*Si10;
    float K01 = ekf->P[0][0]*Si01 + ekf->P[0][1]*Si11;
    float K10 = ekf->P[1][0]*Si00 + ekf->P[1][1]*Si10;
    float K11 = ekf->P[1][0]*Si01 + ekf->P[1][1]*Si11;
    float K20 = ekf->P[2][0]*Si00 + ekf->P[2][1]*Si10;
    float K21 = ekf->P[2][0]*Si01 + ekf->P[2][1]*Si11;
    float K30 = ekf->P[3][0]*Si00 + ekf->P[3][1]*Si10;
    float K31 = ekf->P[3][0]*Si01 + ekf->P[3][1]*Si11;

    // State update
    ekf->x[0] += K00*y0 + K01*y1;
    ekf->x[1] += K10*y0 + K11*y1;
    ekf->x[2] += K20*y0 + K21*y1;
    ekf->x[3] += K30*y0 + K31*y1;

    // P update: P = (I - K*H)*P
    // K*H affects only columns 0 and 1
    float P00 = (1 - K00)*ekf->P[0][0] - K01*ekf->P[1][0];
    float P01 = (1 - K00)*ekf->P[0][1] - K01*ekf->P[1][1];
    float P02 = (1 - K00)*ekf->P[0][2] - K01*ekf->P[1][2];
    float P03 = (1 - K00)*ekf->P[0][3] - K01*ekf->P[1][3];

    float P10 = -K10*ekf->P[0][0] + (1 - K11)*ekf->P[1][0];
    float P11 = -K10*ekf->P[0][1] + (1 - K11)*ekf->P[1][1];
    float P12 = -K10*ekf->P[0][2] + (1 - K11)*ekf->P[1][2];
    float P13 = -K10*ekf->P[0][3] + (1 - K11)*ekf->P[1][3];

    float P20 = -K20*ekf->P[0][0] - K21*ekf->P[1][0] + ekf->P[2][0];
    float P21 = -K20*ekf->P[0][1] - K21*ekf->P[1][1] + ekf->P[2][1];
    float P22 = -K20*ekf->P[0][2] - K21*ekf->P[1][2] + ekf->P[2][2];
    float P23 = -K20*ekf->P[0][3] - K21*ekf->P[1][3] + ekf->P[2][3];

    float P30 = -K30*ekf->P[0][0] - K31*ekf->P[1][0] + ekf->P[3][0];
    float P31 = -K30*ekf->P[0][1] - K31*ekf->P[1][1] + ekf->P[3][1];
    float P32 = -K30*ekf->P[0][2] - K31*ekf->P[1][2] + ekf->P[3][2];
    float P33 = -K30*ekf->P[0][3] - K31*ekf->P[1][3] + ekf->P[3][3];

    ekf->P[0][0]=P00; ekf->P[0][1]=P01; ekf->P[0][2]=P02; ekf->P[0][3]=P03;
    ekf->P[1][0]=P10; ekf->P[1][1]=P11; ekf->P[1][2]=P12; ekf->P[1][3]=P13;
    ekf->P[2][0]=P20; ekf->P[2][1]=P21; ekf->P[2][2]=P22; ekf->P[2][3]=P23;
    ekf->P[3][0]=P30; ekf->P[3][1]=P31; ekf->P[3][2]=P32; ekf->P[3][3]=P33;
}
