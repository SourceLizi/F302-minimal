/**
  ******************************************************************************
  * @file           : imu_ekf.c
  * @brief          : Quaternion Based EKF
  ******************************************************************************
  * @attention
  *
  *  Copyright (c) 2022-2023 Lizi <lizi_bussiness[AT]outlook.com>
  *  
  *  Permission is hereby granted, free of charge, to any person obtaining a copy
  *  of this software and associated documentation files (the "Software"), to deal
  *  in the Software without restriction, including without limitation the rights
  *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  *  copies of the Software, and to permit persons to whom the Software is
  *  furnished to do so, subject to the following conditions:
  *  
  *  The above copyright notice and this permission notice shall be included in all
  *  copies or substantial portions of the Software.
  *  
  *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  *  SOFTWARE.
  ******************************************************************************
  */
#include "imu_ekf.h"

#define __FPU_PRESENT 1
#include "arm_math.h"
#include <math.h>

#define Q_SIZE 4 //quaternion size
#define VEC_SIZE 3 //vector size

static float F32_Xk[Q_SIZE*1];
static float F32_Xk_pri[Q_SIZE*1];
static float F32_Pk[Q_SIZE*Q_SIZE];
static float F32_Pk_pri[Q_SIZE*Q_SIZE];
static float F32_Zk[VEC_SIZE*1];

static float F32_A[Q_SIZE*Q_SIZE];
static float F32_H[VEC_SIZE*Q_SIZE];
static float F32_K[Q_SIZE*VEC_SIZE];

static float F32_R[VEC_SIZE*VEC_SIZE];
static float F32_Q[Q_SIZE*Q_SIZE];

static float F32_TMP1[Q_SIZE*Q_SIZE];
static float F32_TMP2[Q_SIZE*Q_SIZE];

arm_matrix_instance_f32 Xk = {Q_SIZE,1,F32_Xk};
arm_matrix_instance_f32 Xk_pri = {Q_SIZE,1,F32_Xk_pri};
arm_matrix_instance_f32 Pk  = {Q_SIZE,Q_SIZE,F32_Pk};
arm_matrix_instance_f32 Pk_pri  = {Q_SIZE,Q_SIZE,F32_Pk_pri};
arm_matrix_instance_f32 Zk  = {VEC_SIZE,1,F32_Zk};

arm_matrix_instance_f32 A = {Q_SIZE,Q_SIZE,F32_A};
arm_matrix_instance_f32 H = {VEC_SIZE,Q_SIZE,F32_H};
arm_matrix_instance_f32 K = {Q_SIZE,VEC_SIZE,F32_K};

arm_matrix_instance_f32 R = {VEC_SIZE,VEC_SIZE,F32_R};
arm_matrix_instance_f32 Q = {Q_SIZE,Q_SIZE,F32_Q};

arm_matrix_instance_f32 tmp1 = {Q_SIZE,Q_SIZE,F32_TMP1};
arm_matrix_instance_f32 tmp2 = {Q_SIZE,Q_SIZE,F32_TMP2};

arm_status result;

float sigma_r = 1e-5;
float sigma_q = 0.045f;

void imu_ekf_init(void){
	memset(F32_A,Q_SIZE*Q_SIZE*sizeof(float),0);
	F32_A[Q_SIZE*0+0] = F32_A[Q_SIZE*1+1] = F32_A[Q_SIZE*2+2] = F32_A[Q_SIZE*3+3] = 1.0f;
	memset(F32_Pk,Q_SIZE*Q_SIZE*sizeof(float),0);

	F32_Xk[0] = 1.0f; F32_Xk[1] = 0.0f; F32_Xk[2] = 0.0f; F32_Xk[3] = 0.0f;
}

void imu_ekf_update(imu_data_f32_t* _imu, float dt){
	float dx, dy, dz;
	float a_norm , q_norm;
	
	arm_sqrt_f32(_imu->ax * _imu->ax + _imu->ay * _imu->ay + _imu->az * _imu->az, &a_norm);

	//proc: x_hat = f(x_hat) = A*x_k
	dx = _imu->gx * dt /2; dy = _imu->gy * dt /2; dz = _imu->gz * dt /2;
	F32_A[Q_SIZE*0+1] = -dx, F32_A[Q_SIZE*1+0] = dx, F32_A[Q_SIZE*2+3] = dx, F32_A[Q_SIZE*3+2] = -dx;
	F32_A[Q_SIZE*0+2] = -dy, F32_A[Q_SIZE*2+0] = dy, F32_A[Q_SIZE*1+3] = -dy, F32_A[Q_SIZE*3+1] = dy;
	F32_A[Q_SIZE*0+3] = -dz, F32_A[Q_SIZE*3+0] = dz, F32_A[Q_SIZE*2+1] = -dz, F32_A[Q_SIZE*1+2] = dz;
	
	result = arm_mat_mult_f32(&A, &Xk, &Xk_pri);
	
	if(a_norm > 0.0f){

		//proc: P_hat = A*P_k*A^T + Q
		tmp1.numRows = Q_SIZE; tmp1.numCols = Q_SIZE;
		tmp2.numRows = Q_SIZE; tmp2.numCols = Q_SIZE;

		//1.tmp1 = A^T, tmp2=Pk*tmp1=Pk*A^T
		result = arm_mat_trans_f32(&A, &tmp1);
		result = arm_mat_mult_f32(&Pk, &tmp1, &tmp2);
		//2.Pk_pri = A*tmp2=A*Pk*A^T
		result = arm_mat_mult_f32(&A, &tmp2, &Pk_pri);

		//3.Q=sigma_q*W*W^T
		tmp1.numRows = Q_SIZE; tmp1.numCols = VEC_SIZE;
		tmp2.numRows = VEC_SIZE; tmp2.numCols = Q_SIZE;	

		F32_TMP1[VEC_SIZE*1+0] = F32_Xk[0] * dt/2, F32_TMP1[VEC_SIZE*2+1] = F32_Xk[0] * dt/2, F32_TMP1[VEC_SIZE*3+2] = F32_Xk[0] * dt/2;
		F32_TMP1[VEC_SIZE*0+0] = -F32_Xk[1] * dt/2, F32_TMP1[VEC_SIZE*2+2] = -F32_Xk[1] * dt/2, F32_TMP1[VEC_SIZE*3+1] = F32_Xk[1] * dt/2;
		F32_TMP1[VEC_SIZE*0+1] = -F32_Xk[2] * dt/2, F32_TMP1[VEC_SIZE*1+2] = F32_Xk[2] * dt/2, F32_TMP1[VEC_SIZE*3+0] = -F32_Xk[2] * dt/2;
		F32_TMP1[VEC_SIZE*0+2] = -F32_Xk[3] * dt/2, F32_TMP1[VEC_SIZE*1+1] = -F32_Xk[3] * dt/2, F32_TMP1[VEC_SIZE*2+0] = F32_Xk[3] * dt/2;
		//tmp1=W, tmp2=W^T
		result = arm_mat_trans_f32(&tmp1, &tmp2);
		//Q=tmp1*tmp2 = W*W^T
		result = arm_mat_mult_f32(&tmp1, &tmp2, &Q);
		//Q=sigma_q*Q=sigma_q*W*W^T
		result = arm_mat_scale_f32(&Q, sigma_q, &Q);

		//Pk_pri = Pk_pri + Q
		result = arm_mat_add_f32(&Pk_pri, &Q, &Pk_pri);

		//proc: K=(P_hat*H^T)/(H*P_hat*H^T + R)
		F32_H[Q_SIZE*2+0] = 2.0f*F32_Xk_pri[0], F32_H[Q_SIZE*1+1] = 2.0f*F32_Xk_pri[0], F32_H[Q_SIZE*0+2] = -2.0f*F32_Xk_pri[0];
		F32_H[Q_SIZE*1+0] = 2.0f*F32_Xk_pri[1], F32_H[Q_SIZE*2+1] = -2.0f*F32_Xk_pri[1], F32_H[Q_SIZE*0+3] = 2.0f*F32_Xk_pri[1];
		F32_H[Q_SIZE*0+0] = -2.0f*F32_Xk_pri[2], F32_H[Q_SIZE*1+3] = 2.0f*F32_Xk_pri[2], F32_H[Q_SIZE*2+2] = -2.0f*F32_Xk_pri[2];
		F32_H[Q_SIZE*0+1] = 2.0f*F32_Xk_pri[3], F32_H[Q_SIZE*1+2] = 2.0f*F32_Xk_pri[3], F32_H[Q_SIZE*2+3] = 2.0f*F32_Xk_pri[3];

		tmp1.numRows= Q_SIZE; tmp1.numCols = VEC_SIZE;
		tmp2.numRows= Q_SIZE; tmp2.numCols = VEC_SIZE;
		
		//1.tmp1 = H^T, tmp2=Pk_pri*tmp1=Pk_pri*H^T
		result = arm_mat_trans_f32(&H, &tmp1);
		result = arm_mat_mult_f32(&Pk_pri, &tmp1, &tmp2);
		//2. R = H*tmp2 = H*Pk_pri*H^T
		result = arm_mat_mult_f32(&H, &tmp2, &R);
		
		//3.R = R + I3*sigma_r
		F32_R[VEC_SIZE*0+0] += sigma_r;
		F32_R[VEC_SIZE*1+1] += sigma_r;
		F32_R[VEC_SIZE*2+2] += sigma_r;

		//4. tmp1 = R^-1
		tmp1.numRows= VEC_SIZE; tmp1.numCols = VEC_SIZE;
		
		result = arm_mat_inverse_f32(&R,&tmp1);
		if(result == ARM_MATH_SINGULAR){
			memcpy(F32_Xk,F32_Xk_pri,Q_SIZE*1*sizeof(float));
			return;
		}
		//5. K=tmp2*tmp1= Pk_pri*H^T*(H*Pk_pri*H^T + I3*sigma_r)^-1
		result = arm_mat_mult_f32(&tmp2, &tmp1, &K);

		//proc: P_k=(I-KH)P_hat(I-KH)^T+KRK^T
		
		//1. Pk = I-KH
		tmp1.numRows = Q_SIZE; tmp1.numCols = Q_SIZE;
		tmp2.numRows = Q_SIZE; tmp2.numCols = Q_SIZE;
		
		result = arm_mat_mult_f32(&K, &H, &Pk);
		result = arm_mat_scale_f32(&Pk, -1.0f, &Pk);
		F32_TMP1[Q_SIZE*0+0] += 1;
		F32_TMP1[Q_SIZE*1+1] += 1;
		F32_TMP1[Q_SIZE*2+2] += 1;
		//2. tmp1 = Pk*Pk_pri = (I-KH)*Pk_pri, tmp2 = Pk^T = (I-KH)^T
		result = arm_mat_mult_f32(&Pk, &Pk_pri, &tmp1);
		result = arm_mat_trans_f32(&Pk, &tmp2);
		
		//3.  Pk = tmp1*tmp2 = (I-KH)*Pk_pri*(I-KH)^T
		result = arm_mat_mult_f32(&tmp1, &tmp2, &Pk);
		
		//4. tmp1 = K^T, tmp2=sigma_r*K*tmp1=sigma_r*K*K^T
		tmp1.numRows = VEC_SIZE; tmp1.numCols = Q_SIZE;
		result = arm_mat_trans_f32(&K, &tmp1);
		
		tmp2.numRows = Q_SIZE; tmp2.numCols = Q_SIZE;
		result = arm_mat_mult_f32(&K, &tmp1, &tmp2);
		result = arm_mat_scale_f32(&tmp2, sigma_r, &tmp2);
		
		//5.Pk=Pk+tmp2= (I-KH)*Pk_pri*(I-KH)^T+sigma_r*K*K^T
		
		result = arm_mat_add_f32(&Pk, &tmp2, &Pk);
		
		//proc: x_k = x_hat + K(zk - h(x_hat))
		F32_Zk[0] = _imu->ax/a_norm - 2.0f*(F32_Xk_pri[1]*F32_Xk_pri[3] - F32_Xk_pri[0]*F32_Xk_pri[2]);
		F32_Zk[1] = _imu->ay/a_norm - 2.0f*(F32_Xk_pri[2]*F32_Xk_pri[3] + F32_Xk_pri[0]*F32_Xk_pri[1]);
		F32_Zk[2] = _imu->az/a_norm - (F32_Xk_pri[0]*F32_Xk_pri[0] - F32_Xk_pri[1]*F32_Xk_pri[1] - F32_Xk_pri[2]*F32_Xk_pri[2] + F32_Xk_pri[3]*F32_Xk_pri[3]);

		tmp1.numRows = Q_SIZE; tmp1.numCols = 1;

		result = arm_mat_mult_f32(&K, &Zk, &tmp1);

		result = arm_mat_add_f32(&tmp1, &Xk_pri, &Xk);
		
	}else{
		memcpy(F32_Xk,F32_Xk_pri,Q_SIZE*1*sizeof(float));
	}

	//normalize quaternion
	//q_norm = sqrtf(F32_Xk[0]*F32_Xk[0] + F32_Xk[1]*F32_Xk[1] + F32_Xk[2]*F32_Xk[2] + F32_Xk[3]*F32_Xk[3]);
	arm_sqrt_f32(F32_Xk[0]*F32_Xk[0] + F32_Xk[1]*F32_Xk[1] + F32_Xk[2]*F32_Xk[2] + F32_Xk[3]*F32_Xk[3], &q_norm);
	if(q_norm > 0.0f){
		F32_Xk[0] /= q_norm; F32_Xk[1] /= q_norm; F32_Xk[2] /= q_norm; F32_Xk[3] /= q_norm;
	}else{
		imu_ekf_init();
	}
}

void imu_ekf_eular(euler_t* euler){
	euler->pitch = asinf(-2 * F32_Xk[1] * F32_Xk[3] + 2 * F32_Xk[0]* F32_Xk[2]);	// pitch
	euler->roll  = atan2f(2 *( F32_Xk[2] * F32_Xk[3] + F32_Xk[0] * F32_Xk[1]), 1 -2 * (F32_Xk[1] * F32_Xk[1] +  F32_Xk[2]* F32_Xk[2]));	// roll
	euler->yaw   = atan2f(2* (F32_Xk[1] * F32_Xk[2] + F32_Xk[0] * F32_Xk[3]), 1 - 2*(F32_Xk[2] * F32_Xk[2] + F32_Xk[3] * F32_Xk[3]));	//yaw
	
	// arm_atan2_f32(2 * F32_Xk[2] * F32_Xk[3] + 2 * F32_Xk[0] * F32_Xk[1], -2 * F32_Xk[1] * F32_Xk[1] - 2 * F32_Xk[2]* F32_Xk[2] + 1, &euler->roll);	// roll
	// arm_atan2_f32(2* (F32_Xk[1] * F32_Xk[2] + F32_Xk[0] * F32_Xk[3]), F32_Xk[0] * F32_Xk[0] + F32_Xk[1] * F32_Xk[1] - F32_Xk[2] * F32_Xk[2] - F32_Xk[3] * F32_Xk[3], &euler->yaw);	//yaw
	
	euler->pitch *= 180.0f/3.14159265f;
	euler->roll *= 180.0f/3.14159265f;
	euler->yaw *= 180.0f/3.14159265f;
}

