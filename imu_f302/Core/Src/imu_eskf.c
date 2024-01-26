/**
  ******************************************************************************
  * @file           : imu_eskf.c
  * @brief          : Quaternion Based error-state KF
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
  Date        Author        Notes
  2024/01/26 SourceLizi  Initial release
  */
#include "imu_eskf.h"

#define __FPU_PRESENT 1
#include "arm_math.h"
#include <math.h>

#define Q_SIZE 4 //quaternion size
#define VEC_SIZE 3 //vector size

static float F32_Xk[Q_SIZE*1];
static float F32_Xk_pri[Q_SIZE*1];
static float F32_Pk[VEC_SIZE*VEC_SIZE];
static float F32_Pk_pri[VEC_SIZE*VEC_SIZE];

static float F32_F[VEC_SIZE*VEC_SIZE];
static float F32_H[VEC_SIZE*VEC_SIZE];
static float F32_K[VEC_SIZE*VEC_SIZE];
static float F32_VEC[VEC_SIZE*1];

static float F32_R[VEC_SIZE*VEC_SIZE];
static float F32_G[VEC_SIZE*VEC_SIZE];

static float F32_TMP1[VEC_SIZE*VEC_SIZE];
static float F32_TMP2[VEC_SIZE*VEC_SIZE];

arm_matrix_instance_f32 F  = {VEC_SIZE,VEC_SIZE,F32_F};
arm_matrix_instance_f32 Xk = {VEC_SIZE,1,F32_Xk};
arm_matrix_instance_f32 Xk_pri = {VEC_SIZE,1,F32_Xk_pri};
arm_matrix_instance_f32 Pk  = {VEC_SIZE,VEC_SIZE,F32_Pk};
arm_matrix_instance_f32 Pk_pri  = {VEC_SIZE,VEC_SIZE,F32_Pk_pri};

arm_matrix_instance_f32 H = {VEC_SIZE,VEC_SIZE,F32_H};
arm_matrix_instance_f32 K = {VEC_SIZE,VEC_SIZE,F32_K};
arm_matrix_instance_f32 vec = {VEC_SIZE,1,F32_VEC};

arm_matrix_instance_f32 R = {VEC_SIZE,VEC_SIZE,F32_R};

arm_matrix_instance_f32 G = {VEC_SIZE,VEC_SIZE,F32_G};

arm_matrix_instance_f32 tmp1 = {VEC_SIZE,VEC_SIZE,F32_TMP1};
arm_matrix_instance_f32 tmp2 = {VEC_SIZE,VEC_SIZE,F32_TMP2};

arm_status result;

#define SIGMA_R (60.0f)
#define SIGMA_Q (0.0144f)
#define K_GQ (500.0f)
#define PK_INIT (1e-6f)

void imu_eskf_init(void){
	memset(F32_F,VEC_SIZE*VEC_SIZE*sizeof(float),0);
	F32_F[VEC_SIZE*0+0] = F32_F[VEC_SIZE*1+1] = F32_F[VEC_SIZE*2+2] = 1.0f;
	memset(F32_Pk,VEC_SIZE*VEC_SIZE*sizeof(float),0);
	F32_Pk[VEC_SIZE*0+0] = PK_INIT;
	F32_Pk[VEC_SIZE*1+1] = PK_INIT;
	F32_Pk[VEC_SIZE*2+2] = PK_INIT;
	F32_Xk[0] = 1.0f; F32_Xk[1] = 0.0f; F32_Xk[2] = 0.0f; F32_Xk[3] = 0.0f;
}

void get_skew_matrix(float* raw_matrix,float* v3d, float diag_val){
	raw_matrix[VEC_SIZE*0+0] = diag_val, raw_matrix[VEC_SIZE*0+1] = -v3d[2], raw_matrix[VEC_SIZE*0+2] = v3d[1];
	raw_matrix[VEC_SIZE*1+0] = v3d[2], raw_matrix[VEC_SIZE*1+1] = diag_val, raw_matrix[VEC_SIZE*1+2] = -v3d[0];
	raw_matrix[VEC_SIZE*2+0] = -v3d[1], raw_matrix[VEC_SIZE*2+1] = v3d[0], raw_matrix[VEC_SIZE*2+2] = diag_val;
}

void get_negitive_skew_matrix(float* raw_matrix,float* v3d, float diag_val){
	raw_matrix[VEC_SIZE*0+0] = diag_val, raw_matrix[VEC_SIZE*0+1] = v3d[2], raw_matrix[VEC_SIZE*0+2] = -v3d[1];
	raw_matrix[VEC_SIZE*1+0] = -v3d[2], raw_matrix[VEC_SIZE*1+1] = diag_val, raw_matrix[VEC_SIZE*1+2] = v3d[0];
	raw_matrix[VEC_SIZE*2+0] = v3d[1], raw_matrix[VEC_SIZE*2+1] = -v3d[0], raw_matrix[VEC_SIZE*2+2] = diag_val;
}

void imu_eskf_update(imu_data_f32_t* _imu, float dt){
	static float quat2[Q_SIZE*1];
	static float d_gyro[VEC_SIZE],h_q[VEC_SIZE];
	static float sigma_r,dt2;
	static float a_norm , q_norm;
	
	arm_sqrt_f32(_imu->ax * _imu->ax + _imu->ay * _imu->ay + _imu->az * _imu->az, &a_norm);
	dt2 = dt*dt;
	//proc: x_hat = f(x_hat) = A*x_k
	d_gyro[0] = _imu->gx * dt; d_gyro[1] = _imu->gy * dt; d_gyro[2] = _imu->gz * dt;
	F32_Xk_pri[0] = F32_Xk[0] + (-d_gyro[0]*F32_Xk[1] - d_gyro[1]*F32_Xk[2] - d_gyro[2]*F32_Xk[3])/2;
	F32_Xk_pri[1] = F32_Xk[1] + (d_gyro[0]*F32_Xk[0] - d_gyro[1]*F32_Xk[3] + d_gyro[2]*F32_Xk[2])/2;
	F32_Xk_pri[2] = F32_Xk[2] + (d_gyro[0]*F32_Xk[3] + d_gyro[1]*F32_Xk[0] - d_gyro[2]*F32_Xk[1])/2;
	F32_Xk_pri[3] = F32_Xk[3] + (-d_gyro[0]*F32_Xk[2] + d_gyro[1]*F32_Xk[1] + d_gyro[2]*F32_Xk[0])/2;

	quat2[0] = F32_Xk_pri[0]*F32_Xk_pri[0];
	quat2[1] = F32_Xk_pri[1]*F32_Xk_pri[1];
	quat2[2] = F32_Xk_pri[2]*F32_Xk_pri[2];
	quat2[3] = F32_Xk_pri[3]*F32_Xk_pri[3];
	arm_sqrt_f32(quat2[0] + quat2[1] + quat2[2] + quat2[3], &q_norm);
	F32_Xk_pri[0] /= q_norm; F32_Xk_pri[1] /= q_norm; F32_Xk_pri[2] /= q_norm; F32_Xk_pri[3] /= q_norm;

	get_negitive_skew_matrix(F32_F,d_gyro,1.0f);
	
	
	if(a_norm > 0.0f){

		//proc: P_hat = F*P_k*F^T + Q
		tmp1.numRows = VEC_SIZE; tmp1.numCols = VEC_SIZE;
		tmp2.numRows = VEC_SIZE; tmp2.numCols = VEC_SIZE;

		//1.tmp1 = F^T, tmp2=Pk*tmp1=Pk*F^T
		result = arm_mat_trans_f32(&F, &tmp1);
		result = arm_mat_mult_f32(&Pk, &tmp1, &tmp2);
		//2.Pk_pri = F*tmp2=F*Pk*F^T
		result = arm_mat_mult_f32(&F, &tmp2, &Pk_pri);
		
		//2.Pk_pri = F*Pk*F^T + Q
		F32_Pk_pri[VEC_SIZE*0+0] += SIGMA_Q*dt2;
		F32_Pk_pri[VEC_SIZE*1+1] += SIGMA_Q*dt2;
		F32_Pk_pri[VEC_SIZE*2+2] += SIGMA_Q*dt2;
		
		//proc: K=(P_hat*H^T)/(H*P_hat*H^T + R)
		h_q[0] =  2.0f*(F32_Xk_pri[1]*F32_Xk_pri[3] - F32_Xk_pri[0]*F32_Xk_pri[2]);
		h_q[1] =  2.0f*(F32_Xk_pri[2]*F32_Xk_pri[3] + F32_Xk_pri[0]*F32_Xk_pri[1]);
		h_q[2] =  (quat2[0] - quat2[1] - quat2[2] + quat2[3]);
		//vec = [h(q)]x
		get_skew_matrix(F32_H,h_q,0.0f);

		// tmp1.numRows= VEC_SIZE; tmp1.numCols = VEC_SIZE;
		// tmp2.numRows= VEC_SIZE; tmp2.numCols = VEC_SIZE;
		
		//1.tmp1 = H^T, tmp2=Pk_pri*tmp1=Pk_pri*H^T
		result = arm_mat_trans_f32(&H, &tmp1);
		result = arm_mat_mult_f32(&Pk_pri, &tmp1, &tmp2);
		//2. R = H*tmp2 = H*Pk_pri*H^T
		result = arm_mat_mult_f32(&H, &tmp2, &R);
		
		//3.R = R + I3*sigma_r
		sigma_r = (K_GQ*fabsf(a_norm-1)+SIGMA_R)*dt2;
		//sigma_r = (SIGMA_R)*dt2;
		F32_R[VEC_SIZE*0+0] += sigma_r;
		F32_R[VEC_SIZE*1+1] += sigma_r;
		F32_R[VEC_SIZE*2+2] += sigma_r;

		//4. tmp1 = R^-1
		//tmp1.numRows= VEC_SIZE; tmp1.numCols = VEC_SIZE;
		
		result = arm_mat_inverse_f32(&R,&tmp1);
		if(result != ARM_MATH_SUCCESS){
			memcpy(F32_Xk,F32_Xk_pri,Q_SIZE*1*sizeof(float));
			return;
		}
		//5. K=tmp2*tmp1= Pk_pri*H^T*(H*Pk_pri*H^T + I3*sigma_r)^-1
		result = arm_mat_mult_f32(&tmp2, &tmp1, &K);

		//proc: dtheta = K*(z-h(q))
		//1.tmp1 = z-h(q)
		F32_TMP1[0] = _imu->ax/a_norm -h_q[0];
		F32_TMP1[1] = _imu->ay/a_norm -h_q[1];
		F32_TMP1[2] = _imu->az/a_norm -h_q[2];

		tmp1.numRows= VEC_SIZE; tmp1.numCols = 1;
		//2.dtheta = vec = K*tmp1 = K*(z-h(q))
		result = arm_mat_mult_f32(&K, &tmp1, &vec);

		//proc: P_k=(I-KH)P_hat(I-KH)^T+KRK^T
		
		//1. Pk = I-KH
		tmp1.numRows = VEC_SIZE; tmp1.numCols = VEC_SIZE;
		//tmp2.numRows = VEC_SIZE; tmp2.numCols = VEC_SIZE;
		
		result = arm_mat_mult_f32(&K, &H, &Pk);
		result = arm_mat_scale_f32(&Pk, -1.0f, &Pk);
		F32_Pk[VEC_SIZE*0+0] += 1.0f;
		F32_Pk[VEC_SIZE*1+1] += 1.0f;
		F32_Pk[VEC_SIZE*2+2] += 1.0f;
		//2. tmp1 = Pk*Pk_pri = (I-KH)*Pk_pri, tmp2 = Pk^T = (I-KH)^T
		result = arm_mat_trans_f32(&Pk, &tmp2);
		result = arm_mat_mult_f32(&Pk, &Pk_pri, &tmp1);
		
		
		//3.  Pk = tmp1*tmp2 = (I-KH)*Pk_pri*(I-KH)^T
		result = arm_mat_mult_f32(&tmp1, &tmp2, &Pk);
		
		//4. tmp1 = K^T, tmp2=K*tmp1=K*K^T, tmp1 = sigma_r*tmp2 = sigma_r*K*K^T

		result = arm_mat_trans_f32(&K, &tmp1);
		result = arm_mat_mult_f32(&K, &tmp1, &tmp2);
		result = arm_mat_scale_f32(&tmp2, sigma_r, &tmp1);
		
		//5.Pk=Pk+tmp1= (I-KH)*Pk_pri*(I-KH)^T+sigma_r*K*K^T
		
		result = arm_mat_add_f32(&Pk, &tmp1, &Pk);
		
		
	}else{
		memcpy(F32_Xk,F32_Xk_pri,Q_SIZE*1*sizeof(float));
		return;
	}

	//normalize quaternion
	//dtheta/2 = vec /2;
	F32_VEC[0] /= 2; F32_VEC[1] /= 2; F32_VEC[2] /= 2;
	//q=q_hat times [1 dtheta/2]
	F32_Xk[0] = F32_Xk_pri[0] + (-F32_VEC[0]*F32_Xk_pri[1] - F32_VEC[1]*F32_Xk_pri[2] - F32_VEC[2]*F32_Xk_pri[3]);
	F32_Xk[1] = F32_Xk_pri[1] + (F32_VEC[0]*F32_Xk_pri[0] - F32_VEC[1]*F32_Xk_pri[3] + F32_VEC[2]*F32_Xk_pri[2]);
	F32_Xk[2] = F32_Xk_pri[2] + (F32_VEC[0]*F32_Xk_pri[3] + F32_VEC[1]*F32_Xk_pri[0] - F32_VEC[2]*F32_Xk_pri[1]);
	F32_Xk[3] = F32_Xk_pri[3] + (-F32_VEC[0]*F32_Xk_pri[2] + F32_VEC[1]*F32_Xk_pri[1] + F32_VEC[2]*F32_Xk_pri[0]);

	quat2[0] = F32_Xk[0]*F32_Xk[0];
	quat2[1] = F32_Xk[1]*F32_Xk[1];
	quat2[2] = F32_Xk[2]*F32_Xk[2];
	quat2[3] = F32_Xk[3]*F32_Xk[3];
	arm_sqrt_f32(quat2[0] + quat2[1] + quat2[2] + quat2[3], &q_norm);
	
	if(q_norm > 0.0f){
		//proc: Pk|corr=G*Pk*G^T
		//1.get jacobian matrix G:
		
		//G=I-[dtheta/2]x
		get_negitive_skew_matrix(F32_G,F32_VEC,1.0f);
		
		// tmp1.numRows = VEC_SIZE; tmp1.numCols = VEC_SIZE;
		// tmp2.numRows = VEC_SIZE; tmp2.numCols = VEC_SIZE;
		//2. tmp1 = G^T
		result = arm_mat_trans_f32(&G, &tmp1);
		//3. tmp2 = G*Pk
		result = arm_mat_mult_f32(&G, &Pk, &tmp2);
		//4. Pk|corr = G*Pk*G^T = tmp2*tmp1
		result = arm_mat_mult_f32(&tmp2, &tmp1, &Pk);

		//normalize quaternion
		//q_norm = sqrtf(F32_Xk[0]*F32_Xk[0] + F32_Xk[1]*F32_Xk[1] + F32_Xk[2]*F32_Xk[2] + F32_Xk[3]*F32_Xk[3]);
		F32_Xk[0] /= q_norm; F32_Xk[1] /= q_norm; F32_Xk[2] /= q_norm; F32_Xk[3] /= q_norm;
	}else{
		imu_eskf_init();
	}
}

void imu_eskf_eular(euler_t* euler){
	euler->pitch = asinf(-2 * F32_Xk[1] * F32_Xk[3] + 2 * F32_Xk[0]* F32_Xk[2]);	// pitch
	euler->roll  = atan2f(2 *( F32_Xk[2] * F32_Xk[3] + F32_Xk[0] * F32_Xk[1]), 1 -2 * (F32_Xk[1] * F32_Xk[1] +  F32_Xk[2]* F32_Xk[2]));	// roll
	euler->yaw   = atan2f(2* (F32_Xk[1] * F32_Xk[2] + F32_Xk[0] * F32_Xk[3]), 1 - 2*(F32_Xk[2] * F32_Xk[2] + F32_Xk[3] * F32_Xk[3]));	//yaw
	
	// arm_atan2_f32(2 * F32_Xk[2] * F32_Xk[3] + 2 * F32_Xk[0] * F32_Xk[1], -2 * F32_Xk[1] * F32_Xk[1] - 2 * F32_Xk[2]* F32_Xk[2] + 1, &euler->roll);	// roll
	// arm_atan2_f32(2* (F32_Xk[1] * F32_Xk[2] + F32_Xk[0] * F32_Xk[3]), F32_Xk[0] * F32_Xk[0] + F32_Xk[1] * F32_Xk[1] - F32_Xk[2] * F32_Xk[2] - F32_Xk[3] * F32_Xk[3], &euler->yaw);	//yaw
	
	euler->pitch *= 180.0f/3.14159265f;
	euler->roll *= 180.0f/3.14159265f;
	euler->yaw *= 180.0f/3.14159265f;
}

