/**
  ******************************************************************************
  * @file           : imu_ekf.h
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

#ifndef _IMU_EKF_H
#define _IMU_EKF_H

typedef struct{	
	float gx; float gy; float gz;
	float ax; float ay; float az;
}imu_data_f32_t;


typedef struct{
	float pitch;
	float roll;
	float yaw;
}euler_t;

void imu_ekf_init(void);
void imu_ekf_update(imu_data_f32_t* _imu, float dt);
void imu_ekf_eular(euler_t* euler);

#endif

