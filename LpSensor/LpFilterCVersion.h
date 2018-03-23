/***********************************************************************
** Copyright (C) 2018 LP-RESEARCH Inc.
** All rights reserved
** Contact: info@lp-research.com
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with
** or without modification, are permitted provided that the
** following conditions are met:
**
** Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright
** notice, this list of conditions and the following disclaimer in
** the documentation and/or other materials provided with the
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef LP_FILTER_C_VERSION
#define LP_FILTER_C_VERSION

#ifndef __IAR_SYSTEMS_ICC__
	#include "stdio.h"
#endif

#include "LpMatrix.h"

typedef struct _LpFilterState {
	LpMatrix3x4f H;         // Direction cosine matrix
	LpMatrix3x4f tM3x4;     // Temporary
	LpMatrix4x3f Ht;        // Transposed H matrix
	LpMatrix4x4f P;         // Kalman filter P matrix	
	LpVector4f qV;          // Global quaternion
	LpVector3f tV3;         // Temporary 
	LpVector4f tV4;         // Temporary 
	LpVector3f yE;          // Accelerometer orientation error 
	LpMatrix3x3f tM3x3;     // Temporary
	LpMatrix3x3f S;         // Kalman filter S matrix
	LpMatrix3x3f Si;        // Inverted S matrix
	LpMatrix4x3f K;         // Accelerometer Kalman gain K
	LpMatrix4x3f tM4x3;     // Temporary
	LpMatrix4x4f tM4x4ii;   // Temporary
	LpMatrix4x4f tM4x4i;    // Temporary
	LpMatrix4x4f I;         // Identity
	LpMatrix4x4f Q;         // Covariance matrix
	float n;                // Norm
	LpMatrix4x4f F;         // Integration matrix
	LpMatrix4x4f Ft;        // Transposed integration matrix
	float aCG;              // Accelerometer compensation gain
	float bCG;              // Magnetometer compensation gain
	LpMatrix3x3f aR;        // Accelerometer noise covariance
	LpMatrix3x3f bR;        // Accelerometer noise covariance
	LpVector3f aRef;        // Accelerometer reference
	LpVector3f bRef;        // Magnetometer reference
} LpFilterState;

#ifdef __cplusplus
extern "C" {
#endif 

void QuaternionEKFInit(float aCGain, float aC,
	float bCGain, float bC,
	float aRX, float aRY, float aRZ,
	float bRX, float bRY, float bRZ);

void QuaternionEKF(float aX, float aY, float aZ,
	float gX, float gY, float gZ,
	float bX, float bY, float bZ,
	float oBX, float oBY, float oBZ,	
	float T,
	int useAC, int useBC,
	float *oQ0, float *oQ1, float *oQ2, float *oQ3,
	int useGyrT, 
	float gyrTx, float gyrTy, float gyrTz,
	int *magOutOfRange,
	float fitError);

void updateFilter(LpFilterState *fs,
	float aX, float aY, float aZ,
	float gX, float gY, float gZ,
	float bX, float bY, float bZ,
	float T,
	int useAC, int useBC,
	float *oQ0, float *oQ1, float *oQ2, float *oQ3,
	int useGyrT, 
	float gyrTx, float gyrTy, float gyrTz);

void initFilterState(LpFilterState* fs,
	float aCGain, float aC,
	float bCGain, float bC,
	float aRX, float aRY, float aRZ,
	float bRX, float bRY, float bRZ);
	
void updateMagCalibration(float *bX, float *bY, float *bZ);
	
#ifdef __cplusplus
}
#endif 
	
#endif