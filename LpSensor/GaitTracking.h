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

#ifndef REP_DETECTION_THREAD
#define REP_DETECTION_THREAD

#include "stdio.h"
#include "math.h"

#include "ImuData.h"
#include "LpMatrix.h"

typedef struct _PeakDetector {
	int pP;
	float peakAcc;
	float peakV;
	float pH;
	float pT;
	float vel;
	float lpVel;
	float lpAcc;
	int dir;
	int repC;
	float stLp;
	float stC;
	float pSt;
	float stA;
	float aC;
	float aLp;
	int first;
	float ppA;
	float stF;
	float stV;
	float pDSt;
	float sym;
} PeakDetector;

typedef struct _RingBuffer {
	float v[1024];
	int l;
	int i;
} RingBuffer;

typedef struct _LinAccIntegrator {
	float aLp0, aLp1, v0, vHp0, x0, xHp0, xLp0;
	float x1, v1, xHp1, xLp1, o, vHp1;
	float c0, c1, c2, c3, c4, c5, c6, c7, c8, c9;
	float avgTime;
	float x, tX;
	int r;
} LinAccIntegrator;

class GaitTracking {
public:
	GaitTracking(void);
	void update(ImuData *d);
	void getData(float *h, int *d, float *t, float *a, int *rc, float *acc, float *v);
	
public:
	RingBuffer rb;
	RingBuffer yRb;
	LinAccIntegrator gLAI;
	LinAccIntegrator yLAI;
	PeakDetector gPD;
	PeakDetector yPD;
	int first;
	float pT;
};

#endif