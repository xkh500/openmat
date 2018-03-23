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

#include "GaitTracking.h"

void resetPeakDetector(PeakDetector *pd)
{
}

void initIntegrator(LinAccIntegrator *lai)
{
}

void resetIntegrator(LinAccIntegrator *lai)
{
}

float lpFilter(float v, float *c, float a)
{
	return 0.0f;
}

float hpFilter(float v, float *c, float a)
{
	return 0.0f;
}

void integrateAcceleration(float T, float acc, LinAccIntegrator *lai, int rP)
{
}

void fitab(float *x, float *y, int n, float *a, float *b)
{
}

void initRB(RingBuffer *rb, int l)
{
}

void printRB(RingBuffer rb) 
{
}

void addRB(RingBuffer *rb, float v)
{
}

float getRBAvg(RingBuffer rb)
{
	return 0.0f;
}

void getRBAvgD(PeakDetector *pd, RingBuffer rb, float *b0, float *b1, float t, float acc, float h)
{
}

GaitTracking::GaitTracking(void)
{
}

void GaitTracking::update(ImuData *d)
{					
}