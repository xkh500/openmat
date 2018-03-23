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

#include "LpMagnetometerCalibration.h"

float bCalPythag(float a, float b)
{
	return 0.0f;
}

void bCalOrientationFromAccMag(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc)
{
}

void bCalInitEllipsoidFit(void)
{
}

void bCalUpdateBMap(LpVector3f r, LpVector3f bRaw)
{
}

int bCalCalcSVD(float **mat, int m, int n, float **w, float **v, int maxCalElements)
{
	return 0;
}

int bCalFitEllipsoid(void)
{
	return 1;
}	

void bCalTestEllipsoidFit(void)
{
}

LpMatrix3x3f bCalGetSoftIronMatrix(void)
{
	LpMatrix3x3f m;
	
	matZero3x3(&m);
	
	return m;
}

LpVector3f bCalGetHardIronOffset(void)
{
	LpVector3f v;
	
	vectZero3x1(&v);
	
	return v;
}

float bCalGetFieldRadius(void)
{
	return 0.0f;
}

void bCalSetSoftIronMatrix(LpMatrix3x3f m)
{
}

void bCalSetHardIronOffset(LpVector3f v)
{
}

void bCalSetFieldRadius(float r)
{
}

float bCalGetFieldMapElement(int i, int j, int k, int l)
{
	return 0.0f;
}

LpVector3f bCalCorrect(LpVector3f b)
{
	LpVector3f v;
	
	vectZero3x1(&v);
	
	return v;
}

void getReferenceYZ(LpVector3f b, LpVector3f a, LpVector3f *r, float *inc)
{
}