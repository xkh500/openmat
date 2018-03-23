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

#include "LpSensorCWrapper.h"

#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"

LpmsSensorManagerI* manager;
LpmsSensorI* lpms;

int connected = 0;
int initialized = 0;

void APIENTRY initializeLpms(void)
{
	manager = LpmsSensorManagerFactory();
	initialized = 1;
}

void APIENTRY connectToLpmsB(const char* deviceId)
{
	if (initialized == 1) {
		lpms = manager->addSensor(DEVICE_LPMS_B, deviceId);
		connected = 1;
	}
}

void APIENTRY connectToLpmsCU(const char* deviceId)
{
	if (initialized == 1) {
		lpms = manager->addSensor(DEVICE_LPMS_U, deviceId);
		connected = 1;
	}
}

int APIENTRY getConnectionStatus(void)
{
	if (connected == 0) return SENSOR_CONNECTION_CONNECTING;
	
	return lpms->getConnectionStatus();
}

int APIENTRY getSensorStatus(void)
{
	if (connected == 0) return SENSOR_CONNECTION_CONNECTING;	
	
	return lpms->getSensorStatus();
}

float APIENTRY getQuaternionW(void)
{
	if (connected == 0) return 1.0f;
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		
		return d.q[0];
	}
		
	return 1.0f;
}

float APIENTRY getQuaternionX(void)
{
	if (connected == 0) return 0.0f;
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.q[1];
	}
	
	return 0.0f;
}

float APIENTRY getQuaternionY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.q[2];
	}
	
	return 0.0f;
}

float APIENTRY getQuaternionZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.q[3];
	}
	
	return 0.0f;
}

float APIENTRY getEulerX(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.r[0];
	}
	
	return 0.0f;
}

float APIENTRY getEulerY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.r[1];
	}
	
	return 0.0f;
}

float APIENTRY getEulerZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.r[2];
	}
	
	return 0.0f;
}

float APIENTRY getGyrX(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.g[0];
	}
	
	return 0.0f;
}

float APIENTRY getGyrY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.g[1];
	}
	
	return 0.0f;
}

float APIENTRY getGyrZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.g[2];
	}
	
	return 0.0f;
}

float APIENTRY getAccX(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.a[0];
	}
	
	return 0.0f;
}

float APIENTRY getAccY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.a[1];
	}
	
	return 0.0f;
}

float APIENTRY getAccZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.a[2];
	}
	
	return 0.0f;
}

float APIENTRY getMagX(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.b[0];
	}
	
	return 0.0f;
}

float APIENTRY getMagY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.b[1];
	}
	
	return 0.0f;
}

float APIENTRY getMagZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.b[2];
	}
	
	return 0.0f;
}

float APIENTRY getLinAccX(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.linAcc[0];
	}
	
	return 0.0f;
}

float APIENTRY getLinAccY(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.linAcc[1];
	}
	
	return 0.0f;
}

float APIENTRY getLinAccZ(void)
{
	if (connected == 0) return 0.0f;	
	
	if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
		lpms->getSensorStatus() == SENSOR_STATUS_RUNNING) {
		ImuData d = lpms->getCurrentData();
		return d.linAcc[2];
	}
	
	return 0.0f;
}

void APIENTRY disconnectLpms(void)
{
	if (connected == 1) manager->removeSensor(lpms);
	if (initialized == 1) delete manager;
}