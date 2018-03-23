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

#ifndef LPEMG_SENSOR_MANAGER
#define LPEMG_SENSOR_MANAGER

#include "LpemgSensorManagerI.h"
#include "LpemgSensor.h"
#include "CalibrationData.h"
#include "LpemgIoInterface.h"
#include "DeviceListItem.h"
#include "util.h"

#ifdef ANDROID
using namespace std;

#include <jni.h>
#include <android/log.h>

#include "AndroidBluetooth.h"	
#endif

#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>

#pragma warning( disable: 4251 )

#include "pugixml.hpp"

/* See LpemgSensorManagerI for comments on this class. */
class LpemgSensorManager : public LpemgSensorManagerI
{
    const std::string TAG;
public:
#ifndef ANDROID		
    LpemgSensorManager(void);
#else
    LpemgSensorManager(JavaVM* thisVm, jobject bluetoothAdapter);
#endif

    ~LpemgSensorManager(void) override;
    void start(void);
    void run(void);
    LpemgSensorI* addSensor(int mode, const char* deviceId) override;
    void removeSensor(LpemgSensorI* sensor) override;
    void startListDevices(bool scan_serial_ports) override;
    bool listDevicesBusy(void) override;
    void stopListDevices(void) override;
    LpmsDeviceList getDeviceList(void) override;
    bool saveSensorData(const char* fn) override;
    void stopSaveSensorData(void) override;
    bool isRecordingActive(void) override;
    void setThreadTiming(int delay) override;
    int getThreadTiming(void) override;
    void setVerbose(bool v) override;

private:
    list<LpemgSensorI*> sensorList;
    bool stopped;
    std::string configurationFile;
    LpmsDeviceList deviceList;
    int managerState;
    std::ofstream saveDataHandle;
    bool isRecording;
    std::mutex lm;
    int threadDelay;
    char writeBuffer[65536];
    bool scan_serial_ports_;
    MicroMeasure savePeriodTimer;
    int frameCounter;
    long savePeriod;
    int currentUartBaudrate;
    bool verbose;

#ifdef ANDROID
    JavaVM* thisVm;
    jobject bluetoothAdapter;
#endif
};

#endif
