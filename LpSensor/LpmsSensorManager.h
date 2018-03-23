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

#ifndef LPMS_SENSOR_MANAGER
#define LPMS_SENSOR_MANAGER

#ifdef _WIN32
// Windows headers collide among themselves, so we have to make sure that this is included before windows.h
#include <winsock2.h>
#endif

#include "LpmsSensorManagerI.h"
#include "LpmsSensor.h"
#include "CalibrationData.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"
#include "util.h"
#ifdef _WIN32
#include "LpmsU2.h"
#include "CanEngine.h"
#include "LpmsRS232.h"
#include "LpmsTcp.h"
#endif

#ifdef __GNUC__
#include "CanEngine.h"
//#include "LpmsU2.h"
#endif

#ifdef ANDROID
using namespace std;

#include <jni.h>
#include <android/log.h>

#include "AndroidBluetooth.h"	
#endif

// #include <boost/thread/thread.hpp> 
// #include <boost/foreach.hpp>

#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>

#ifdef _WIN32
#ifdef DLL_EXPORT
#define DLL_MACRO __declspec(dllexport)
#else
#define DLL_MACRO __declspec(dllimport)
#endif
#else
#define DLL_MACRO
#endif

#pragma warning( disable: 4251 )

#include "pugixml.hpp"

// See LpmsSensorManagerI for comments on this class
class LpmsSensorManager : public LpmsSensorManagerI
{
    const std::string TAG;
public:
#ifndef ANDROID		
    LpmsSensorManager(void);
#else
    LpmsSensorManager(JavaVM* thisVm, jobject bluetoothAdapter);
#endif

    ~LpmsSensorManager(void) override;
    void start(void);
    void run(void);
    LpmsSensorI* addSensor(int mode, const char* deviceId) override;
    void removeSensor(LpmsSensorI* sensor) override;
    void startListDevices(bool scan_serial_ports) override;
    bool listDevicesBusy(void) override;
    void stopListDevices(void) override;
    LpmsDeviceList getDeviceList(void) override;
    bool saveSensorData(const char* fn) override;
    void stopSaveSensorData(void) override;
    bool isRecordingActive(void) override;
    void setThreadTiming(int delay) override;
    int getThreadTiming(void) override;
    bool isCanPresent(void) override;
    void setCanBaudrate(int i) override;
    void setRs232Baudrate(int i) override;
    void setVerbose(bool v) override;

private:
    list<LpmsSensor*> sensorList;
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

#ifdef _WIN32	
    CanEngine ce;
    //	BleEngine be;
#endif

#ifdef ANDROID
    JavaVM* thisVm;
    jobject bluetoothAdapter;
#endif
};

#endif
