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

#ifndef LPEMG_SENSOR_MANAGER_I
#define LPEMG_SENSOR_MANAGER_I

#include "LpemgSensorI.h"
#include "DeviceListItem.h"

#ifdef _WIN32
#pragma warning( disable: 4251 )
#endif

// Manages LpemgSensorI objects. */
class LpemgSensorManagerI
{
public:
    // SensorManager destructor
    virtual ~LpemgSensorManagerI(void) { };

    // Adds one Lpms device to the list of sensor devices
    virtual LpemgSensorI* addSensor(int mode, const char* deviceId) = 0;

    // Removes an Lpms device from the list of connected devices
    virtual void removeSensor(LpemgSensorI* sensor) = 0;

    // Lists all connected LPMS devices. The device desicovery runs in a seperate thread
    virtual void startListDevices(bool scan_serial_ports) = 0;

    // Indicates if device detection is busy
    virtual bool listDevicesBusy(void) = 0;

    // Cancels device detection
    virtual void stopListDevices(void) = 0;

    // Gets a lists of detected devices
    virtual LpmsDeviceList getDeviceList(void) = 0;

    // Adjusts the tightness of the thread polling. Higher values mean
    // less CPU usage. If you don't have any resource problems,
    // leave this at its default value 750.
    virtual void setThreadTiming(int delay) = 0;

    // Get the current thread polling timing
    virtual int getThreadTiming(void) = 0;

    // Starts saving sensor data to file fn
    virtual bool saveSensorData(const char* fn) = 0;

    // Stops saving sensor data
    virtual void stopSaveSensorData(void) = 0;

    // Checks, if sensor data is currently being recorded
    virtual bool isRecordingActive(void) = 0;

    // Sets library verbosity. True to output debug information
    // Defaults to output debug information
    virtual void setVerbose(bool v) = 0;
};

#ifdef _WIN32
#include "windows.h"

#ifdef DLL_EXPORT
#define LP_DECLSPEC __declspec(dllexport)
#else // DLL_EXPORT
#define LP_DECLSPEC __declspec(dllimport)
#endif // DLL_EXPORT

#define LP_APIENTRY APIENTRY
#else // _WIN32
#define LP_DECLSPEC
#define LP_APIENTRY
#endif // _WIN32

#ifndef ANDROID
extern "C" LP_DECLSPEC LpemgSensorManagerI* LP_APIENTRY LpemgSensorManagerFactory(void);
#else // ANDROID
#include <jni.h>

extern "C" LpemgSensorManagerI* LpemgSensorManagerFactory(JavaVM* thisVm, jobject bluetoothAdapter);
#endif // ANDROID

#undef LP_DECLSPEC
#undef LP_APIENTRY

#endif
