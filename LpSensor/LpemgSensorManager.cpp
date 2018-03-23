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

#include "LpemgSensorManager.h" 

#ifdef BUILD_BLUETOOTH
#include "LpemgBluetooth.h"
#endif

#define SENSOR_UPDATE_PERIOD 1000

#define SMANAGER_LIST 0
#define SMANAGER_MEASURE 1

#ifdef _WIN32
LpemgSensorManagerI* APIENTRY LpemgSensorManagerFactory(void)
{
    return (LpemgSensorManagerI*) new LpemgSensorManager();
}

LpemgSensorManager::LpemgSensorManager(void)
    :TAG("LpemgSensorManager")
#endif

#ifdef __GNUC__
LpemgSensorManagerI* LpemgSensorManagerFactory(void)
{
    return (LpemgSensorManagerI*) new LpemgSensorManager();
}

LpemgSensorManager::LpemgSensorManager(void)
#endif

#ifdef ANDROID
LpemgSensorManagerI* APIENTRY LpemgSensorManagerFactory(JavaVM *thisVm, jobject bluetoothAdapter)
{
    return (LpemgSensorManagerI*) new LpemgSensorManager(JavaVM *thisVm, jobject bluetoothAdapter);
}

LpemgSensorManager::LpemgSensorManager(JavaVM *thisVm, jobject bluetoothAdapter) :
thisVm(thisVm),
bluetoothAdapter(bluetoothAdapter)
#endif
{
    stopped = false;
    isRecording = false;
    threadDelay = 500;
    currentUartBaudrate = SELECT_LPMS_UART_BAUDRATE_115200;
    verbose = true;
    managerState = SMANAGER_MEASURE;

    std::thread t(&LpemgSensorManager::run, this);

#ifdef _WIN32	
#ifdef THREAD_HIGH_PRIORITY
    HANDLE th = t.native_handle();
    SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
#endif
#endif

    t.detach();
#ifdef ANDROID
    LOGV("[LpemgSensorManager] Started");
#endif
}

LpemgSensorManager::~LpemgSensorManager(void)
{
    stopped = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    deviceList.clear();
}

void LpemgSensorManager::startListDevices(bool scan_serial_ports)
{
    if (listDevicesBusy() == true) return;

    scan_serial_ports_ = scan_serial_ports;

    deviceList.clear();
    managerState = SMANAGER_LIST;

    return;
}

bool LpemgSensorManager::listDevicesBusy(void) {
    return managerState == SMANAGER_LIST;
}

void LpemgSensorManager::stopListDevices(void) {
    if (managerState != SMANAGER_LIST) return;

#ifdef BUILD_BLUETOOTH
    LpemgBluetooth::stopDiscovery();
#endif

    managerState = SMANAGER_MEASURE;
    if (verbose)
        logd(TAG, "Cancelling discovery\n");
    return;
}

void LpemgSensorManager::start(void)
{
    stopped = false;
    std::thread t(&LpemgSensorManager::run, this);

#ifdef _WIN32
#ifdef THREAD_HIGH_PRIORITY
    HANDLE th = t.native_handle();
    SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
#endif
#endif

    t.detach();
}


void LpemgSensorManager::run(void)
{
    MicroMeasure mm;

    float prevTimestamp = 0.0f;
    int deviceType = 0;

#ifdef ANDROID
    LOGV("[LpemgSensorManager] Thread running\n");
#endif

    mm.reset();
    int sleepFlag = 0;
    while (stopped == false) {
        switch (managerState) {
        case SMANAGER_MEASURE:
            {
                std::unique_lock<std::mutex> lock(lm);
                for (auto i = sensorList.begin(); i != sensorList.end(); i++) {
                    (*i)->pollData();
                }
            }

            if (mm.measure() > threadDelay) {
                mm.reset();

                std::unique_lock<std::mutex> lock(lm);
                for (auto i = sensorList.begin(); i != sensorList.end(); i++) {
                    (*i)->update();
                }
            }
            break;

        case SMANAGER_LIST:
            deviceList.clear();

            if (managerState != SMANAGER_LIST)
                break;
#ifdef BUILD_BLUETOOTH
            LpemgBluetooth::listDevices(&deviceList);
#endif

            managerState = SMANAGER_MEASURE;
            break;
        }

#ifdef __GNUC__
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }
}


LpemgSensorI* LpemgSensorManager::addSensor(int mode, const char *deviceId)
{
    LpemgSensor* sensor;
    std::unique_lock<std::mutex> lock(lm);

    switch (mode) {
    case DEVICE_LPEMG_B:

#ifndef ANDROID
        sensor = new LpemgSensor(DEVICE_LPEMG_B, deviceId);
#else
        sensor = new LpemgSensor(DEVICE_LPEMG_B, deviceId, thisVm, bluetoothAdapter);

        LOGV("[LpemgSensorManager] Sensor LpemgB added\n");
#endif

        sensorList.push_back(sensor);
        break;
    }
    //sensor->setVerbose(verbose);

    return sensor;
}

void LpemgSensorManager::removeSensor(LpemgSensorI *sensor)
{
    sensor->close();

    std::unique_lock<std::mutex> lock(lm);
    sensorList.remove(sensor);

    delete sensor;
}

LpmsDeviceList LpemgSensorManager::getDeviceList(void)
{
    return deviceList;
}

bool LpemgSensorManager::isRecordingActive(void)
{
    return isRecording;
}

bool LpemgSensorManager::saveSensorData(const char* fn)
{
    if (isRecording)
        return false;

    saveDataHandle.open(fn, ios_base::out);
    saveDataHandle.rdbuf()->pubsetbuf(writeBuffer, 65536);
    if (!saveDataHandle.is_open()) {
        if (verbose)
            logd(TAG, "Failed to open %s\n", fn);

        return false;
    }

    EmgData::writeCSVHeader(saveDataHandle);

    if (verbose)
        logd(TAG, "Writing Lpemg data to %s\n", fn);

    std::unique_lock<std::mutex> lock(lm);
    for (auto i = sensorList.begin(); i != sensorList.end(); ++i) {
        (*i)->startSaveData(&saveDataHandle);
    }
    isRecording = true;

    return true;
}


void LpemgSensorManager::stopSaveSensorData(void)
{
    if (isRecording == false) return;

    std::unique_lock<std::mutex> lock(lm);
    for (auto i = sensorList.begin(); i != sensorList.end(); ++i) {
        (*i)->stopSaveData();
    }
    //if (saveDataHandle != NULL) saveDataHandle.close();
    if (saveDataHandle.is_open())
        saveDataHandle.close();

    isRecording = false;
}

void LpemgSensorManager::setThreadTiming(int delay)
{
    if (delay >= 0) {
        threadDelay = delay;
    }
}

int LpemgSensorManager::getThreadTiming(void)
{
    return threadDelay;
}


void LpemgSensorManager::setVerbose(bool v)
{
    verbose = v;
}
