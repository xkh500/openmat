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

#include "LpmsSensorManager.h" 

#ifdef BUILD_LPMS_U
#include "LpmsU.h"
#endif

#ifdef BUILD_BLUETOOTH
#include "LpmsBBluetooth.h"
#ifdef _WIN32
#include "BleEngine.h"
#endif
#endif

#define SENSOR_UPDATE_PERIOD 1000

#define SMANAGER_LIST 0
#define SMANAGER_MEASURE 1

#ifdef _WIN32
LpmsSensorManagerI* APIENTRY LpmsSensorManagerFactory(void)
{
    return (LpmsSensorManagerI*) new LpmsSensorManager();
}

LpmsSensorManager::LpmsSensorManager(void)
    :TAG("LpmsSensorManager")
#endif

#ifdef __GNUC__
LpmsSensorManagerI* LpmsSensorManagerFactory(void)
{
    return (LpmsSensorManagerI*) new LpmsSensorManager();
}

LpmsSensorManager::LpmsSensorManager(void)
#endif

#ifdef ANDROID
LpmsSensorManagerI* APIENTRY LpmsSensorManagerFactory(JavaVM *thisVm, jobject bluetoothAdapter)
{
    return (LpmsSensorManagerI*) new LpmsSensorManager(JavaVM *thisVm, jobject bluetoothAdapter);
}

LpmsSensorManager::LpmsSensorManager(JavaVM *thisVm, jobject bluetoothAdapter) :
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

    std::thread t(&LpmsSensorManager::run, this);

#ifdef _WIN32	
#ifdef THREAD_HIGH_PRIORITY
    HANDLE th = t.native_handle();
    SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
#endif
#endif

    t.detach();
#ifdef ANDROID
    LOGV("[LpmsSensorManager] Started");
#endif
}

LpmsSensorManager::~LpmsSensorManager(void)
{
    stopped = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    deviceList.clear();
}

void LpmsSensorManager::startListDevices(bool scan_serial_ports)
{
    if (listDevicesBusy() == true) return;

    scan_serial_ports_ = scan_serial_ports;

    deviceList.clear();
    managerState = SMANAGER_LIST;

    return;
}

bool LpmsSensorManager::listDevicesBusy(void) {
    if (managerState == SMANAGER_LIST) return true;

    return false;
}

void LpmsSensorManager::stopListDevices(void) {
    if (managerState != SMANAGER_LIST) return;

    CanEngine::stopDiscovery();
#ifdef BUILD_BLUETOOTH
    LpmsBBluetooth::stopDiscovery();
#endif

    managerState = SMANAGER_MEASURE;
    if (verbose)
        logd(TAG, "Cancelling discovery\n");
    return;
}

void LpmsSensorManager::start(void)
{
    stopped = false;
    std::thread t(&LpmsSensorManager::run, this);

#ifdef _WIN32
#ifdef THREAD_HIGH_PRIORITY
    HANDLE th = t.native_handle();
    SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
#endif
#endif

    t.detach();
}

#define LPMS_B_LATENCY_ESTIMATE 0.030f
#define LPMS_OTHER_LATENCY_ESTIMATE 0.005f

void LpmsSensorManager::run(void)
{
    MicroMeasure mm;

    float prevTimestamp = 0.0f;
    int deviceType = 0;

#ifdef _WIN32	
    ce.connect();
    // be.connect();
#endif	

#ifdef ANDROID
    LOGV("[LpmsSensorManager] Thread running\n");
#endif

    mm.reset();
    int sleepFlag = 0;
    while (stopped == false) {
        switch (managerState) {
        case SMANAGER_MEASURE:
            lm.lock();
            for (auto i = sensorList.begin(); i != sensorList.end(); i++) {
                (*i)->pollData();
            }

#ifdef _WIN32
            ce.poll();
#endif

            lm.unlock();

            if (mm.measure() > threadDelay) {
                mm.reset();

                lm.lock();
                for (auto i = sensorList.begin(); i != sensorList.end(); i++) {
                    (*i)->update();
                }
                lm.unlock();
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            break;

        case SMANAGER_LIST:
            deviceList.clear();

#ifdef _WIN32
            ce.listDevices(&deviceList);
            // be.listDevices(&deviceList);			
#endif
            if (managerState != SMANAGER_LIST)
                break;
            if (scan_serial_ports_ == true)
            {
                if (verbose)
                    logd(TAG, "List RS2323 devices\n");
                LpmsRS232::listDevices(&deviceList);
            }

            // if (managerState != SMANAGER_LIST)
                // break;
			// LpmsTcp::listDevices(&deviceList);

#ifdef BUILD_LPMS_U
            if (managerState != SMANAGER_LIST)
                break;
            LpmsU::listDevices(&deviceList);
#endif
#ifdef _WIN32
            if (managerState != SMANAGER_LIST)
                break;
            LpmsU2::listDevices(&deviceList);
#endif
#ifdef BUILD_BLUETOOTH
            if (managerState != SMANAGER_LIST)
                break;
            LpmsBBluetooth::listDevices(&deviceList);
#endif
            managerState = SMANAGER_MEASURE;
            break;
        }

#ifdef __GNUC__
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }

#ifdef _WIN32		
    ce.close();
    // be.close();
#endif
}


LpmsSensorI* LpmsSensorManager::addSensor(int mode, const char *deviceId)
{
    LpmsSensor* sensor;

    lm.lock();
    switch (mode) {
    case DEVICE_LPMS_B:

#ifndef ANDROID
        sensor = new LpmsSensor(DEVICE_LPMS_B, deviceId);
#else
        sensor = new LpmsSensor(DEVICE_LPMS_B, deviceId, thisVm, bluetoothAdapter);

        LOGV("[LpmsSensorManager] Sensor LPMSB added\n");
#endif

        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_BLE:
        sensor = new LpmsSensor(DEVICE_LPMS_BLE, deviceId);
        sensorList.push_back(sensor);

#ifdef _WIN32	
        // ((LpmsBle *)sensor->getIoInterface())->deviceId = deviceId;
        // be.addSensor((LpmsBle *)sensor->getIoInterface());
#endif
        break;

    case DEVICE_LPMS_C:
        sensor = new LpmsSensor(DEVICE_LPMS_C, deviceId);

#ifdef _WIN32		
        ce.addSensor((LpmsCanIo *)sensor->getIoInterface());
#endif

        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_U:
        sensor = new LpmsSensor(DEVICE_LPMS_U, deviceId);
        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_RS232:
        sensor = new LpmsSensor(DEVICE_LPMS_RS232, deviceId);
        ((LpmsRS232 *)sensor->getIoInterface())->setRs232Baudrate(currentUartBaudrate);
        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_B2:
        sensor = new LpmsSensor(DEVICE_LPMS_B2, deviceId);
        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_U2:
        sensor = new LpmsSensor(DEVICE_LPMS_U2, deviceId);
        ((LpmsRS232 *)sensor->getIoInterface())->setRs232Baudrate(SELECT_LPMS_UART_BAUDRATE_921600);
        sensorList.push_back(sensor);
        break;

    case DEVICE_LPMS_C2:
        sensor = new LpmsSensor(DEVICE_LPMS_C2, deviceId);

#ifdef _WIN32		
        ce.addSensor((LpmsCanIo *)sensor->getIoInterface());
        ce.setSensorToCommandMode(deviceId);
#endif

        sensorList.push_back(sensor);
        break;

	// case DEVICE_LPMS_TCP:	
		// sensor = new LpmsSensor(DEVICE_LPMS_TCP, deviceId);
		// sensorList.push_back(sensor);
	// break;	

    }

    sensor->setVerbose(verbose);
    lm.unlock();

    return (LpmsSensorI*)sensor;
}

void LpmsSensorManager::removeSensor(LpmsSensorI *sensor)
{
    sensor->close();

    lm.lock();
    sensorList.remove((LpmsSensor*)sensor);

#ifdef _WIN32
    ce.removeSensor((LpmsCanIo*) ((LpmsSensor*) sensor)->getIoInterface());
    // be.removeSensor((LpmsBle*) ((LpmsSensor*) sensor)->getIoInterface());	
#endif

    delete (LpmsSensor*)sensor;

    lm.unlock();
}

LpmsDeviceList LpmsSensorManager::getDeviceList(void)
{
    return deviceList;
}

bool LpmsSensorManager::isRecordingActive(void)
{
    return isRecording;
}

bool LpmsSensorManager::saveSensorData(const char* fn)
{
    list<LpmsSensor*>::iterator i;

    if (isRecording == true) return false;

    saveDataHandle.open(fn, ios_base::out);
    saveDataHandle.rdbuf()->pubsetbuf(writeBuffer, 65536);
    if (saveDataHandle.is_open() == true) {
        saveDataHandle << "SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g), GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX (deg), EulerY (deg), EulerZ (deg), QuatW, QuatX, QuatY, QuatZ, LinAccX (g), LinAccY (g), LinAccZ (g), Pressure (kPa), Altitude (m), Temperature (degC), HeaveMotion (m)\n";

        if (verbose)
            logd(TAG, "Writing LPMS data to %s\n", fn);
        lm.lock();
        for (i = sensorList.begin(); i != sensorList.end(); ++i) {
            (*i)->startSaveData(&saveDataHandle);
        }
        lm.unlock();

        isRecording = true;

        return true;
    }

    // cout << "[LpmsSensorManager] Failed to open " << fn << endl;
    if (verbose)
        logd(TAG, "Failed to open %s\n", fn);
    return false;
}

void LpmsSensorManager::stopSaveSensorData(void)
{
    list<LpmsSensor*>::iterator i;

    if (isRecording == false) return;

    lm.lock();
    for (i = sensorList.begin(); i != sensorList.end(); ++i) {
        (*i)->stopSaveData();
    }
    lm.unlock();

    isRecording = false;

    // if (saveDataHandle != NULL) saveDataHandle.close();
    if (saveDataHandle.is_open()) saveDataHandle.close();
}

void LpmsSensorManager::setThreadTiming(int delay)
{
    if (delay >= 0) {
        threadDelay = delay;
    }
}

int LpmsSensorManager::getThreadTiming(void)
{
    return threadDelay;
}

bool LpmsSensorManager::isCanPresent(void)
{
#ifdef _WIN32
    return ce.isInterfacePresent();
#else
    return false;
#endif
}

void LpmsSensorManager::setCanBaudrate(int i)
{
#ifdef _WIN32	
    ce.setBaudrate(i);
#endif
}

void LpmsSensorManager::setRs232Baudrate(int i)
{
    currentUartBaudrate = i;
}

void LpmsSensorManager::setVerbose(bool v)
{
    verbose = v;
}