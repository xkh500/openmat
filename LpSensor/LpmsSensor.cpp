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

#include "LpmsSensor.h"

#include <iostream>
#include <iomanip>
#include "util.h"
#ifdef _WIN32
#include "windows.h"
#endif

#ifdef BUILD_BLUETOOTH
#include "LpmsBBluetooth.h"
#ifdef _WIN32
#include "LpmsBle2.h"
#endif
#endif


#ifdef BUILD_LPMS_U
#include "LpmsU.h"
#endif

const float pi = 3.141592f;

/***********************************************************************
** CONSTRUCTORS / DESTRUCTORS
***********************************************************************/

#ifdef WIN32
LpmsSensorI* APIENTRY LpmsSensorFactory(int deviceType, const char *deviceId)
{
    return (LpmsSensorI*) new LpmsSensor(deviceType, deviceId);
}

LpmsSensor::LpmsSensor(int deviceType, const char *deviceId) :
#endif

#ifdef __GNUC__
LpmsSensorI* LpmsSensorFactory(int deviceType, const char *deviceId)
{
    return (LpmsSensorI*) new LpmsSensor(deviceType, deviceId);
}

LpmsSensor::LpmsSensor(int deviceType, const char *deviceId) :
#endif

#ifdef ANDROID
LpmsSensorI* LpmsSensorFactory(int deviceType, const char *deviceId)
{
    return (LpmsSensorI*) new LpmsSensor(deviceType, deviceId);
}

LpmsSensor::LpmsSensor(int deviceType, const char *deviceId, JavaVM *thisVm, jobject bluetoothAdapter) :
#endif

TAG("LpmsSensor"),
deviceId(deviceId),
deviceType(deviceType) {

    this->configData.setParameter(PRM_DEVICE_ID, deviceId);
    this->configData.setParameter(PRM_DEVICE_TYPE, deviceType);
    this->configData.setParameter(PRM_OPENMAT_ID, 1);

    switch (deviceType) {

#ifdef ANDROID
    case DEVICE_LPMS_B:
        bt = new AndroidBluetooth(&(this->configData), thisVm, bluetoothAdapter);
        LOGV(TAG, "Sensor initialized\n");
        break;
#endif

#ifdef _WIN32
#ifdef BUILD_BLUETOOTH
    case DEVICE_LPMS_B:
    case DEVICE_LPMS_B2:
        bt = new LpmsBBluetooth(&(this->configData));
        break;

    case DEVICE_LPMS_BLE:
        bt = new LpmsBle(&(this->configData));
        break;
#endif

    case DEVICE_LPMS_C:
    case DEVICE_LPMS_C2:
        bt = new LpmsCanIo(&(this->configData));
        break;

#ifdef BUILD_LPMS_U
    case DEVICE_LPMS_U:
        bt = new LpmsU(&(this->configData));
        break;
#endif

    case DEVICE_LPMS_RS232:
        bt = new LpmsRS232(&(this->configData));
        break;

    case DEVICE_LPMS_U2:
        bt = new LpmsU2(&(this->configData));
        break;

#endif

	case DEVICE_LPMS_TCP:
		bt = new LpmsTcp(&(this->configData));
	break;

#ifdef __GNUC__
#ifdef BUILD_BLUETOOTH
    case DEVICE_LPMS_B:
    case DEVICE_LPMS_B2:
        bt = new LpmsBBluetooth(&(this->configData));
        break;
#endif

    case DEVICE_LPMS_C:
    case DEVICE_LPMS_C2:
        bt = new LpmsCanIo(&(this->configData));
        break;

    case DEVICE_LPMS_RS232:
        bt = new LpmsRS232(&(this->configData));
        break;

#ifdef BUILD_LPMS_U
    case DEVICE_LPMS_U:
        bt = new LpmsU(&(this->configData));
        break;
#endif

    case DEVICE_LPMS_U2:
        bt = new LpmsRS232(&(this->configData)); // using VCP for U2 devices
        ((LpmsRS232*) bt)->setRs232Baudrate(SELECT_LPMS_UART_BAUDRATE_921600);
        break;
        
#endif

    default:
        bt = new LpmsIoInterface(&(this->configData));
        break;
    }

    verbose = true;
    bt->setVerbose(verbose);
    paused = false;
    stopped = false;

    setSensorStatus(SENSOR_STATUS_PAUSED);
    setConnectionStatus(SENSOR_CONNECTION_CONNECTING);

    frameNo = 0;
    configReg = 0;

    state = STATE_CONNECT;
    retrialsConnect = 0;

    isGetMisalign = false;
    isGetGyrMisalign = false;
    misalignSetIndex = 0;
    isFirmwareUpdated = false;
    isSaveData = false;
    isMagCalibrationEnabled = false;
    isGetGyrTempCal = false;
    isPlanarMagCalibrationEnabled = false;
    isRefCalibrationEnabled = false;
    timestampOffset = 0.0;
    frameCounterOffset = 0;
    currentOffsetResetMethod = 2;
    lpmsCallback = 0;

    bt->zeroImuData(&currentData);
    connectRetry = 0;

}

LpmsSensor::~LpmsSensor(void)
{
    close();

    delete bt;

}

void LpmsSensor::setVerbose(bool v)
{
    verbose = v;
    bt->setVerbose(v);
}

void LpmsSensor::pollData(void)
{
    if (bt->deviceStarted() == true) {
        if (!bt->pollData())
            if (verbose) logd(TAG, "Poll Data error: %s\n", bt->getErrorMsg().c_str());

        bt->checkState();
    }
}

void LpmsSensor::assertConnected(void)
{
    if (bt->deviceStarted() == false) {
        state = STATE_CONNECT;
        setConnectionStatus(SENSOR_CONNECTION_INTERRUPTED);
        if (verbose) logd(TAG, "Connection interrupted. Sensor disconnected..\n");
    }
}

void LpmsSensor::update(void)
{
    ImuData imuData;
    int p;
    int pa[64];
    LpVector4f q;
    LpMatrix3x3f m;

    if (stopped == true) return;

    switch (state) {
        // Initiates the connection to the sensor using the hardware interface
    case STATE_CONNECT:
        setConnectionStatus(SENSOR_CONNECTION_CONNECTING);
        setSensorStatus(SENSOR_STATUS_PAUSED);
        lpmsTimer.reset();
        if (verbose) logd(TAG, "Connecting to %s\n", deviceId.c_str());

        if (!bt->connect(deviceId))
        {
            if (verbose) logd(TAG, "Connection error %d: %s\n", connectRetry,  bt->getErrorMsg().c_str());
            connectRetry++;
            setConnectionStatus(SENSOR_CONNECTION_FAILED);
            state = STATE_NONE;
            // if (connectRetry > 2)
            // {
            // setConnectionStatus(SENSOR_CONNECTION_FAILED);
            // state = STATE_NONE;
            // }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return;
        }
        if (verbose) logd(TAG, "Connected..\n");
        retrialsCommandMode = 0;
        prepareStream = 0;
        state = STATE_WAIT_CONNECT;
        break;

    // Waits for a successful connect for a certain time period
    case STATE_WAIT_CONNECT:
        if (lpmsTimer.measure() < bt->getConnectWait() && bt->deviceStarted() == false) {
            state = STATE_WAIT_CONNECT;
            // if (verbose) logd(TAG, "Wait connect\n");
            // bt->close();
        }
        else {
            if (bt->deviceStarted() == false) {
                setConnectionStatus(SENSOR_CONNECTION_FAILED);
                state = STATE_CONNECT;
                if (verbose) logd(TAG, "Connection failed(timeout)..\n");
                bt->close();
            }
            else {
                state = STATE_WAIT_AFTER_CONNECT;
            }
        }
        break;

    // Waits for a certains period after the connect 
    case STATE_WAIT_AFTER_CONNECT:
        if (lpmsTimer.measure() > WAIT_AFTER_CONNECT) {
            lpmsTimer.reset();
            if (verbose) logd(TAG, "Waiting after connect..\n");

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GOTO_COMMAND_MODE;
        }
        // setConnectionStatus(SENSOR_CONNECTION_CONNECTED);
        break;

    // Retrieves the current parameter settings of the sensor
    case STATE_GET_SETTINGS:

        assertConnected();
        if (connectionStatus == SENSOR_CONNECTION_INTERRUPTED)
        {
            // state = STATE_NONE;
            state = STATE_CONNECT;
            if (verbose) logd(TAG, "State get settings: connection interrupted..\n");
            break;
        }

        // wait for ack timeout
        // if (lpmsTimer.measure() > 5000000) {
            // if (verbose) logd(TAG, "wait for ack timeout..\n");
            // bt->close();
            // state = STATE_CONNECT;
        // }

        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) 
        {
            lpmsTimer.reset();

            switch (getConfigState) {
            // Switches to command mode.
            case C_STATE_GOTO_COMMAND_MODE:
                if (verbose) logd(TAG, "Switch to command mode\n");
                bt->setCommandMode();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_FIRMWARE_VERSION;
                break;

            // Retrieves firmware version
            case C_STATE_GET_FIRMWARE_VERSION:
                if (verbose) logd(TAG, "Get firmware version\n");
                bt->getFirmwareVersion();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_SERIAL_NUMBER;
                break;

            case C_STATE_GET_SERIAL_NUMBER:
                if (bt->isLegacyFirmware())
                {
                    state = STATE_GET_SETTINGS;
                    getConfigState = C_STATE_GET_CONFIG;
                }
                else
                {
                    if (verbose) logd(TAG, "Get serial number\n");
                    bt->getSensorSerialNumber();
                    state = STATE_GET_SETTINGS;
                    getConfigState = C_STATE_GET_DEVICE_NAME;
                }
                break;

            case C_STATE_GET_DEVICE_NAME:
                if (verbose) logd(TAG, "Get device name\n");
                bt->getSensorDeviceName();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_FIRMWARE_INFO;
                break;

            case C_STATE_GET_FIRMWARE_INFO:
                if (verbose) logd(TAG, "Get firmware info\n");
                bt->getSensorFirmwareInfo();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_CONFIG;
                break;

            // Gets the current configuration word
            case C_STATE_GET_CONFIG:
                if (verbose) logd(TAG, "Get configuration data\n");
                bt->getConfig();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_FILTER_MODE;
                break;

            // Retrieves the current filter mode
            case C_STATE_FILTER_MODE:
                if (verbose) logd(TAG, "Get filter mode\n");
                bt->getFilterMode();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_FILTER_PRESET;
                break;

            // Retrieves the current filter parameter preset
            case C_STATE_FILTER_PRESET:
                if (verbose) logd(TAG, "Get filter preset\n");
                bt->getFilterPreset();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GYR_RANGE;
                break;

            // Retrieves the current gyroscope range
            case C_STATE_GYR_RANGE:
                if (verbose) logd(TAG, "Get gyr. range parameters\n");
                bt->getGyrRange();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_ACC_RANGE;
                break;

            // Retrieves the current accelerometer range
            case C_STATE_ACC_RANGE:
                if (verbose) logd(TAG, "Get acc. range parameters\n");
                bt->getAccRange();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_MAG_RANGE;
                break;

            // Retrieves the current magnetometer range
            case C_STATE_MAG_RANGE:
                if (verbose) logd(TAG, "Get mag. range parameters\n");
                bt->getMagRange();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_IMU_ID;
                break;

            // Retrieves the current IMU ID
            case C_STATE_IMU_ID:
                if (verbose) logd(TAG, "Get IMU ID\n");
                bt->getImuId();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_HARD;
                break;

            // Retrieves hard iron matrix
            case C_STATE_GET_HARD:
                if (verbose) logd(TAG, "Get hard iron offset\n");
                bt->getHardIronOffset();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_SOFT;
                break;

            // Retrieves hard soft matrix
            case C_STATE_GET_SOFT:
                if (verbose) logd(TAG, "Get soft iron matrix\n");
                bt->getSoftIronMatrix();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_ESTIMATE;
                break;

            // Retrieves field estimate
            case C_STATE_GET_ESTIMATE:
                if (verbose) logd(TAG, "Get field estimate\n");
                bt->getFieldEstimate();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_MAG_ALIGNMENT_MATRIX;
                break;

            // Retrieves magnetometer alignment matrix
            case C_STATE_GET_MAG_ALIGNMENT_MATRIX:
                if (verbose) logd(TAG, "Get magnetometer alignment matrix\n");
                bt->getMagAlignmentMatrix();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_MAG_ALIGNMENT_BIAS;
                break;

            // Retrieves magnetometer alignment matrix
            case C_STATE_GET_MAG_ALIGNMENT_BIAS:
                if (verbose) logd(TAG, "Get magnetometer alignment bias\n");
                bt->getMagAlignmentBias();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_MAG_REFERENCE;
                break;

            // Retrieves magnetometer alignment matrix
            case C_STATE_GET_MAG_REFERENCE:
                if (verbose) logd(TAG, "Get magnetometer reference\n");
                bt->getMagReference();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_ACC_ALIGNMENT;
                break;

            // Retrieves accelerometer alignment matrix
            case C_STATE_GET_ACC_ALIGNMENT:
                if (verbose) logd(TAG, "Get acc. alignment matrix\n");
                bt->getAccAlignment();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_ACC_BIAS;
                break;

            // Retrieves accelerometer bias
            case C_STATE_GET_ACC_BIAS:
                if (verbose) logd(TAG, "Get acc. bias\n");
                bt->getAccBias();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_GYR_ALIGNMENT;
                break;

            // Retrieves gyroscope alignment matrix
            case C_STATE_GET_GYR_ALIGNMENT:
                if (verbose) logd(TAG, "Get gyr. alignment matrix\n");
                bt->getGyrAlignment();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_GYR_ALIGNMENT_BIAS;
                break;

            // Retrieves gyroscope alignment bias
            case C_STATE_GET_GYR_ALIGNMENT_BIAS:
                if (verbose) logd(TAG, "Get gyr. alignment bias\n");
                bt->getGyrAlignmentBias();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_LOW_PASS;
                break;

            // Retrieves low-pass filter settings
            case C_STATE_GET_LOW_PASS:
                if (verbose) logd(TAG, "Get low-pass filter settings\n");
                bt->getRawDataLpFilter();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_CAN_MAPPING;
                //getConfigState = C_STATE_GET_LIN_ACC_COMP_MODE;
                break;

            // Retrieves CAN bus mapping
            case C_STATE_GET_CAN_MAPPING:
                if (verbose) logd(TAG, "Get CANopen mapping\n");
                bt->getCanMapping();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_CAN_HEARTBEAT;
                break;

            // Retrieves CANopen heartbeat timing
            case C_STATE_GET_CAN_HEARTBEAT:
                if (verbose) logd(TAG, "Get CANopen heartbeat timing\n");
                bt->getCanHeartbeat();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_LIN_ACC_COMP_MODE;
                break;

            // Retrieves CANopen heartbeat timing
            case C_STATE_GET_LIN_ACC_COMP_MODE:
                if (verbose) if (verbose) logd(TAG, "Get linear acceleration compensation mode\n");
                bt->getLinAccCompMode();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_CENTRI_COMP_MODE;
                break;

            // Retrieves CANopen heartbeat timing
            case C_STATE_CENTRI_COMP_MODE:
                if (verbose) logd(TAG, "Get centripetal acceleration compensation mode\n");
                bt->getCentriCompMode();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_CAN_CONFIGURATION;

                //getConfigState = C_STATE_SETTINGS_DONE;
                break;

            // Retrieves CAN configuration
            case C_STATE_GET_CAN_CONFIGURATION:
                if (verbose) logd(TAG, "Get CAN configuration\n");
                bt->getCanConfiguration();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_GET_UART_BAUDRATE;
                break;

            case C_STATE_GET_UART_BAUDRATE:
                if (verbose) logd(TAG, "Get UART baud rate\n");
                bt->getUartBaudRate();
                state = STATE_GET_SETTINGS;
                getConfigState = C_STATE_SETTINGS_DONE;
                break;

            // Resets the timer and retrieves the field map (soft/hard iron calibration parameters)
            case C_STATE_SETTINGS_DONE:
                state = STATE_MEASURE;
                if (verbose) logd(TAG, "Done reading configuration\n");

                lpmsTimer.reset();
                statusTimer.reset();

                newFieldMap = true;
                bt->resetTsCount();
                bt->startStreaming();

                setConnectionStatus(SENSOR_CONNECTION_CONNECTED);
                setSensorStatus(SENSOR_STATUS_RUNNING);

                retrialsConnect = 0;
                retrialsCommandMode = 0;

                if (verbose)
                    configData.print();

                if (isFirmwareUpdated == true) {
                    loadCalibrationData(FIRMWARE_BACKUP_FILE);
                    isFirmwareUpdated = false; 
                    isSaveParametersToSensor = true;
                }
                else if (isSaveParametersToSensor)
                {
                    saveCalibrationData();
                    isSaveParametersToSensor = false;
                }
                break;
            }
        }

        break;

    // Main measurement state
    case STATE_MEASURE:
        assertConnected();

        // Start next measurement step only if program is not waiting for data or ACK
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            if (bt->getMode() != SELECT_LPMS_MODE_STREAM) {
                bt->setStreamMode();
                prepareStream = 0;
                break;
            }
        }

        // TODO: Insert error handling for sensor.
        // if (bt->isError() == true) {
        // setSensorStatus(SENSOR_STATUS_ERROR);
        // }

        if (paused == true) {
            break;
        }

        if (prepareStream < STREAM_N_PREPARE) {
            ++prepareStream;
            break;
        }

        // Load current data from hardware and calculate rotation matrix and Euler angle
        if (bt->getLatestImuData(&imuData) == false) break;

        frameTime = lpmsTimer.measure() / 1000.0f;
        lpmsTimer.reset();
        setFps(frameTime);

        convertArrayToLpVector4f(imuData.q, &q);
        quaternionToMatrix(&q, &m);
        convertLpMatrixToArray(&m, imuData.rotationM);

        // Add frame number timestamp and IMU ID to current ImuData
        ++frameNo;
        imuData.frameCount = frameNo;
        imuData.openMatId = configData.openMatId;

        setConnectionStatus(SENSOR_CONNECTION_CONNECTED);

        if (isMagCalibrationEnabled == true) {
            setSensorStatus(SENSOR_STATUS_CALIBRATING);
        }
        else {
            if (paused == false) {
                setSensorStatus(SENSOR_STATUS_RUNNING);
            }
            else {
                setSensorStatus(SENSOR_STATUS_PAUSED);
            }
        }

        convertArrayToLpVector3f(imuData.aRaw, &aRaw);
        convertArrayToLpVector3f(imuData.bRaw, &bRaw);
        convertArrayToLpVector3f(imuData.gRaw, &gRaw);

        // Corrects magnetometer measurement
        if ((bt->getConfigReg() & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
            vectSub3x1(&bRaw, &configData.hardIronOffset, &b);
            matVectMult3(&configData.softIronMatrix, &b, &b);
        }
        else {
            vectZero3x1(&b);
        }

        // Corrects accelerometer measurement
        if ((bt->getConfigReg() & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
            matVectMult3(&configData.misalignMatrix, &aRaw, &a);
            vectAdd3x1(&configData.accBias, &a, &a);
        }
        else {
            vectZero3x1(&a);
        }

        // Corrects gyro measurement
        if ((bt->getConfigReg() & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
            matVectMult3(&configData.gyrMisalignMatrix, &gRaw, &g);
            vectAdd3x1(&configData.gyrAlignmentBias, &g, &g);
        }
        else {
            vectZero3x1(&g);
        }

        convertLpVector3fToArray(&a, imuData.a);
        convertLpVector3fToArray(&b, imuData.b);
        convertLpVector3fToArray(&g, imuData.g);

        // Checks, if calibration is active
        checkMagCal(frameTime);
        checkPlanarMagCal(frameTime);
        checkMisalignCal(frameTime);
        checkGyrMisalignCal(frameTime);
        checkMagMisalignCal(frameTime);
        checkMagReferenceCal(frameTime);
        
        // Sets current datac
        setCurrentData(imuData);

        // Checks, if data saving is active
        checkSaveData();
        break;

    // Prepares parameter adjustment by switching to command mode
    case PREPARE_PARAMETER_ADJUSTMENT:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setCommandMode();
            state = getConfigState;
        }
        break;

    // Enables / disables gyroscope threshold
    case STATE_SET_CONFIG:
    case STATE_ENABLE_THRESHOLD:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_GYR_THRESHOLD_ENABLED, &p);
            switch (p) {
            case SELECT_IMU_GYR_THRESH_ENABLED:
                bt->enableGyrThres(LPMS_ENABLE_GYR_THRESHOLD);
                break;

            default:
                bt->enableGyrThres(LPMS_DISABLE_GYR_THRESHOLD);
                break;
            }
            if (verbose) logd(TAG, "Enable / disable threshold\n");
            state = STATE_GYR_AUTOCALIBRATION;
        }
        break;

    // Enables / disables gyroscope autocalibration
    case STATE_GYR_AUTOCALIBRATION:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_GYR_AUTOCALIBRATION, &p);
            switch (p) {
            case SELECT_GYR_AUTOCALIBRATION_ENABLED:
                bt->enableGyrAutocalibration(LPMS_ENABLE_GYR_AUTOCAL);
                break;

            default:
                bt->enableGyrAutocalibration(LPMS_DISABLE_GYR_AUTOCAL);
                break;
            }
            if (verbose) logd(TAG, "Gyroscope autocalibration on / off\n");
            state = STATE_SET_GYR_RANGE;
        }
        break;

    // Sets current gyroscope range
    case STATE_SET_GYR_RANGE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_GYR_RANGE, &p);
            bt->setGyrRange(p);
            if (verbose) logd(TAG, "Set gyroscope range\n");
            state = STATE_SET_ACC_RANGE;
        }
        break;

    // Sets accelerometer range
    case STATE_SET_ACC_RANGE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_ACC_RANGE, &p);
            bt->setAccRange(p);
            if (verbose) logd(TAG, "Set accelerometer range\n");
            state = STATE_SET_MAG_RANGE;
        }
        break;

    // Sets magnetometer range
    case STATE_SET_MAG_RANGE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_MAG_RANGE, &p);
            bt->setMagRange(p);
            if (verbose) logd(TAG, "Set magnetometer range\n");
            state = STATE_SET_FILTER_MODE;
        }
        break;

    // Sets current filter mode
    case STATE_SET_FILTER_MODE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_FILTER_MODE, &p);
            switch (p) {
            case SELECT_FM_GYRO_ONLY:
                bt->setFilterMode(LPMS_FILTER_GYR);
                break;

            case SELECT_FM_GYRO_ACC:
                bt->setFilterMode(LPMS_FILTER_GYR_ACC);
                break;

            case SELECT_FM_GYRO_ACC_MAG:
                bt->setFilterMode(LPMS_FILTER_GYR_ACC_MAG);
                break;

            case SELECT_FM_MADGWICK_GYRO_ACC:
                bt->setFilterMode(LPMS_FILTER_MADGWICK_GYR_ACC);
                break;

            case SELECT_FM_MADGWICK_GYRO_ACC_MAG:
                bt->setFilterMode(LPMS_FILTER_MADGWICK_GYR_ACC_MAG);
                break;
            }
            if (verbose) logd(TAG, "Set filter mode\n");
            state = STATE_SET_PARAMETER_SET;
        }
        break;

    // Sets current filter parameter set
    case STATE_SET_PARAMETER_SET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_PARAMETER_SET, &p);
            switch (p) {
            case SELECT_IMU_SLOW:
                bt->setFilterPreset(LPMS_FILTER_PRM_SET_1);
                break;

            case SELECT_IMU_MEDIUM:
                bt->setFilterPreset(LPMS_FILTER_PRM_SET_2);
                break;

            case SELECT_IMU_FAST:
                bt->setFilterPreset(LPMS_FILTER_PRM_SET_3);
                break;

            case SELECT_IMU_DYNAMIC:
                bt->setFilterPreset(LPMS_FILTER_PRM_SET_4);
                break;
            }
            if (verbose) logd(TAG, "Set parameter set\n");
            state = STATE_SET_LP_FILTER;
        }
        break;

    // Sets current low-pass filter setting
    case STATE_SET_LP_FILTER:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_LOW_PASS, &p);
            bt->setRawDataLpFilter(p);
            if (verbose) logd(TAG, "Set low-pass filter\n");
            state = STATE_SET_OPENMAT_ID;
        }
        break;

    // Sets OpenMAT ID
    case STATE_SET_OPENMAT_ID:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_OPENMAT_ID, &p);
            bt->setImuId(p);
            if (verbose) logd(TAG, "Set OpenMAT ID\n");
            //state = STATE_SET_CAN_PROTOCOL;// STATE_SET_CAN_BAUDRATE;
            state = STATE_SET_CAN_BAUDRATE;
        }
        break;

    // Sets CAN protocol
    case STATE_SET_CAN_PROTOCOL:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_CAN_STREAM_FORMAT, &p);
            bt->setCanStreamFormat(p);
            if (verbose) logd(TAG, "Set CAN protocol\n");
            state = STATE_SET_CAN_BAUDRATE;
        }
        break;

    // Sets CAN baudrate
    case STATE_SET_CAN_BAUDRATE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_CAN_BAUDRATE, &p);
            bt->setCanBaudrate(p);
            if (verbose) logd(TAG, "Set CAN baudrate\n");
            state = STATE_SET_SAMPLING_RATE;
        }
        break;

    // Sets sampling rate
    case STATE_SET_SAMPLING_RATE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_SAMPLING_RATE, &p);
            bt->setStreamFrequency(p);
            if (verbose) logd(TAG, "Set sampling rate\n");
            state = STATE_SET_HARD_IRON_OFFSET;
        }
        break;

    // Sets hard iron offset
    case STATE_SET_HARD_IRON_OFFSET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setHardIronOffset(configData.hardIronOffset);
            if (verbose) logd(TAG, "Set hard iron offset\n");
            state = STATE_SET_SOFT_IRON_MATRIX;
        }
        break;

    // Sets soft iron matrix
    case STATE_SET_SOFT_IRON_MATRIX:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setSoftIronMatrix(configData.softIronMatrix);
            if (verbose) logd(TAG, "Set soft iron matrix\n");
            state = STATE_SET_FIELD_ESTIMATE;
        }
        break;

    // Sets field estimate
    case STATE_SET_FIELD_ESTIMATE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setFieldEstimate(configData.fieldRadius);
            if (verbose) logd(TAG, "Set field estimate\n");
            state = STATE_SET_MAG_ALIGNMENT_MATRIX;
        }
        break;

    // Sets magnetometer alignment matrix
    case STATE_SET_MAG_ALIGNMENT_MATRIX:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setMagAlignmentMatrix(configData.magMAlignmentMatrix);
            if (verbose) logd(TAG, "Set magnetometer alignment matrix\n");
            state = STATE_SET_MAG_ALIGNMENT_BIAS;
        }
        break;

    // Sets magnetometer alignment bias
    case STATE_SET_MAG_ALIGNMENT_BIAS:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setMagAlignmentBias(configData.magMAlignmentBias);
            if (verbose) logd(TAG, "Set magnetometer alignment bias\n");
            state = STATE_SET_MAG_REFERENCE;
        }
        break;

    // Sets magnetometer reference
    case STATE_SET_MAG_REFERENCE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setMagReference(configData.magReference);
            if (verbose) logd(TAG, "Set magnetometer reference\n");
            state = STATE_SET_GYR_ALIGNMENT;
        }
        break;

    // Sets gyroscope alignment
    case STATE_SET_GYR_ALIGNMENT:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setGyrAlignment(configData.gyrMisalignMatrix);
            if (verbose) logd(TAG, "Set gyroscope alignment\n");
            state = STATE_SET_GYR_ALIGNMENT_BIAS;
        }
        break;

    // Sets gyroscope alignment matrix
    case STATE_SET_GYR_ALIGNMENT_BIAS:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setGyrAlignmentBias(configData.gyrAlignmentBias);
            if (verbose) logd(TAG, "Set gyroscope alignment bias\n");
            state = STATE_SET_ACC_ALIGNMENT;
        }
        break;

    // Sets accelerometer alignment
    case STATE_SET_ACC_ALIGNMENT:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setAccAlignment(configData.misalignMatrix);
            if (verbose) logd(TAG, "Set acclerometer alignment\n");
            state = STATE_SET_ACC_BIAS;
        }
        break;

    // Sets accelerometer bias
    case STATE_SET_ACC_BIAS:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setAccBias(configData.accBias);
            if (verbose) logd(TAG, "Set accelerometer bias\n");
            state = STATE_SET_CAN_MAPPING;
        }
        break;

    // Sets CANopen mapping
    case STATE_SET_CAN_MAPPING:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_CAN_MAPPING, pa);
            bt->setCanMapping(pa);
            if (verbose) logd(TAG, "Set CAN mapping\n");
            state = STATE_SET_CAN_HEARTBEAT;
        }
        break;

    // Sets CANopen heartbeat timing
    case STATE_SET_CAN_HEARTBEAT:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setCanHeartbeat(configData.canHeartbeat);
            if (verbose) logd(TAG, "Set CAN bus heartbeat timing\n");
            state = STATE_SET_LIN_ACC_COMP_MODE;
        }
        break;

    // Sets linear acceleration compensation mode
    case STATE_SET_LIN_ACC_COMP_MODE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_LIN_ACC_COMP_MODE, &p);
            switch (p) {
            case SELECT_LPMS_LIN_ACC_COMP_MODE_OFF:
                bt->setLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_OFF);
                break;

            case SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK:
                bt->setLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_WEAK);
                break;

            case SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM:
                bt->setLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_MEDIUM);
                break;

            case SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG:
                bt->setLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_STRONG);
                break;

            case SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA:
                bt->setLinAccCompMode(LPMS_LIN_ACC_COMP_MODE_ULTRA);
                break;
            }

            if (verbose) logd(TAG, "Set linear acceleration compensation mode\n");
            state = STATE_SET_CENTRI_COMP_MODE;
        }
        break;

    // Sets centripetal acceleration compensation mode
    case STATE_SET_CENTRI_COMP_MODE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setCentriCompMode(configData.centriCompMode);
            if (verbose) logd(TAG, "Set centripetal acceleration compensation mode\n");
            state = STATE_SET_CAN_CHANNEL_MODE;
        }
        break;

    // Sets CAN channel mode
    case STATE_SET_CAN_CHANNEL_MODE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_CAN_CHANNEL_MODE, &p);
            switch (p) {
            case SELECT_CAN_CHANNEL_MODE_CANOPEN:
                bt->setCanChannelMode(0);
                break;

            case SELECT_CAN_CHANNEL_MODE_SEQUENTIAL:
                bt->setCanChannelMode(LPMS_CAN_SEQUENTIAL_MODE);
                break;
            }
            if (verbose) logd(TAG, "Set CAN channel mode\n");
            state = STATE_SET_CAN_POINT_MODE;
        }
        break;

    // Sets CAN point mode
    case STATE_SET_CAN_POINT_MODE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_CAN_POINT_MODE, &p);
            switch (p) {
            case SELECT_CAN_POINT_MODE_FLOATING:
                bt->setCanPointMode(0);
                break;

            case SELECT_CAN_POINT_MODE_FIXED:
                bt->setCanPointMode(LPMS_CAN_FIXEDPOINT_MODE);
                break;
            }
            if (verbose) logd(TAG, "Set CAN point mode\n");
            state = STATE_SET_CAN_START_ID;
        }
        break;

    // Sets CAN point mode
    case STATE_SET_CAN_START_ID:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setCanStartId(configData.canStartId);
            if (verbose) logd(TAG, "Set CAN start ID\n");
            state = STATE_SET_LPBUS_DATA_MODE;
        }
        break;

    // Sets CAN point mode
    case STATE_SET_LPBUS_DATA_MODE:
        /* if (assertFwVersion(1, 3, 0) == false) {
            state = STATE_SELECT_DATA;
            break;
            } */

        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_LPBUS_DATA_MODE, &p);
            switch (p) {
            case SELECT_LPMS_LPBUS_DATA_MODE_32:
                bt->setLpBusDataMode(LPMS_LPBUS_DATA_MODE_32);
                break;

            case SELECT_LPMS_LPBUS_DATA_MODE_16:
                bt->setLpBusDataMode(LPMS_LPBUS_DATA_MODE_16);
                break;
            }
            if (verbose) logd(TAG, "Set LP-BUS data mode\n");
            state = STATE_SELECT_DATA;
        }
        break;

    // Selects transmission data
    case STATE_SELECT_DATA:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_SELECT_DATA, &p);
            bt->selectData(p);
            if (verbose) logd(TAG, "Select data 0x%x\n", p);
            state = STATE_SELECT_UART_BAUDRATE;
            //state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    case STATE_SELECT_UART_BAUDRATE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_UART_BAUDRATE, &p);

            switch (p) {
            case SELECT_LPMS_UART_BAUDRATE_19200:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_19200);
                break;

            case SELECT_LPMS_UART_BAUDRATE_38400:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_38400);
                break;

            case SELECT_LPMS_UART_BAUDRATE_57600:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_57600);
                break;

            case SELECT_LPMS_UART_BAUDRATE_115200:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_115200);
                break;

            case SELECT_LPMS_UART_BAUDRATE_230400:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_230400);
                break;

            case SELECT_LPMS_UART_BAUDRATE_256000:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_256000);
                break;

            case SELECT_LPMS_UART_BAUDRATE_460800:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_460800);
                break;

            case SELECT_LPMS_UART_BAUDRATE_921600:
                bt->setUartBaudRate(LPMS_UART_BAUDRATE_921600);
                break;
            }

            if (verbose) logd(TAG, "Select UART baud rate to mode %d\n", p);

            state = STATE_SELECT_UART_FORMAT;
        }
        break;

    case STATE_SELECT_UART_FORMAT:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_UART_FORMAT, &p);

            switch (p) {
            case SELECT_LPMS_UART_FORMAT_LPBUS:
                bt->setUartFormat(LPMS_UART_FORMAT_LPBUS);
                break;

            case SELECT_LPMS_UART_FORMAT_CSV:
                bt->setUartFormat(LPMS_UART_FORMAT_CSV);
                break;
            }

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    // Writes parameters to the sensor flash memory
    case STATE_WRITE_PARAMETERS:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->writeRegisters();
            setSensorStatus(SENSOR_STATUS_CALIBRATING);

            state = STATE_MEASURE;
        }
        break;

    // Starts uploading firmware
    case STATE_UPLOAD_FIRMWARE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->startUploadFirmware(firmwareFilename);
            state = STATE_CHECK_FIRMWARE_UPLOAD;
            setSensorStatus(SENSOR_STATUS_UPLOADING);
        }
        break;

    // Starts checking state of firmware upload
    case STATE_CHECK_FIRMWARE_UPLOAD:
        if (bt->getCurrentState() == UPDATE_FIRMWARE) {
        }
        else {
            state = STATE_WAIT_FIRMWARE_WRITE;
            lpmsTimer.reset();
        }
        break;

    // Waits for firmware writing to be finished (10s)
    case STATE_WAIT_FIRMWARE_WRITE:
        if (lpmsTimer.measure() > WAIT_FIRMWARE_WRITE_TIME) {
            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GOTO_COMMAND_MODE;
            if (paused == false) {
                isFirmwareUpdated = true;
                setSensorStatus(SENSOR_STATUS_RUNNING);
            }
            else {
                setSensorStatus(SENSOR_STATUS_PAUSED);
            }
        }
        break;

    // Starts uploading the IAP
    case STATE_UPLOAD_IAP:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->startUploadIap(iapFilename);
            state = STATE_CHECK_IAP_UPLOAD;
            setSensorStatus(SENSOR_STATUS_UPLOADING);
        }
        break;

    // Starts checking the status of the IAP upload
    case STATE_CHECK_IAP_UPLOAD:
        if (bt->getCurrentState() == UPDATE_IAP) {
        }
        else {
            state = STATE_WAIT_IAP_WRITE;
            lpmsTimer.reset();
        }
        break;

    // Waits for IAP writing to be finished
    case STATE_WAIT_IAP_WRITE:
        if (lpmsTimer.measure() > WAIT_IAP_WRITE_TIME) {
            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GOTO_COMMAND_MODE;
            if (paused == false) {
                setSensorStatus(SENSOR_STATUS_RUNNING);
            }
            else {
                setSensorStatus(SENSOR_STATUS_PAUSED);
            }
        }
        break;

    case STATE_GET_FLASH_LOG_SIZE:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->getFlashLogSize();
            state = STATE_GET_FLASH_LOG;
            setSensorStatus(SENSOR_STATUS_UPLOADING);
        }
        break;

    case STATE_GET_FLASH_LOG:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->getFlashLog(flashLogFilename);
            state = STATE_CHECK_GET_FLASH_LOG;
            setSensorStatus(SENSOR_STATUS_UPLOADING);
        }
        break;

    case STATE_CHECK_GET_FLASH_LOG:
        if (bt->getCurrentState() == GET_FLASH_LOG) {
        }
        else {
            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GOTO_COMMAND_MODE;
            setSensorStatus(SENSOR_STATUS_RUNNING);
            lpmsTimer.reset();
        }
        break;

    // Initiates self-test
    case STATE_SET_SELF_TEST:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            configData.getParameter(PRM_SELF_TEST, &p);
            bt->setSelfTest(p);
            state = STATE_MEASURE;
        }
        break;

    // Retrieves communication latency
    case STATE_GET_LATENCY:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            setSensorStatus(SENSOR_STATUS_CALIBRATING);

            bt->getConfig();

            if (latencyCounter == 0) {
                accLatency = 0.0f;
            }
            else if (latencyCounter > 0 && latencyCounter < 50) {
                accLatency += bt->getLatestLatency();
            }
            else {
                avgLatency = accLatency / (float)(latencyCounter - 1) / 2.0f;
                std::cout << "Average latency: " << avgLatency << "ms" << std::endl;
                state = STATE_MEASURE;
            }

            latencyCounter++;
        }
        break;

    // Starts gyroscope calibration
    case STATE_CALIBRATE_GYRO:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->startGyrCalibration();

            state = STATE_CALIBRATING;
            getConfigState = CAL_STATE_GET_STATUS;
        }
        break;

    // Waits for calibration to finish
    case STATE_CALIBRATING:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            switch (getConfigState) {
            case CAL_STATE_GET_STATUS:
                bt->getStatus();

                setSensorStatus(SENSOR_STATUS_CALIBRATING);

                getConfigState = CAL_STATE_WAIT_FINISH;
                break;

            case CAL_STATE_WAIT_FINISH:
                if (bt->isCalibrating() == false) {
                    state = STATE_MEASURE;
                }
                else if (statusTimer.measure() > STATUS_PERIOD) {
                    bt->getStatus();
                    statusTimer.reset();
                }
            }
        }
        break;

    // Resets offset
    case STATE_RESET_ORIENTATION_OFFSET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->resetOrientationOffset();
            state = STATE_SET_CONFIG;
        }
        break;

    // Sets offset
    case STATE_SET_ORIENTATION_OFFSET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setOrientationOffset(currentOffsetResetMethod);
            state = STATE_SET_CONFIG;
        }
        break;

    // Restores factory defaults
    case STATE_RESET_TO_FACTORY_DEFAULTS:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->restoreFactoryValue();
            if (verbose) logd(TAG, "Reset factory defaults\n");

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    // Restores factory defaults
    case STATE_HARD_RESET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->hardReset();
            if (verbose) logd(TAG, "Hard reset\n");

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    // Resets sensor timestamp
    case STATE_RESET_TIMESTAMP:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->setTimestamp(0);
            if (verbose) logd(TAG, "Reset sensor timestamp\n");

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    case STATE_ARM_TIMESTAMP_RESET:
        if (bt->isWaitForData() == false && bt->isWaitForAck() == false) {
            bt->armTimestampReset(LPMS_ARM_TIMESTAMP_RESET);
            if (verbose) logd(TAG, "Arm hardware timestamp reset\n");

            state = STATE_GET_SETTINGS;
            getConfigState = C_STATE_GET_CONFIG;
        }
        break;

    // Error state
    case STATE_NONE:
        state = STATE_NONE;
        break;

    default:
        break;
    }
}

void LpmsSensor::getDeviceId(char *str)
{
    std::string deviceId;

    configData.getParameter(PRM_DEVICE_ID, &deviceId);

    strcpy(str, deviceId.c_str());
}

CalibrationData *LpmsSensor::getConfigurationData(void)
{
    return &configData;
}

bool LpmsSensor::assertFwVersion(int d0, int d1, int d2)
{
    if (configData.firmwareVersionDig0 > d0) return true;
    if (configData.firmwareVersionDig0 == d0 && configData.firmwareVersionDig1 > d1) return true;
    if (configData.firmwareVersionDig0 == d0 && configData.firmwareVersionDig1 == d1 && configData.firmwareVersionDig2 >= d2) return true;

    return false;
}

bool LpmsSensor::hasNewFieldMap(void)
{
    bool f = newFieldMap;
    newFieldMap = false;

    return f;
}

void LpmsSensor::setFps(float f)
{
    currentFps = f;
}

float LpmsSensor::getFps(void)
{
    return currentFps;
}

void LpmsSensor::setSensorStatus(int s)
{
    sensorStatus = s;
}

int LpmsSensor::getSensorStatus(void)
{
    return sensorStatus;
}

void LpmsSensor::setConnectionStatus(int s)
{
    connectionStatus = s;
}

int LpmsSensor::getConnectionStatus(void)
{
    return connectionStatus;
}

void LpmsSensor::setCurrentData(ImuData d)
{
    std::unique_lock<std::mutex> lock(sensorMutex);

    currentData = d;

    if (dataQueue.size() < dataQueueLength) {
        dataQueue.push(d);
    }
    else {
        dataQueue.pop();
        dataQueue.push(d);
    }

    if (lpmsCallback) {
        lpmsCallback(d, deviceId.c_str());
    }

    newDataCondition.notify_one();
}

void LpmsSensor::setCallback(LpmsCallback cb)
{
    lpmsCallback = cb;
}

int LpmsSensor::hasImuData(void)
{
    return dataQueue.size();
}

ImuData LpmsSensor::getCurrentData(void)
{
    ImuData d;

    bt->zeroImuData(&d);

    sensorMutex.lock();

    if (dataQueue.size() > 0) {
        d = dataQueue.front();
        dataQueue.pop();
    }
    else {
        d = currentData;
    }

    sensorMutex.unlock();

    return d;
}

bool LpmsSensor::getNextData(ImuData& d, double timeOut = 0)
{
    bt->zeroImuData(&d);

    std::unique_lock<std::mutex> lock(sensorMutex);

    // If no data is available, we block until data becomes available or until we time out.
    auto pred = [&]() -> bool { return dataQueue.size() > 0; };
    if (!pred()) {
        if (timeOut <= 0) {
            newDataCondition.wait(lock, pred);
        } else {
           newDataCondition.wait_for(lock, std::chrono::nanoseconds(long long(timeOut * 1e9)));
        }
    }

    // If data became available before timeout, return it.
    if (pred()) {
        d = dataQueue.front();
        dataQueue.pop();
        return true;
    }

    return false;
}

void LpmsSensor::getCalibratedSensorData(float g[3], float a[3], float b[3])
{
    sensorMutex.lock();
    for (int i = 0; i < 3; i++) g[i] = currentData.g[i];
    for (int i = 0; i < 3; i++) a[i] = currentData.a[i];
    for (int i = 0; i < 3; i++) b[i] = currentData.b[i];
    sensorMutex.unlock();
}

void LpmsSensor::getQuaternion(float q[4])
{
    sensorMutex.lock();
    for (int i = 0; i < 4; i++) q[i] = currentData.q[i];
    sensorMutex.unlock();
}

void LpmsSensor::getEulerAngle(float r[3])
{
    sensorMutex.lock();
    for (int i = 0; i < 3; i++) r[i] = currentData.r[i];
    sensorMutex.unlock();
}

void LpmsSensor::getRotationMatrix(float M[3][3])
{
    sensorMutex.lock();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            M[i][j] = currentData.rotationM[i * 3 + j];
        }
    }
    sensorMutex.unlock();
}

void LpmsSensor::getGyroData(float g[3])
{
    sensorMutex.lock();
    for (int i = 0; i < 3; i++) g[i] = currentData.g[i];
    sensorMutex.unlock();
}

void LpmsSensor::getAccelerometerData(float a[3])
{
    sensorMutex.lock();
    for (int i = 0; i < 3; i++) a[i] = currentData.a[i];
    sensorMutex.unlock();
}

bool LpmsSensor::isRunning(void)
{
    return !paused;
}

void LpmsSensor::pause(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    setSensorStatus(SENSOR_STATUS_PAUSED);
    paused = true;
}

void LpmsSensor::run(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    setSensorStatus(SENSOR_STATUS_RUNNING);
    paused = false;
}

void LpmsSensor::close(void)
{
    stopped = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    bt->close();
}

void LpmsSensor::startCalibrateGyro(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
    if (state != STATE_MEASURE) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_CALIBRATE_GYRO;
}

void LpmsSensor::setOrientationOffset(int v)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
    if (state != STATE_MEASURE) return;

    currentOffsetResetMethod = v;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_SET_ORIENTATION_OFFSET;
}

void LpmsSensor::resetOrientationOffset(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;
    if (state != STATE_MEASURE) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_RESET_ORIENTATION_OFFSET;
}

void LpmsSensor::setOpenMatId(int id)
{
    configData.openMatId = id;
}

int LpmsSensor::getOpenMatId(void)
{
    return configData.openMatId;
}

bool LpmsSensor::updateParameters(void)
{
    bool r = true;

    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;

    if (state != STATE_MEASURE) {
        return false;
    }

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_SET_CONFIG;

    return r;
}

bool LpmsSensor::setConfigurationPrm(int parameterIndex, int parameter)
{
    bool f = true;

    configData.setParameter(parameterIndex, parameter);

    switch (parameterIndex) {
    case PRM_SELF_TEST:
        state = PREPARE_PARAMETER_ADJUSTMENT;
        getConfigState = STATE_SET_SELF_TEST;
        break;

    default:
        f = updateParameters();
        break;
    }

    return f;
}

bool LpmsSensor::setConfigurationPrm(int parameterIndex, int *parameter)
{
    bool f = true;

    configData.setParameter(parameterIndex, parameter);

    f = updateParameters();

    return f;
}

bool LpmsSensor::getConfigurationPrm(int parameterIndex, int* parameter)
{
    return configData.getParameter(parameterIndex, parameter);
}

bool LpmsSensor::getConfigurationPrm(int parameterIndex, char* parameter)
{
    std::string cppStr;

    const bool ret = configData.getParameter(parameterIndex, &cppStr);

    strcpy(parameter, cppStr.c_str());

    return ret;
}

float LpmsSensor::getBatteryLevel(void)
{
    return bt->getBatteryLevel();
}

float LpmsSensor::getBatteryVoltage(void)
{
    return bt->getBatteryVoltage();
}

int LpmsSensor::getChargingStatus(void)
{
    return bt->getChargingStatus();
}

std::string LpmsSensor::getDeviceName(void)
{
    return bt->getDeviceName();
}

std::string LpmsSensor::getFirmwareInfo(void)
{
    return bt->getFirmwareInfo();
}

void LpmsSensor::startSync()
{
    bt->startSync();
}

void LpmsSensor::stopSync()
{
    bt->stopSync();
}

void LpmsSensor::getGyroStaticBias()
{
    bt->getGyroStaticBias();
}

bool LpmsSensor::uploadFirmware(const char *fn)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;
#ifdef __WIN32
    WIN32_FIND_DATA FindFileData;
    HANDLE handle = FindFirstFile(fn, &FindFileData);
    int found = handle != INVALID_HANDLE_VALUE;
    if (found)
    {
        FindClose(handle);

        saveCalibrationData(FIRMWARE_BACKUP_FILE);

        state = PREPARE_PARAMETER_ADJUSTMENT;
        getConfigState = STATE_UPLOAD_FIRMWARE;
        firmwareFilename = fn;

        return true;
    }
    return false;
#else
    saveCalibrationData(FIRMWARE_BACKUP_FILE);
    
    state = PREPARE_PARAMETER_ADJUSTMENT;   
    getConfigState = STATE_UPLOAD_FIRMWARE;
    firmwareFilename = fn;
    
    return true;
#endif
}

bool LpmsSensor::uploadIap(const char *fn)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;

#ifdef __WIN32
    WIN32_FIND_DATA FindFileData;
    HANDLE handle = FindFirstFile(fn, &FindFileData);
    int found = handle != INVALID_HANDLE_VALUE;
    if (found)
    {
        FindClose(handle);

        state = PREPARE_PARAMETER_ADJUSTMENT;
        getConfigState = STATE_UPLOAD_IAP;
        iapFilename = fn;

        return true;
    }
    return false;
#else
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;

    state = PREPARE_PARAMETER_ADJUSTMENT;   
    getConfigState = STATE_UPLOAD_IAP;
    iapFilename = fn;
    
    return true;
#endif
}

int LpmsSensor::getUploadProgress(int *p)
{
    if (state != STATE_CHECK_IAP_UPLOAD &&
        state != STATE_WAIT_IAP_WRITE &&
        state != STATE_UPLOAD_IAP &&
        state != STATE_CHECK_FIRMWARE_UPLOAD &&
        state != STATE_WAIT_FIRMWARE_WRITE &&
        state != STATE_UPLOAD_FIRMWARE) {
        return 0;
    }

    if (bt->getUploadProgress(p) == false) return 2;

    if (state == STATE_WAIT_FIRMWARE_WRITE) {
        *p = *p + (int)(lpmsTimer.measure() * 100 / WAIT_FIRMWARE_WRITE_TIME);
    }

    if (state == STATE_WAIT_IAP_WRITE) {
        *p = *p + (int)(lpmsTimer.measure() * 100 / WAIT_IAP_WRITE_TIME);
    }

    return 1;
}

void LpmsSensor::saveCalibrationData(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_WRITE_PARAMETERS;
}

LpmsIoInterface *LpmsSensor::getIoInterface(void)
{
    return (LpmsIoInterface *)bt;
}

void LpmsSensor::measureAvgLatency(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    latencyCounter = 0;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_GET_LATENCY;
}

void LpmsSensor::acquireFieldMap(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_START_GET_FIELD_MAP;
}

bool LpmsSensor::getPressure(float *p)
{
    sensorMutex.lock();
    *p = currentData.pressure;
    sensorMutex.unlock();

    return true;
}

void LpmsSensor::getHardIronOffset(float v[3])
{
    for (int i = 0; i < 3; i++) {
        v[i] = configData.hardIronOffset.data[i];
    }
}

void LpmsSensor::getSoftIronMatrix(float M[3][3], float *fieldRadius)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            M[i][j] = configData.softIronMatrix.data[i][j];
        }
    }

    *fieldRadius = configData.fieldRadius;
}

float LpmsSensor::getFieldNoise(void)
{
    return fabs(configData.fieldRadius - sqrtf(b.data[0] * b.data[0] + b.data[1] * b.data[1] + b.data[2] * b.data[2]));
}

void LpmsSensor::getFieldMap(float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3]) {
    for (int i = 0; i < ABSMAXPITCH; i++) {
        for (int j = 0; j < ABSMAXROLL; j++) {
            for (int k = 0; k < ABSMAXYAW; k++) {
                for (int l = 0; l < 3; l++) {
                    fieldMap[i][j][k][l] = configData.fieldMap[i][j][k].data[l];
                }
            }
        }
    }
}

void LpmsSensor::zeroFieldMap(void)
{
    for (int i = 0; i < ABSMAXPITCH; i++) {
        for (int j = 0; j < ABSMAXROLL; j++) {
            for (int k = 0; k < ABSMAXYAW; k++) {
                for (int l = 0; l < 3; l++) {
                    configData.fieldMap[i][j][k].data[l] = 0.0f;
                }
            }
        }
    }
}

void LpmsSensor::resetToFactorySettings(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_RESET_TO_FACTORY_DEFAULTS;
}

void LpmsSensor::hardReset(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_HARD_RESET;
}

long LpmsSensor::getStreamFrequency(void)
{
    int i;

    getConfigurationPrm(PRM_SAMPLING_RATE, &i);
    return i;
}

void LpmsSensor::armTimestampReset(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_ARM_TIMESTAMP_RESET;
}

void LpmsSensor::setUploadPageSize(int size)
{
    bt->setUploadPageSize(size);
}

void LpmsSensor::startPlanarMagCalibration(void)
{
    int p = 0;

    if (isPlanarMagCalibrationEnabled == true) return;

    isPlanarMagCalibrationEnabled = true;
    magCalibrationDuration = 0.0f;

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
    p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;

    setConfigurationPrm(PRM_SELECT_DATA, p);

    for (int i = 0; i < 3; i++) {
        bMax.data[i] = -1.0e4f;
        bMin.data[i] = 1.0e4f;
        magAvg.data[i] = 0.0f;
    }

    runOnce = true;
}

void LpmsSensor::checkPlanarMagCal(float T)
{
    if (isPlanarMagCalibrationEnabled == true) {
        magCalibrationDuration += T;

        if (runOnce == true) {
            for (int i = 0; i < 3; i++) magAvg.data[i] = currentData.bRaw[i];
            runOnce = false;
        }

        for (int i = 0; i < 3; i++) magAvg.data[i] = magAvg.data[i] * 0.9f + currentData.bRaw[i] * 0.1f;

        for (int i = 0; i < 3; i++) {
            if (currentData.bRaw[i] > bMax.data[i]) {
                bMax.data[i] = magAvg.data[i];
                if (verbose) logd(TAG, "New maximum detected: Axis=%d, field=%f\n", i, currentData.bRaw[i]);
            }

            if (currentData.bRaw[i] < bMin.data[i]) {
                bMin.data[i] = magAvg.data[i];
                if (verbose) logd(TAG, "New minimum detected: Axis=%d, field=%f\n", i, currentData.bRaw[i]);
            }
        }

        if (magCalibrationDuration >= LPMS_MAG_CALIBRATION_DURATION_20S) {
            stopPlanarMagCalibration();
        }
    }
}

void LpmsSensor::stopPlanarMagCalibration(void)
{
    int i;
    float sqSum = 0;
    LpVector3f bBias;
    LpVector3f bRadius;

    if (isPlanarMagCalibrationEnabled == false) return;

    for (i = 0; i < 3; i++) {
        bBias.data[i] = (bMax.data[i] + bMin.data[i]) / 2.0f;
        if (verbose) logd(TAG, "Axis=%d, min=%f, max=%f\n", i, bMin.data[i], bMax.data[i]);
        if (verbose) logd(TAG, "Calculated bias: Axis=%d, field=%f\n", i, bBias.data[i]);

    }

    for (i = 0; i < 3; i++) {
        bRadius.data[i] = bMax.data[i] - bBias.data[i];
        sqSum += bRadius.data[i];
    }

    configData.fieldRadius = sqSum / 3;

    matZero3x3(&configData.softIronMatrix);
    for (int i = 0; i < 3; i++) {
        configData.softIronMatrix.data[i][i] = configData.fieldRadius / bRadius.data[i];
        if (verbose) logd(TAG, "Calculated radius: Axis=%d, field=%f\n", i, configData.softIronMatrix.data[i][i]);
        configData.hardIronOffset.data[i] = bBias.data[i];
    }

    newFieldMap = true;
    isPlanarMagCalibrationEnabled = false;
    magCalibrationDuration = 0.0f;

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);

    updateParameters();
}

void LpmsSensor::startMagCalibration(void)
{
    int p = 0;

    if (isMagCalibrationEnabled == true) return;

    isMagCalibrationEnabled = true;
    magCalibrationDuration = 0.0f;

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
    p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
    p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();

    bCalInitEllipsoidFit();
}

void LpmsSensor::checkMagCal(float T)
{
    float bInc;
    LpVector3f tV, tV2;

    convertArrayToLpVector3f(currentData.bRaw, &tV);
    bCalOrientationFromAccMag(tV, a, &tV2, &bInc);

    if (isMagCalibrationEnabled == true) {
        magCalibrationDuration += T;

        bCalUpdateBMap(tV2, tV);

        if (magCalibrationDuration >= LPMS_MAG_CALIBRATION_DURATION_20S) {
            stopMagCalibration();
        }
    }
}

void LpmsSensor::stopMagCalibration(void)
{
    if (isMagCalibrationEnabled == false) return;

    if (bCalFitEllipsoid() == 1) {
        configData.softIronMatrix = bCalGetSoftIronMatrix();
        configData.hardIronOffset = bCalGetHardIronOffset();
        configData.fieldRadius = bCalGetFieldRadius();
    }

    for (int i = 0; i < ABSMAXPITCH; i++) {
        for (int j = 0; j < ABSMAXROLL; j++) {
            for (int k = 0; k < ABSMAXYAW; k++) {
                for (int l = 0; l < 3; l++) {
                    configData.fieldMap[i][j][k].data[l] = bCalGetFieldMapElement(i, j, k, l);
                }
            }
        }
    }

    newFieldMap = true;
    isMagCalibrationEnabled = false;
    magCalibrationDuration = 0.0f;

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);

    updateParameters();
}

void LpmsSensor::initMisalignCal(void)
{
    int p = 0;

    if (isGetMisalign == true) return;

    isGetMisalign = false;
    misalignSetIndex = 0;
    misalignSamples = 0;
    misalignTime = 0.0f;

    for (int i = 0; i < N_ALIGNMENT_SETS; i++) {
        vectZero3x1(&misalignAData[i]);
        vectZero3x1(&misalignBData[i]);
        vectZero3x1(&misalignADataAcc);
    }

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();
}

void LpmsSensor::startGetMisalign(int i)
{
    if (i < N_ALIGNMENT_SETS) {
        isGetMisalign = true;
        misalignSetIndex = i;
        misalignSamples = 0;
        misalignTime = 0.0f;

        vectZero3x1(&misalignADataAcc);
    }
}

void LpmsSensor::checkMisalignCal(float T)
{
    if (isGetMisalign == true) {
        for (int i = 0; i < 3; i++) {
            misalignADataAcc.data[i] += aRaw.data[i];
        }

        ++misalignSamples;

        for (int i = 0; i < 3; i++) {
            if (aRaw.data[i] > 0.5) {
                misalignBData[misalignSetIndex].data[i] = 1.0f;
            }
            else if (aRaw.data[i] < -0.5) {
                misalignBData[misalignSetIndex].data[i] = -1.0f;
            }
            else {
                misalignBData[misalignSetIndex].data[i] = 0.0f;
            }
        }

        misalignTime += T;
        if (misalignTime > CALC_ACC_MA_DURATION) {
            isGetMisalign = false;

            if (verbose) logd(TAG, "Average acc. vector %d: ", misalignSetIndex);

            for (int i = 0; i < 3; i++) {
                if (misalignSamples == 0) break;

                misalignAData[misalignSetIndex].data[i] = misalignADataAcc.data[i] / misalignSamples;

                if (verbose)  printf("%f ", misalignAData[misalignSetIndex].data[i]);
            }
            if (verbose) printf("\n");

            vectZero3x1(&misalignADataAcc);

            misalignSamples = 0;
            misalignTime = 0.0f;
        }
    }
}

void LpmsSensor::calcMisalignMatrix(void)
{
    float *aTest[N_ALIGNMENT_SETS];
    float *bTest[N_ALIGNMENT_SETS];

    for (int i = 0; i < N_ALIGNMENT_SETS; i++) {
        aTest[i] = misalignAData[i].data;
        bTest[i] = misalignBData[i].data;
    }

    maCalCalcMisalignment(aTest, bTest, &configData.misalignMatrix, &configData.accBias, 6);

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);
    updateParameters();
}

void LpmsSensor::stopMisalignCal(void)
{
    isGetMisalign = false;
    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);
    updateParameters();
}

void LpmsSensor::saveCalibrationData(const char* fn)
{
    if (verbose) logd(TAG, "Saving calibration data to %s\n", fn);
    configData.save(fn);
}

bool LpmsSensor::loadCalibrationData(const char* fn)
{
    if (verbose) logd(TAG, "Loading calibration data from %s\n", fn);
    if (!configData.load(fn))
        return false;
    return updateParameters();
}

void LpmsSensor::initGyrMisalignCal(void)
{
    int p = 0;

    if (isGetGyrMisalign == true) return;

    isGetGyrMisalign = false;
    misalignSetIndex = 0;
    misalignSamples = 0;
    misalignTime = 0.0f;

    for (int i = 0; i < N_ALIGNMENT_SETS; i++) {
        vectZero3x1(&gyrMisalignAData[i]);
        vectZero3x1(&gyrMisalignBData[i]);
        vectZero3x1(&gyrMisalignADataAcc);
    }

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_GYRO_OUTPUT_ENABLED;
    p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();
}

void LpmsSensor::startGetGyrMisalign(int i)
{
    if (i < N_ALIGNMENT_SETS) {
        isGetGyrMisalign = true;
        misalignSetIndex = i;
        misalignSamples = 0;
        misalignTime = 0.0f;

        vectZero3x1(&gyrMisalignADataAcc);
    }
}

void LpmsSensor::checkGyrMisalignCal(float T)
{
    if (isGetGyrMisalign == true) {
        for (int i = 0; i < 3; i++) {
            gyrMisalignADataAcc.data[i] += gRaw.data[i];
        }

        ++misalignSamples;
        misalignTime += T;

        if (misalignTime >= CALC_GYR_MA_DURATION) {
            isGetGyrMisalign = false;

            if (verbose) logd(TAG, "Accumulated gyr. vector %d: \n", misalignSetIndex);

            for (int i = 0; i < 3; i++) {
                if (misalignSamples == 0) break;

                gyrMisalignAData[misalignSetIndex].data[i] = gyrMisalignADataAcc.data[i] / misalignSamples;
            }

            for (int i = 0; i < 3; i++) {
                if (gyrMisalignAData[misalignSetIndex].data[i] > 50.0f) {
                    gyrMisalignBData[misalignSetIndex].data[i] = 270.0f;
                }
                else if (gyrMisalignAData[misalignSetIndex].data[i] < -50.0f) {
                    gyrMisalignBData[misalignSetIndex].data[i] = -270.0f;
                }
                else {
                    gyrMisalignBData[misalignSetIndex].data[i] = 0.0f;
                }
                if (verbose) printf("%f ", gyrMisalignAData[misalignSetIndex].data[i]);
            }
            if (verbose) printf("\n");

            vectZero3x1(&gyrMisalignADataAcc);

            misalignSamples = 0;
            misalignTime = 0.0f;
        }
    }
}

void LpmsSensor::calcGyrMisalignMatrix(void)
{
    const float r2d = 57.2958f;
    const float d2r = 0.01745f;

    float *aTest[N_ALIGNMENT_SETS];
    float *bTest[N_ALIGNMENT_SETS];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            gyrMisalignAData[3 + i].data[j] = -gyrMisalignAData[i].data[j];
            gyrMisalignBData[3 + i].data[j] = -gyrMisalignBData[i].data[j];
        }
    }

    for (int i = 0; i < N_ALIGNMENT_SETS; i++) {
        aTest[i] = gyrMisalignAData[i].data;
        bTest[i] = gyrMisalignBData[i].data;
    }

    maCalCalcGyrMisalignment(aTest, bTest, &configData.gyrMisalignMatrix, &configData.gyrAlignmentBias, 6);

    for (int i = 0; i < 3; i++) {
        configData.gyrAlignmentBias.data[i] = configData.gyrAlignmentBias.data[i] * d2r;
    }

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);
    updateParameters();
}

void LpmsSensor::stopGyrMisalignCal(void)
{
    isGetGyrMisalign = false;
    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);
    updateParameters();
}

void LpmsSensor::resetTimestamp(void)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return;

    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_RESET_TIMESTAMP;

    frameNo = 0;
}

void LpmsSensor::setTimestamp(float t)
{
    int ts = int(t * 400);
    bt->setTimestamp(ts);
}

void LpmsSensor::startAutoMagMisalignCal(void)
{
    int p = 0;

    if (isAutoMagMisalignCalEnabled == true) return;

    isAutoMagMisalignCalEnabled = true;
    misalignTime = 0.0f;

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
    p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();

    if (verbose) logd(TAG, "Starting magnetometer misalignment calibration.\n");

    bMACalInitEllipsoidFit();
}

void LpmsSensor::checkAutoMagMisalignCal(float T)
{
    LpVector3f tV, bR;
    LpVector4f tV2;

    bR.data[0] = 0.0f;
    bR.data[1] = -1.0f;
    bR.data[2] = -1.0f;

    convertArrayToLpVector3f(currentData.b, &tV);
    convertArrayToLpVector4f(currentData.q, &tV2);

    if (isAutoMagMisalignCalEnabled == true) {
        misalignTime += T;

        bMAUpdateMap(tV2, tV, configData.magReference);

        if (misalignTime >= LPMS_REF_CALIBRATION_DURATION_20S) {
            calcMagMisalignCal();
        }

        logd(TAG, "b: %f, %f, %f\n", tV.data[0], tV.data[1], tV.data[2]);
        logd(TAG, "q: %f, %f, %f, %f\n", tV2.data[0], tV2.data[1], tV2.data[2], tV2.data[3]);
        logd(TAG, "T: %f\n", misalignTime);
    }
}

void LpmsSensor::calcAutoMagMisalignCal(void)
{
    LpMatrix3x3f R;
    LpVector3f t;

    if (isAutoMagMisalignCalEnabled == false) return;

    if (bMACalFitEllipsoid(&R, &t) == 1) {
        configData.magMAlignmentMatrix = R;
        configData.magMAlignmentBias = t;
    }

    isAutoMagMisalignCalEnabled = false;
    misalignTime = 0.0f;

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);

    updateParameters();
}

void LpmsSensor::startMagReferenceCal(void)
{
    int p = 0;

    if (isRefCalibrationEnabled == true) return;

    isRefCalibrationEnabled = true;

    cumulatedRefCounter = 0;
    refCalibrationDuration = 0.0f;

    for (int i = 0; i < 3; i++) cumulatedRefData[i] = 0;

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_MAG_OUTPUT_ENABLED;
    p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
    p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;

    logd(TAG, "Starting magnetometer reference calibration.\n");

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();
}

void LpmsSensor::checkMagReferenceCal(float T)
{
    float bInc;
    LpVector3f tR, tV, tV2;

    if (isRefCalibrationEnabled == true) {
        refCalibrationDuration += T;

        cumulatedRefCounter++;

        convertArrayToLpVector3f(currentData.b, &tV);
        convertArrayToLpVector3f(currentData.a, &tV2);
        if (verbose)
        {
            logd(TAG, "b: %f, %f, %f\n", tV.data[0], tV.data[1], tV.data[2]);
            logd(TAG, "a: %f, %f, %f\n", tV2.data[0], tV2.data[1], tV2.data[2]);
            logd(TAG, "T: %f\n", refCalibrationDuration);
        }
        getReferenceYZ(tV, tV2, &tR, &bInc);

        cumulatedRefData[0] = cumulatedRefData[0] + bInc;
        cumulatedRefData[1] = cumulatedRefData[1] + tR.data[1];
        cumulatedRefData[2] = cumulatedRefData[2] + tR.data[2];

        if (verbose)
        {
            logd(TAG, "ref: %f, %f, %f\n", cumulatedRefData[0], cumulatedRefData[1], cumulatedRefData[2]);
            logd(TAG, "Calibrating magnetometer reference..\n");
        }
        if (refCalibrationDuration >= LPMS_REF_CALIBRATION_DURATION_1S) {
            calcMagReferenceCal();
        }
    }
}

void LpmsSensor::calcMagReferenceCal(void)
{
    float n;

    if (isRefCalibrationEnabled == false) return;

    configData.magReference.data[0] = 0.0f;
    configData.magReference.data[1] = cumulatedRefData[1] / (float)cumulatedRefCounter;
    configData.magReference.data[2] = cumulatedRefData[2] / (float)cumulatedRefCounter;

    n = vect3x1Norm(configData.magReference);
    scalarVectMult3x1(n, &configData.magReference, &configData.magReference);
    if (verbose)
    {
        logd(TAG, "Magnetometer reference: %f, %f, %f\n", 0.0f, configData.magReference.data[1], configData.magReference.data[2]);
        logd(TAG, "Magnetometer reference length: %f\n", n);
    }
    isRefCalibrationEnabled = false;

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);

    updateParameters();
}

void LpmsSensor::initMagMisalignCal(void)
{
    int p;

    if (isMagMisalignCalEnabled == true) return;

    isMagMisalignCalEnabled = false;
    misalignSetIndex = 0;
    misalignSamples = 0;
    misalignTime = 0.0f;

    for (int i = 0; i < N_MAG_ALIGNMENT_SETS; i++) {
        vectZero3x1(&misalignAData[i]);
        vectZero3x1(&misalignBData[i]);
        vectZero3x1(&misalignADataAcc);
    }

    configData.getParameter(PRM_SELECT_DATA, &prevDataSelection);

    p = SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    p |= SELECT_LPMS_MAG_OUTPUT_ENABLED;

    configData.setParameter(PRM_SELECT_DATA, p);
    updateParameters();
}

void LpmsSensor::startMagMisalignCal(int i)
{
    if (i < N_MAG_ALIGNMENT_SETS) {
        isMagMisalignCalEnabled = true;
        misalignSetIndex = i;
        misalignSamples = 0;
        misalignTime = 0.0f;

        vectZero3x1(&misalignADataAcc);
    }
}

void LpmsSensor::checkMagMisalignCal(float T)
{
    if (isMagMisalignCalEnabled == true) {
        for (int i = 0; i < 3; i++) {
            misalignADataAcc.data[i] += bRaw.data[i];
        }

        ++misalignSamples;

        for (int i = 0; i < 3; i++) {
            if (aRaw.data[i] > 50.0f) {
                misalignBData[misalignSetIndex].data[i] = 100.0f;
            }
            else if (aRaw.data[i] < -50.0f) {
                misalignBData[misalignSetIndex].data[i] = -100.0f;
            }
            else {
                misalignBData[misalignSetIndex].data[i] = 0.0f;
            }
        }

        misalignTime += T;
        if (misalignTime > 2000.0f) {
            isMagMisalignCalEnabled = false;

            for (int i = 0; i < 3; i++) {
                if (misalignSamples == 0) break;

                if ((misalignSetIndex % 2) == 0) {
                    misalignAData[misalignSetIndex].data[i] = misalignADataAcc.data[i] / misalignSamples;
                }
                else {
                    misalignAData[misalignSetIndex].data[i] = (misalignADataAcc.data[i] / misalignSamples) - misalignAData[misalignSetIndex - 1].data[i];
                }
            }

            vectZero3x1(&misalignADataAcc);

            misalignSamples = 0;
            misalignTime = 0.0f;
        }
    }
}

void LpmsSensor::calcMagMisalignCal(void)
{
    float *aTest[N_MAG_ALIGNMENT_SETS];
    float *bTest[N_MAG_ALIGNMENT_SETS];

    for (int i = 0; i < N_MAG_ALIGNMENT_SETS; i += 2) {
        aTest[i / 2] = misalignAData[i + 1].data;
        bTest[i / 2] = misalignBData[i + 1].data;
    }

    maCalCalcMagMisalignment(aTest, bTest, &configData.misalignMatrix, &configData.accBias, N_MAG_ALIGNMENT_SETS / 2);

    configData.setParameter(PRM_SELECT_DATA, prevDataSelection);
    updateParameters();
}

LpVector3f LpmsSensor::getMisalignAData(int i)
{
    return misalignAData[i];
}

LpVector3f LpmsSensor::getGyrMisalignAData(int i)
{
    return gyrMisalignAData[i];
}

bool LpmsSensor::isSamplingAcc()
{
    return isGetMisalign;
}
bool LpmsSensor::isSamplingGyro()
{
    return isGetGyrMisalign;
}

void LpmsSensor::startSaveData(std::ofstream *saveDataHandle)
{
    sensorMutex.lock();
    bt->clearDataQueue();
    isSaveData = true;
    this->saveDataHandle = saveDataHandle;
    frameNo = 0;
    saveDataPreroll = 10;
    sensorMutex.unlock();
}

void LpmsSensor::checkSaveData(void)
{
    double currentTimestamp;

    if (saveDataPreroll > 1) {
        --saveDataPreroll;
        return;
    }
    else if (saveDataPreroll == 1) {
        --saveDataPreroll;
        timestampOffset = currentData.timeStamp;
        frameCounterOffset = currentData.frameCount;
    }

    currentTimestamp = (currentData.timeStamp - timestampOffset);

    sensorMutex.lock();
    if (isSaveData == true && saveDataHandle->is_open() == true) {
        *saveDataHandle << currentData.openMatId << ", " << std::fixed << std::setprecision(4) << currentTimestamp << std::fixed << std::setprecision(4) << ", " << (currentData.frameCount - frameCounterOffset) << ", " << currentData.a[0] << ", " << currentData.a[1] << ", " << currentData.a[2] << ", " << currentData.g[0] << ", " << currentData.g[1] << ", " << currentData.g[2] << ", " << currentData.b[0] << ", " << currentData.b[1] << ", " << currentData.b[2] << ", " << currentData.r[0] << ", " << currentData.r[1] << ", " << currentData.r[2] << ", " << currentData.q[0] << ", " << currentData.q[1] << ", " << currentData.q[2] << ", " << currentData.q[3] << ", " << currentData.linAcc[0] << ", " << currentData.linAcc[1] << ", " << currentData.linAcc[2] << ", " << currentData.pressure << ", " << currentData.altitude << ", " << currentData.temperature << ", " << currentData.heave << std::endl;
    }
    sensorMutex.unlock();
}

void LpmsSensor::stopSaveData(void)
{
    isSaveData = false;
}

void LpmsSensor::startFlashLogging(void)
{
    bt->startFlashLogging();
}

void LpmsSensor::stopFlashLogging(void)
{
    bt->stopFlashLogging();
}

void LpmsSensor::clearFlashLog(void)
{
    bt->clearFlashLog();
}

void LpmsSensor::fullEraseFlash(void)
{
    bt->fullEraseFlash();
}

void LpmsSensor::getFlashLoggingStatus(void)
{
    bt->getFlashLoggingStatus();
}

void LpmsSensor::getFlashMetaTableSize(void)
{
    bt->getFlashMetaTableSize();
}

void LpmsSensor::getFlashMetaTable(void)
{
    bt->getFlashMetaTable();
}

void LpmsSensor::getFlashLogSize(void)
{
    bt->getFlashLogSize();
}

bool LpmsSensor::getFlashLog(const char* fn)
{
    if (connectionStatus != SENSOR_CONNECTION_CONNECTED) return false;

    bt->getFlashLogSize();
    state = PREPARE_PARAMETER_ADJUSTMENT;
    getConfigState = STATE_GET_FLASH_LOG_SIZE;
    flashLogFilename = fn;

    return true;
}

void LpmsSensor::cancelGetFlashLog(void)
{
    bt->cancelGetFlashLog();
}

bool LpmsSensor::isDownloadingFlashLog()
{
    if (state == PREPARE_PARAMETER_ADJUSTMENT || state == STATE_GET_FLASH_LOG_SIZE || state == STATE_GET_FLASH_LOG || state == STATE_CHECK_GET_FLASH_LOG)
        return true;

    return false;
}

int LpmsSensor::getDownloadFlashLogProgress(int *p)
{

    if (!isDownloadingFlashLog()) {

        return 0;
    }
    if (bt->getDownloadFlashLogProgress(p) == false) return 2;

    return 1;
}

std::string LpmsSensor::getErrorMsg()
{
    return bt->getErrorMsg();
}
