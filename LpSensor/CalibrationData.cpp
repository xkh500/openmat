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

#include "CalibrationData.h"

#define DEF_GYR_BIAS_X -0.420771
#define DEF_GYR_BIAS_Y -1.41633	
#define DEF_GYR_BIAS_Z -0.141261		

#define DEF_MAG_BIAS_X -0.420771
#define DEF_MAG_BIAS_Y -1.41633	
#define DEF_MAG_BIAS_Z -0.141261		

#define DEF_ACC_REF_X 0
#define DEF_ACC_REF_Y 0
#define DEF_ACC_REF_Z -1

#define DEF_MAG_REF_X -0.114444
#define DEF_MAG_REF_Y 0.474849
#define DEF_MAG_REF_Z -0.285217

#define DEF_MAG_THRES_X 0.551806
#define DEF_MAG_THRES_Y 0.596217
#define DEF_MAG_THRES_Z 0.501186

#define DEF_GYR_THRES_X 0.910771
#define DEF_GYR_THRES_Y 1.34633
#define DEF_GYR_THRES_Z 1.75126

#define DEF_SAMPLING_RATE 200.0f

CalibrationData::CalibrationData(void):
TAG("CalibrationData")
{
    LpMatrix3x3f m;
    createIdentity3x3(&m);
    LpVector3f v;
    vectZero3x1(&v);

    for (int i = 0; i < ABSMAXPITCH; i++) {
        for (int j = 0; j < ABSMAXROLL; j++) {
            for (int k = 0; k < ABSMAXYAW; k++) {
                for (int l = 0; l < 3; l++) {
                    fieldMap[i][j][k].data[l] = 0.0f;
                }
            }
        }
    }

    firmwareVersion = std::string("n/a");
    firmwareVersionDig2 = 0;
    firmwareVersionDig1 = 0;
    firmwareVersionDig0 = 0;
    name = std::string("");
    deviceId = std::string("");
    openMatId = 1;
    deviceType = 0;
    parameterSet = 0;
    filterMode = 0;
    gyrThresEnable = 0;
    accCompGain = 0.0f;
    accCovariance = 0.0f;
    magCompGain = 0.0f;
    magCovariance = 0.0f;
    quaternionCalcLocal = 0;
    samplingRate = 0;
    gyrRange = 0;
    magRange = 0;
    accRange = 0;
    magAutocalibration = 0;
    canStreamFormat = 0;
    canBaudrate = 0;
    selfTestOn = 0;
    fieldRadius = 0;
    magThreshold = 0;
    magOutOfRange = 0;
    gyrAutocalibration = 0;
    hardIronOffset = v;
    softIronMatrix = m;
    misalignMatrix = m;
    accBias = v;
    gyrMisalignMatrix = m;
    gyrAlignmentBias = v;
    firmwareVersion = std::string("");
    lowPassFilter = 0;
    canHeartbeat = 0;
    heavemotionEnabled = 0;
    gaitTrackingEnabled = 0;
    linAccCompMode = 0;
    centriCompMode = 0;
    canPointMode = 0;
    canChannelMode = 0;
    canStartId = 0;

    selectedData = 0xffff;
}

bool CalibrationData::setDefaultParameters(std::string name, std::string deviceId, int deviceType)
{
    setParameter(PRM_NAME, name);
    setParameter(PRM_OPENMAT_ID, 0);
    setParameter(PRM_DEVICE_ID, deviceId);
    setParameter(PRM_DEVICE_TYPE, deviceType);
    setParameter(PRM_GYR_THRESHOLD_ENABLED, 0);
    setParameter(PRM_PARAMETER_SET, 0);
    setParameter(PRM_FILTER_MODE, 0);
    setParameter(PRM_GYR_RANGE, 0);
    setParameter(PRM_MAG_RANGE, 0);
    setParameter(PRM_ACC_RANGE, 0);
    setParameter(PRM_SAMPLING_RATE, DEF_SAMPLING_RATE);
    setParameter(PRM_LOCAL_Q, 0);
    setParameter(PRM_ACC_COVARIANCE, 1.0f);
    setParameter(PRM_MAG_COVARIANCE, 1.0f);
    setParameter(PRM_ACC_GAIN, 1.0f);
    setParameter(PRM_MAG_GAIN, 1.0f);
    setParameter(PRM_MAG_AUTOCALIBRATION, 0);
    setParameter(PRM_SELF_TEST, SELECT_SELF_TEST_OFF);
    setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_DISABLED);
    setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_DISABLED);

    return true;
}

bool CalibrationData::setParameter(int parameterIndex, std::string parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_NAME:
        name = parameter;
        break;

    case PRM_DEVICE_ID:
        deviceId = parameter;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::setParameter(int parameterIndex, int parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_OPENMAT_ID:
        openMatId = parameter;
        break;

    case PRM_DEVICE_TYPE:
        deviceType = parameter;
        break;

    case PRM_GYR_THRESHOLD_ENABLED:
        gyrThresEnable = parameter;
        break;

    case PRM_PARAMETER_SET:
        parameterSet = parameter;
        break;

    case PRM_FILTER_MODE:
        filterMode = parameter;
        break;

    case PRM_GYR_RANGE:
        gyrRange = parameter;
        break;

    case PRM_MAG_RANGE:
        magRange = parameter;
        break;

    case PRM_ACC_RANGE:
        accRange = parameter;
        break;

    case PRM_LOCAL_Q:
        quaternionCalcLocal = parameter;
        break;

    case PRM_MAG_AUTOCALIBRATION:
        magAutocalibration = parameter;
        break;

    case PRM_CAN_STREAM_FORMAT:
        canStreamFormat = parameter;
        break;

    case PRM_CAN_BAUDRATE:
        canBaudrate = parameter;
        break;

    case PRM_SAMPLING_RATE:
        samplingRate = parameter;
        break;

    case PRM_SELF_TEST:
        selfTestOn = parameter;
        break;

    case PRM_GYR_AUTOCALIBRATION:
        gyrAutocalibration = parameter;
        break;

    case PRM_SELECT_DATA:
        selectedData = parameter;
        break;

    case PRM_LOW_PASS:
        lowPassFilter = parameter;
        break;

    case PRM_CAN_HEARTBEAT:
        canHeartbeat = parameter;
        break;

    case PRM_HEAVEMOTION_ENABLED:
        heavemotionEnabled = parameter;
        break;

    case PRM_GAIT_TRACKING_ENABLED:
        gaitTrackingEnabled = parameter;
        break;

    case PRM_LIN_ACC_COMP_MODE:
        linAccCompMode = parameter;
        break;

    case PRM_CENTRI_COMP_MODE:
        centriCompMode = parameter;
        break;

    case PRM_CAN_CHANNEL_MODE:
        canChannelMode = parameter;
        break;

    case PRM_CAN_POINT_MODE:
        canPointMode = parameter;
        break;

    case PRM_CAN_START_ID:
        canStartId = parameter;
        break;

    case PRM_LPBUS_DATA_MODE:
        lpBusDataMode = parameter;
        break;

    case PRM_UART_BAUDRATE:
        uartBaudrate = parameter;
        break;

    case PRM_UART_FORMAT:
        uartFormat = parameter;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::setParameter(int parameterIndex, float parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_ACC_COVARIANCE:
        accCovariance = parameter;
        break;

    case PRM_MAG_COVARIANCE:
        magCovariance = parameter;
        break;

    case PRM_ACC_GAIN:
        accCompGain = parameter;
        break;

    case PRM_MAG_GAIN:
        magCompGain = parameter;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::setParameter(int parameterIndex, int *parameter)
{
    switch (parameterIndex) {
    case PRM_CAN_MAPPING:
        for (int i = 0; i < 16; ++i) {
            canMapping[i] = parameter[i];
        }
        break;
    }

    return true;
}

bool CalibrationData::getParameter(int parameterIndex, std::string *parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_NAME:
        *parameter = name;
        break;

    case PRM_DEVICE_ID:
        *parameter = deviceId;
        break;

    case PRM_FIRMWARE_VERSION:
        *parameter = firmwareVersion;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::getParameter(int parameterIndex, int *parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_OPENMAT_ID:
        *parameter = openMatId;
        break;

    case PRM_DEVICE_TYPE:
        *parameter = deviceType;
        break;

    case PRM_GYR_THRESHOLD_ENABLED:
        *parameter = gyrThresEnable;
        break;

    case PRM_PARAMETER_SET:
        *parameter = parameterSet;
        break;

    case PRM_FILTER_MODE:
        *parameter = filterMode;
        break;

    case PRM_GYR_RANGE:
        *parameter = gyrRange;
        break;

    case PRM_MAG_RANGE:
        *parameter = magRange;
        break;

    case PRM_ACC_RANGE:
        *parameter = accRange;
        break;

    case PRM_LOCAL_Q:
        *parameter = quaternionCalcLocal;
        break;

    case PRM_MAG_AUTOCALIBRATION:
        *parameter = magAutocalibration;
        break;

    case PRM_CAN_STREAM_FORMAT:
        *parameter = canStreamFormat;
        break;

    case PRM_CAN_BAUDRATE:
        *parameter = canBaudrate;
        break;

    case PRM_SAMPLING_RATE:
        *parameter = samplingRate;
        break;

    case PRM_SELF_TEST:
        *parameter = selfTestOn;
        break;

    case PRM_GYR_AUTOCALIBRATION:
        *parameter = gyrAutocalibration;
        break;

    case PRM_SELECT_DATA:
        *parameter = selectedData;
        break;

    case PRM_LOW_PASS:
        *parameter = lowPassFilter;
        break;

    case PRM_CAN_HEARTBEAT:
        *parameter = canHeartbeat;
        break;

    case PRM_HEAVEMOTION_ENABLED:
        *parameter = heavemotionEnabled;
        break;

    case PRM_GAIT_TRACKING_ENABLED:
        *parameter = gaitTrackingEnabled;
        break;

    case PRM_CAN_MAPPING:
        for (int i = 0; i < 16; ++i) {
            parameter[i] = canMapping[i];
        }
        break;

    case PRM_LIN_ACC_COMP_MODE:
        *parameter = linAccCompMode;
        break;

    case PRM_CENTRI_COMP_MODE:
        *parameter = centriCompMode;
        break;

    case PRM_CAN_CHANNEL_MODE:
        *parameter = canChannelMode;
        break;

    case PRM_CAN_POINT_MODE:
        *parameter = canPointMode;
        break;

    case PRM_CAN_START_ID:
        *parameter = canStartId;
        break;

    case PRM_LPBUS_DATA_MODE:
        *parameter = lpBusDataMode;
        break;

    case PRM_UART_BAUDRATE:
        *parameter = uartBaudrate;
        break;

    case PRM_UART_FORMAT:
        *parameter = uartFormat;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::getParameter(int parameterIndex, float *parameter)
{
    calibrationMutex.lock();

    switch (parameterIndex) {
    case PRM_ACC_COVARIANCE:
        *parameter = accCovariance;
        break;

    case PRM_MAG_COVARIANCE:
        *parameter = magCovariance;
        break;

    case PRM_ACC_GAIN:
        *parameter = accCompGain;
        break;

    case PRM_MAG_GAIN:
        *parameter = magCompGain;
        break;
    }

    calibrationMutex.unlock();

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpVector3f *v)
{
    boost::char_separator<char> sep(", ");
    int i;
    float x;

    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
    i = 0;
    BOOST_FOREACH(std::string t, tokens) {
        try {
            x = boost::lexical_cast<float>(t);
        }
        catch (boost::bad_lexical_cast &) {
            x = 1.0f;
        }

        if (i < 3) v->data[i] = x;
        ++i;
    }

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, float *v)
{
    boost::char_separator<char> sep(", ");
    float x;

    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    try {
        x = boost::lexical_cast<float>(line);
    }
    catch (boost::bad_lexical_cast &) {
        x = 1.0f;
    }

    *v = x;

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpVector4f *v)
{
    boost::char_separator<char> sep(", ");
    int i;
    float x;

    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
    i = 0;
    BOOST_FOREACH(std::string t, tokens) {
        try {
            x = boost::lexical_cast<float>(t);
        }
        catch (boost::bad_lexical_cast &) {
            x = 1.0f;
        }

        if (i < 4) v->data[i] = x;
        ++i;
    }

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, LpMatrix3x3f *m)
{
    boost::char_separator<char> sep(", ");
    int i;
    float x;

    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
    i = 0;
    BOOST_FOREACH(std::string t, tokens) {
        try {
            x = boost::lexical_cast<float>(t);
            if (i < 9) m->data[i / 3][i % 3] = x;
        }
        catch (boost::bad_lexical_cast &) {
            if (i / 3 == i % 3) {
                m->data[i / 3][i % 3] = 1.0f;
            }
            else {
                m->data[i / 3][i % 3] = 0.0f;
            }
        }
        ++i;
    }

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, int *i)
{
    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    try {
        *i = boost::lexical_cast<int>(line);
    }
    catch (boost::bad_lexical_cast &) {
        *i = 0;
    }

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, bool *b)
{
    std::string line = node.child_value(tag.c_str());

    if (line == "") return false;

    if (line == "true") {
        *b = true;
    }
    else {
        *b = false;
    }

    return true;
}

bool CalibrationData::readXML(std::string tag, pugi::xml_node node, std::string *s)
{
    *s = node.child_value(tag.c_str());

    if (*s == "") return false;

    return true;
}

bool CalibrationData::load(std::string fn)
{
    pugi::xml_document document;
    pugi::xml_parse_result r = document.load_file(fn.c_str());

    if (!r) {
        std::cout << "[LpmsSensorManager] Could not open configuration file" << std::endl;
        return false;
    }

    pugi::xml_node configuration = document.child("LpmsControlConfiguration");

    if (!configuration) {
        std::cout << "[LpmsSensorManager] XML file doesn't contain SensorConfiguration tag" << std::endl;
        return false;
    }

    readXML("FieldEstimate", configuration, &fieldRadius);
    readXML("HardIronOffset", configuration, &hardIronOffset);
    readXML("SoftIronMatrix", configuration, &softIronMatrix);
    readXML("MisalignmentMatrix", configuration, &misalignMatrix);
    readXML("AccelerometerBias", configuration, &accBias);
    readXML("GyroMisalignmentMatrix", configuration, &gyrMisalignMatrix);
    readXML("GyroMisalignmentBias", configuration, &gyrAlignmentBias);

    return true;
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, float v)
{
    std::ostringstream s;
    s << v;

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpVector3f v)
{
    std::ostringstream s;
    s << v.data[0] << ", " << v.data[1] << ", " << v.data[2];

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpVector4f v)
{
    std::ostringstream s;
    s << v.data[0] << ", " << v.data[1] << ", " << v.data[2] << ", " << v.data[3];

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, LpMatrix3x3f m)
{
    std::ostringstream s;
    s << m.data[0][0] << ", " << m.data[0][1] << ", " << m.data[0][2] << ", " <<
        m.data[1][0] << ", " << m.data[1][1] << ", " << m.data[1][2] << ", " <<
        m.data[2][0] << ", " << m.data[2][1] << ", " << m.data[2][2];

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, bool b)
{
    std::ostringstream s;

    if (b == true) {
        s << "true";
    }
    else {
        s << "false";
    }

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, std::string s)
{
    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.c_str());
}

void CalibrationData::writeXML(std::string tag, pugi::xml_node node, int i)
{
    std::ostringstream s;
    s << i;

    node.append_child(tag.c_str()).append_child(pugi::node_pcdata).set_value(s.str().c_str());
}

bool CalibrationData::save(std::string fn)
{
    pugi::xml_document doc;
    pugi::xml_node configuration = doc.append_child("LpmsControlConfiguration");

    writeXML("FieldEstimate", configuration, fieldRadius);
    writeXML("HardIronOffset", configuration, hardIronOffset);
    writeXML("SoftIronMatrix", configuration, softIronMatrix);
    writeXML("MisalignmentMatrix", configuration, misalignMatrix);
    writeXML("AccelerometerBias", configuration, accBias);
    writeXML("GyroMisalignmentMatrix", configuration, gyrMisalignMatrix);
    writeXML("GyroMisalignmentBias", configuration, gyrAlignmentBias);


    if (!doc.save_file(fn.c_str())) {
        logd(TAG, "Error writing configuration file %s\n", fn.c_str());
    }

    return true;
}

void printMatrix(std::string tag, LpMatrix3x3f m)
{
    for (int i = 0; i < 3; i++) {
        logd(tag, "%f %f %f\n", m.data[i][0], m.data[i][1], m.data[i][2]);
    }
}

void printVector(std::string tag, LpVector3f v)
{
    logd(tag, "%f %f %f\n", v.data[0], v.data[1], v.data[2]);
}

void CalibrationData::print(void)
{
    logd(TAG, "DeviceID: %s\n", deviceId.c_str());
    logd(TAG, "OpenMAT ID: %d\n", openMatId);
    logd(TAG, "DeviceType: %d\n", deviceType);
    logd(TAG, "Parameter set: %d\n", parameterSet);
    logd(TAG, "Filter mode: %d\n", filterMode);
    logd(TAG, "Gyroscope threshold: %d\n", gyrThresEnable);
    logd(TAG, "Sampling rate: %d\n", samplingRate);
    logd(TAG, "16 Bit data: %d\n", lpBusDataMode);
    logd(TAG, "Gyro range: %d\n", gyrRange);
    logd(TAG, "Mag. range: %d\n", magRange);
    logd(TAG, "Acc. range: %d\n", accRange);
    logd(TAG, "CAN Baudrate: %d\n", canBaudrate);
    logd(TAG, "Field estimate: %f\n", fieldRadius);
    logd(TAG, "Gyr. auto-calibration on / off: %d\n", gyrAutocalibration);
    logd(TAG, "Centripetal acc compensation on / off: %d\n", centriCompMode);
    logd(TAG, "Hard iron offset:\n");
    printVector(TAG, hardIronOffset);
    logd(TAG, "Soft iron matrix:\n");
    printMatrix(TAG, softIronMatrix);
    logd(TAG, "Misalignment matrix:\n");
    printMatrix(TAG, misalignMatrix);
    logd(TAG, "Accelerometer bias:\n");
    printVector(TAG, accBias);
    logd(TAG, "Gyroscope misalignment matrix:\n");
    printMatrix(TAG, gyrMisalignMatrix);
    logd(TAG, "Gyroscope alignment bias:\n");
    printVector(TAG, gyrAlignmentBias);
    logd(TAG, "Magnetometer misalignment matrix:\n");
    printMatrix(TAG, magMAlignmentMatrix);
    logd(TAG, "Magnetometer alignment bias:\n");
    printVector(TAG, magMAlignmentBias);
    logd(TAG, "Magnetometer reference:\n");
    printVector(TAG, magReference);
    logd(TAG, "Selected data: 0x%d\n", selectedData);
    logd(TAG, "Firmware version %s\n", firmwareVersion.c_str());
    logd(TAG, "Firmware Info: %s\n", firmwareInfo.c_str());
    logd(TAG, "Serial Number: %s\n", serialNumber.c_str());
}