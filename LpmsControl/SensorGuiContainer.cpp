/***********************************************************************
** Copyright (C) 2011 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
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

#include "SensorGuiContainer.h"

SensorGuiContainer::SensorGuiContainer(LpmsSensorI* sensor, QTreeWidget* tree) :
sensor(sensor),
containerSetup(false)
{
    int l;

    deviceType = 0;
    sensor->getConfigurationPrm(PRM_DEVICE_TYPE, &deviceType);

    QGridLayout* gl = new QGridLayout();
    QGridLayout* gl1 = new QGridLayout();
    QGridLayout* gl2 = new QGridLayout();
    selectedDataGl = new QGridLayout();
    QGridLayout* gl4 = new QGridLayout();
    QGridLayout* gl5 = new QGridLayout();
    QGridLayout* gl6 = new QGridLayout();

    l = 0;
    gl5->addWidget(new QLabel("Connection:"), l, 0); ++l;
    gl5->addWidget(new QLabel("Sensor status:"), l, 0); ++l;
    gl5->addWidget(new QLabel("Device ID:"), l, 0); ++l;
    gl5->addWidget(new QLabel("Firmware version:"), l, 0); ++l;
    if (deviceType == DEVICE_LPMS_B2)
        gl5->addWidget(new QLabel("Battery:"), l, 0); ++l;


    l = 0;
    gl->addWidget(new QLabel("IMU ID:"), l, 0); ++l;
    gl->addWidget(new QLabel("Transmission rate:"), l, 0); ++l;

    l = 0;
    if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        gl6->addWidget(new QLabel("Baud rate:"), l, 0); ++l;
        gl6->addWidget(new QLabel("Data format:"), l, 0); ++l;
    }

    l = 0;
    gl1->addWidget(new QLabel("GYR range:"), l, 0); ++l;
    gl1->addWidget(new QLabel("ACC range:"), l, 0); ++l;
    gl1->addWidget(new QLabel("MAG range:"), l, 0); ++l;

    l = 0;
    gl2->addWidget(new QLabel("Filter mode:"), l, 0); ++l;
    gl2->addWidget(new QLabel("MAG correction:"), l, 0); ++l;
    gl2->addWidget(new QLabel("Lin. ACC correction:"), l, 0); ++l;
    gl2->addWidget(new QLabel("Rot. ACC correction:"), l, 0); ++l;
    gl2->addWidget(new QLabel("GYR threshhold:"), l, 0); ++l;
    gl2->addWidget(new QLabel("GYR autocalibration:"), l, 0); ++l;
    gl2->addWidget(new QLabel("Low-pass filter:"), l, 0); ++l;

    l = 0;
    gl5->addWidget(statusItem = new QLabel("<font color='blue'>in progress</font>"), l, 1); ++l;
    gl5->addWidget(runningItem = new QLabel("<font color='blue'>initializing..</font>"), l, 1); ++l;
    gl5->addWidget(addressItem = new QLabel("n/a"), l, 1); ++l;
    gl5->addWidget(firmwareItem = new QLabel("n/a"), l, 1); ++l;
    if (deviceType == DEVICE_LPMS_B2)
        gl5->addWidget(batteryItem = new QLabel("n/a"), l, 1); ++l;

    l = 0;
    gl->addWidget(indexItem = new QComboBox(), l, 1); ++l;
    gl->addWidget(samplingRateCombo = new QComboBox(), l, 1); ++l;

    l = 0;
    gl6->addWidget(baudRateCombo = new QComboBox(), l, 1); ++l;
    gl6->addWidget(uartFormatCombo = new QComboBox(), l, 1); ++l;

    l = 0;
    gl1->addWidget(gyrRangeCombo = new QComboBox(), l, 1); ++l;
    gl1->addWidget(accRangeCombo = new QComboBox(), l, 1); ++l;
    gl1->addWidget(magRangeCombo = new QComboBox(), l, 1); ++l;

    l = 0;
    gl2->addWidget(filterModeCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(parameterSetCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(linAccCompModeCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(centriCompModeCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(thresholdEnableCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(gyrAutocalibrationCombo = new QComboBox(), l, 1); ++l;
    gl2->addWidget(lowPassCombo = new QComboBox(), l, 1); ++l;

    for (int i = 0; i < 129; i++) indexItem->addItem(QString("%1").arg(i));
    connect(indexItem, SIGNAL(currentIndexChanged(int)), this, SLOT(updateOpenMATIndex(int)));

    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(2, 0, 5);
    // if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        // if (legacyFirmware)
        // {
            // baudRateCombo->addItem(QString("19200 bps"));
            // baudRateCombo->addItem(QString("57600 bps"));
            // baudRateCombo->addItem(QString("115200 bps"));
            // baudRateCombo->addItem(QString("921600 bps"));
            // baudRateCombo->setCurrentIndex(2);
        // }
        // else
        // {
            // baudRateCombo->addItem(QString("19200 bps"));
            // baudRateCombo->addItem(QString("38400 bps"));
            // baudRateCombo->addItem(QString("57600 bps"));
            // baudRateCombo->addItem(QString("115200 bps"));
            // baudRateCombo->addItem(QString("230400 bps"));
            // baudRateCombo->addItem(QString("256000 bps"));
            // baudRateCombo->addItem(QString("460800 bps"));
            // baudRateCombo->addItem(QString("921600 bps"));
            // baudRateCombo->setCurrentIndex(3);

        // }
        // connect(baudRateCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateBaudRateIndex(int)));

        // uartFormatCombo->addItem(QString("LPBUS (binary)"));
        // uartFormatCombo->addItem(QString("ASCII (CSV)"));
        // connect(uartFormatCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateUartFormatIndex(int)));
    // }

    samplingRateCombo->addItem(QString("5 Hz"));
    samplingRateCombo->addItem(QString("10 Hz"));
    samplingRateCombo->addItem(QString("25 Hz"));
    if (deviceType != DEVICE_LPMS_BLE) {
        samplingRateCombo->addItem(QString("50 Hz"));
        samplingRateCombo->addItem(QString("100 Hz"));

        // if (deviceType != DEVICE_LPMS_B) {
        samplingRateCombo->addItem(QString("200 Hz"));
        samplingRateCombo->addItem(QString("400 Hz"));
        // } else {
            // samplingRateCombo->addItem(QString("133 Hz"));
        // }
    }

    connect(samplingRateCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updatesamplingRate(int)));

    QComboBox* syncCombo = new QComboBox();
    syncCombo->addItem(QString("On"));
    syncCombo->addItem(QString("Off"));

    parameterSetCombo->addItem(QString("Weak"));
    parameterSetCombo->addItem(QString("Medium"));
    parameterSetCombo->addItem(QString("Strong"));
    parameterSetCombo->addItem(QString("Dynamic"));
    connect(parameterSetCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFilterPreset(int)));

    linAccCompModeCombo->addItem(QString("Off"));
    linAccCompModeCombo->addItem(QString("Weak"));
    linAccCompModeCombo->addItem(QString("Medium"));
    linAccCompModeCombo->addItem(QString("Strong"));
    linAccCompModeCombo->addItem(QString("Ultra"));
    connect(linAccCompModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLinAccCompMode(int)));

    centriCompModeCombo->addItem(QString("Disable"));
    centriCompModeCombo->addItem(QString("Enable"));
    connect(centriCompModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCentriCompMode(int)));

    filterModeCombo->addItem(QString("Gyr only"));
    filterModeCombo->addItem(QString("Gyr + Acc (Kalman)"));
    filterModeCombo->addItem(QString("Gyr + Acc + Mag (Kalman)"));
    filterModeCombo->addItem(QString("Gyr + Acc (DCM)"));
    filterModeCombo->addItem(QString("Gyr + Acc + Mag (DCM)"));
    connect(filterModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFilterMode(int)));

    thresholdEnableCombo->addItem(QString("Disable"));
    thresholdEnableCombo->addItem(QString("Enable"));
    connect(thresholdEnableCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrThresholdEnable(int)));

    gyrAutocalibrationCombo->addItem(QString("Disable"));
    gyrAutocalibrationCombo->addItem(QString("Enable"));
    connect(gyrAutocalibrationCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrAutocalibration(int)));

    lowPassCombo->addItem(QString("off"));
    lowPassCombo->addItem(QString("0.1"));
    lowPassCombo->addItem(QString("0.05"));
    lowPassCombo->addItem(QString("0.01"));
    lowPassCombo->addItem(QString("0.005"));
    lowPassCombo->addItem(QString("0.001"));
    connect(lowPassCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLowPass(int)));

    accRangeCombo->addItem(QString("2G"));
    accRangeCombo->addItem(QString("4G"));
    accRangeCombo->addItem(QString("8G"));
    accRangeCombo->addItem(QString("16G"));
    connect(accRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateAccRange(int)));

    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1,9,9);
    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // magRangeCombo->addItem(QString("4 Gauss"));
        // magRangeCombo->addItem(QString("8 Gauss"));
        // magRangeCombo->addItem(QString("12 Gauss"));
        // magRangeCombo->addItem(QString("16 Gauss"));
    // }
    // else
    // {
        // magRangeCombo->addItem(QString("130 uT"));
        // magRangeCombo->addItem(QString("190 uT"));
        // magRangeCombo->addItem(QString("250 uT"));
        // magRangeCombo->addItem(QString("400 uT"));
        // magRangeCombo->addItem(QString("560 uT"));
        // magRangeCombo->addItem(QString("810 uT"));
    // }
    // connect(magRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateMagRange(int)));

    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // gyrRangeCombo->addItem(QString("125 dps"));
        // gyrRangeCombo->addItem(QString("245 dps"));
        // gyrRangeCombo->addItem(QString("500 dps"));
        // gyrRangeCombo->addItem(QString("1000 dps"));
        // gyrRangeCombo->addItem(QString("2000 dps"));
    // }
    // else
    // {
        // gyrRangeCombo->addItem(QString("250 dps"));
        // gyrRangeCombo->addItem(QString("500 dps"));
        // gyrRangeCombo->addItem(QString("2000 dps"));
    // }
    // connect(gyrRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrRange(int)));

    if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        l = 0;

        gl4->addWidget(new QLabel("CAN baudrate"), l, 0);
        gl4->addWidget(canBaudrateCombo = new QComboBox(), l, 1); ++l;

        canBaudrateCombo->addItem(QString("1 MBit/s"));
        canBaudrateCombo->addItem(QString("500 KBit/s"));
        canBaudrateCombo->addItem(QString("250 KBit/s"));
        canBaudrateCombo->addItem(QString("125 KBit/s"));

        connect(canBaudrateCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanBaudrate(int)));

        gl4->addWidget(new QLabel("Channel mode"), l, 0);
        gl4->addWidget(canChannelModeCombo = new QComboBox(), l, 1); ++l;

        canChannelModeCombo->addItem(QString("CANOpen"));
        canChannelModeCombo->addItem(QString("Sequential"));

        connect(canChannelModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanChannelMode(int)));

        gl4->addWidget(new QLabel("Value mode"), l, 0);
        gl4->addWidget(canPointModeCombo = new QComboBox(), l, 1); ++l;

        canPointModeCombo->addItem(QString("32-bit floating point"));
        canPointModeCombo->addItem(QString("16-bit fixed point"));

        connect(canPointModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanPointMode(int)));

        gl4->addWidget(new QLabel("Start ID"), l, 0);
        gl4->addWidget(canStartIdSpin = new QSpinBox(), l, 1); ++l;
        canStartIdSpin->setRange(0, 0xffff);

        connect(canStartIdSpin, SIGNAL(valueChanged(int)), this, SLOT(updateCanStartId(int)));

        gl4->addWidget(new QLabel("Heartbeat freq."), l, 0);
        gl4->addWidget(canHeartbeatCombo = new QComboBox(), l, 1); ++l;

        canHeartbeatCombo->addItem(QString("0.5s"));
        canHeartbeatCombo->addItem(QString("1.0s"));
        canHeartbeatCombo->addItem(QString("2.0s"));
        canHeartbeatCombo->addItem(QString("5.0s"));
        canHeartbeatCombo->addItem(QString("10.0s"));

        connect(canHeartbeatCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanHeartbeat(int)));

        gl4->addWidget(new QLabel("Channel 1"), l, 0);
        gl4->addWidget(canTpdo1ACombo = new QComboBox(), l, 1); ++l;
        canTpdo1ACombo->addItem(QString("Gyroscope X"));
        canTpdo1ACombo->addItem(QString("Gyroscope Y"));
        canTpdo1ACombo->addItem(QString("Gyroscope Z"));
        canTpdo1ACombo->addItem(QString("Euler angle X"));
        canTpdo1ACombo->addItem(QString("Euler angle Y"));
        canTpdo1ACombo->addItem(QString("Euler angle Z"));
        canTpdo1ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo1ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo1ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo1ACombo->addItem(QString("Magnetometer X"));
        canTpdo1ACombo->addItem(QString("Magnetometer Y"));
        canTpdo1ACombo->addItem(QString("Magnetometer Z"));
        canTpdo1ACombo->addItem(QString("Quaternion W"));
        canTpdo1ACombo->addItem(QString("Quaternion X"));
        canTpdo1ACombo->addItem(QString("Quaternion Y"));
        canTpdo1ACombo->addItem(QString("Quaternion Z"));
        canTpdo1ACombo->addItem(QString("Accelerometer X"));
        canTpdo1ACombo->addItem(QString("Accelerometer Y"));
        canTpdo1ACombo->addItem(QString("Accelerometer Z"));
        canTpdo1ACombo->addItem(QString("Angular Velocity X"));
        canTpdo1ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo1ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo1ACombo->addItem(QString("Pressure"));
        canTpdo1ACombo->addItem(QString("Altitude"));
        canTpdo1ACombo->addItem(QString("Temperature"));
        connect(canTpdo1ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 2"), l, 0);
        gl4->addWidget(canTpdo1BCombo = new QComboBox(), l, 1); ++l;
        canTpdo1BCombo->addItem(QString("Gyroscope X"));
        canTpdo1BCombo->addItem(QString("Gyroscope Y"));
        canTpdo1BCombo->addItem(QString("Gyroscope Z"));
        canTpdo1BCombo->addItem(QString("Euler angle X"));
        canTpdo1BCombo->addItem(QString("Euler angle Y"));
        canTpdo1BCombo->addItem(QString("Euler angle Z"));
        canTpdo1BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo1BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo1BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo1BCombo->addItem(QString("Magnetometer X"));
        canTpdo1BCombo->addItem(QString("Magnetometer Y"));
        canTpdo1BCombo->addItem(QString("Magnetometer Z"));
        canTpdo1BCombo->addItem(QString("Quaternion W"));
        canTpdo1BCombo->addItem(QString("Quaternion X"));
        canTpdo1BCombo->addItem(QString("Quaternion Y"));
        canTpdo1BCombo->addItem(QString("Quaternion Z"));
        canTpdo1BCombo->addItem(QString("Accelerometer X"));
        canTpdo1BCombo->addItem(QString("Accelerometer Y"));
        canTpdo1BCombo->addItem(QString("Accelerometer Z"));
        canTpdo1BCombo->addItem(QString("Angular Velocity X"));
        canTpdo1BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo1BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo1BCombo->addItem(QString("Pressure"));
        canTpdo1BCombo->addItem(QString("Altitude"));
        canTpdo1BCombo->addItem(QString("Temperature"));
        connect(canTpdo1BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 3"), l, 0);
        gl4->addWidget(canTpdo2ACombo = new QComboBox(), l, 1); ++l;
        canTpdo2ACombo->addItem(QString("Gyroscope X"));
        canTpdo2ACombo->addItem(QString("Gyroscope Y"));
        canTpdo2ACombo->addItem(QString("Gyroscope Z"));
        canTpdo2ACombo->addItem(QString("Euler angle X"));
        canTpdo2ACombo->addItem(QString("Euler angle Y"));
        canTpdo2ACombo->addItem(QString("Euler angle Z"));
        canTpdo2ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo2ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo2ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo2ACombo->addItem(QString("Magnetometer X"));
        canTpdo2ACombo->addItem(QString("Magnetometer Y"));
        canTpdo2ACombo->addItem(QString("Magnetometer Z"));
        canTpdo2ACombo->addItem(QString("Quaternion W"));
        canTpdo2ACombo->addItem(QString("Quaternion X"));
        canTpdo2ACombo->addItem(QString("Quaternion Y"));
        canTpdo2ACombo->addItem(QString("Quaternion Z"));
        canTpdo2ACombo->addItem(QString("Accelerometer X"));
        canTpdo2ACombo->addItem(QString("Accelerometer Y"));
        canTpdo2ACombo->addItem(QString("Accelerometer Z"));
        canTpdo2ACombo->addItem(QString("Angular Velocity X"));
        canTpdo2ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo2ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo2ACombo->addItem(QString("Pressure"));
        canTpdo2ACombo->addItem(QString("Altitude"));
        canTpdo2ACombo->addItem(QString("Temperature"));
        connect(canTpdo2ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 4"), l, 0);
        gl4->addWidget(canTpdo2BCombo = new QComboBox(), l, 1); ++l;
        canTpdo2BCombo->addItem(QString("Gyroscope X"));
        canTpdo2BCombo->addItem(QString("Gyroscope Y"));
        canTpdo2BCombo->addItem(QString("Gyroscope Z"));
        canTpdo2BCombo->addItem(QString("Euler angle X"));
        canTpdo2BCombo->addItem(QString("Euler angle Y"));
        canTpdo2BCombo->addItem(QString("Euler angle Z"));
        canTpdo2BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo2BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo2BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo2BCombo->addItem(QString("Magnetometer X"));
        canTpdo2BCombo->addItem(QString("Magnetometer Y"));
        canTpdo2BCombo->addItem(QString("Magnetometer Z"));
        canTpdo2BCombo->addItem(QString("Quaternion W"));
        canTpdo2BCombo->addItem(QString("Quaternion X"));
        canTpdo2BCombo->addItem(QString("Quaternion Y"));
        canTpdo2BCombo->addItem(QString("Quaternion Z"));
        canTpdo2BCombo->addItem(QString("Accelerometer X"));
        canTpdo2BCombo->addItem(QString("Accelerometer Y"));
        canTpdo2BCombo->addItem(QString("Accelerometer Z"));
        canTpdo2BCombo->addItem(QString("Angular Velocity X"));
        canTpdo2BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo2BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo2BCombo->addItem(QString("Pressure"));
        canTpdo2BCombo->addItem(QString("Altitude"));
        canTpdo2BCombo->addItem(QString("Temperature"));
        connect(canTpdo2BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 5"), l, 0);
        gl4->addWidget(canTpdo3ACombo = new QComboBox(), l, 1); ++l;
        canTpdo3ACombo->addItem(QString("Gyroscope X"));
        canTpdo3ACombo->addItem(QString("Gyroscope Y"));
        canTpdo3ACombo->addItem(QString("Gyroscope Z"));
        canTpdo3ACombo->addItem(QString("Euler angle X"));
        canTpdo3ACombo->addItem(QString("Euler angle Y"));
        canTpdo3ACombo->addItem(QString("Euler angle Z"));
        canTpdo3ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo3ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo3ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo3ACombo->addItem(QString("Magnetometer X"));
        canTpdo3ACombo->addItem(QString("Magnetometer Y"));
        canTpdo3ACombo->addItem(QString("Magnetometer Z"));
        canTpdo3ACombo->addItem(QString("Quaternion W"));
        canTpdo3ACombo->addItem(QString("Quaternion X"));
        canTpdo3ACombo->addItem(QString("Quaternion Y"));
        canTpdo3ACombo->addItem(QString("Quaternion Z"));
        canTpdo3ACombo->addItem(QString("Accelerometer X"));
        canTpdo3ACombo->addItem(QString("Accelerometer Y"));
        canTpdo3ACombo->addItem(QString("Accelerometer Z"));
        canTpdo3ACombo->addItem(QString("Angular Velocity X"));
        canTpdo3ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo3ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo3ACombo->addItem(QString("Pressure"));
        canTpdo3ACombo->addItem(QString("Altitude"));
        canTpdo3ACombo->addItem(QString("Temperature"));
        connect(canTpdo3ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 6"), l, 0);
        gl4->addWidget(canTpdo3BCombo = new QComboBox(), l, 1); ++l;
        canTpdo3BCombo->addItem(QString("Gyroscope X"));
        canTpdo3BCombo->addItem(QString("Gyroscope Y"));
        canTpdo3BCombo->addItem(QString("Gyroscope Z"));
        canTpdo3BCombo->addItem(QString("Euler angle X"));
        canTpdo3BCombo->addItem(QString("Euler angle Y"));
        canTpdo3BCombo->addItem(QString("Euler angle Z"));
        canTpdo3BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo3BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo3BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo3BCombo->addItem(QString("Magnetometer X"));
        canTpdo3BCombo->addItem(QString("Magnetometer Y"));
        canTpdo3BCombo->addItem(QString("Magnetometer Z"));
        canTpdo3BCombo->addItem(QString("Quaternion W"));
        canTpdo3BCombo->addItem(QString("Quaternion X"));
        canTpdo3BCombo->addItem(QString("Quaternion Y"));
        canTpdo3BCombo->addItem(QString("Quaternion Z"));
        canTpdo3BCombo->addItem(QString("Accelerometer X"));
        canTpdo3BCombo->addItem(QString("Accelerometer Y"));
        canTpdo3BCombo->addItem(QString("Accelerometer Z"));
        canTpdo3BCombo->addItem(QString("Angular Velocity X"));
        canTpdo3BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo3BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo3BCombo->addItem(QString("Pressure"));
        canTpdo3BCombo->addItem(QString("Altitude"));
        canTpdo3BCombo->addItem(QString("Temperature"));
        connect(canTpdo3BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 7"), l, 0);
        gl4->addWidget(canTpdo4ACombo = new QComboBox(), l, 1); ++l;
        canTpdo4ACombo->addItem(QString("Gyroscope X"));
        canTpdo4ACombo->addItem(QString("Gyroscope Y"));
        canTpdo4ACombo->addItem(QString("Gyroscope Z"));
        canTpdo4ACombo->addItem(QString("Euler angle X"));
        canTpdo4ACombo->addItem(QString("Euler angle Y"));
        canTpdo4ACombo->addItem(QString("Euler angle Z"));
        canTpdo4ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo4ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo4ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo4ACombo->addItem(QString("Magnetometer X"));
        canTpdo4ACombo->addItem(QString("Magnetometer Y"));
        canTpdo4ACombo->addItem(QString("Magnetometer Z"));
        canTpdo4ACombo->addItem(QString("Quaternion W"));
        canTpdo4ACombo->addItem(QString("Quaternion X"));
        canTpdo4ACombo->addItem(QString("Quaternion Y"));
        canTpdo4ACombo->addItem(QString("Quaternion Z"));
        canTpdo4ACombo->addItem(QString("Accelerometer X"));
        canTpdo4ACombo->addItem(QString("Accelerometer Y"));
        canTpdo4ACombo->addItem(QString("Accelerometer Z"));
        canTpdo4ACombo->addItem(QString("Angular Velocity X"));
        canTpdo4ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo4ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo4ACombo->addItem(QString("Pressure"));
        canTpdo4ACombo->addItem(QString("Altitude"));
        canTpdo4ACombo->addItem(QString("Temperature"));
        connect(canTpdo4ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 8"), l, 0);
        gl4->addWidget(canTpdo4BCombo = new QComboBox(), l, 1); ++l;
        canTpdo4BCombo->addItem(QString("Gyroscope X"));
        canTpdo4BCombo->addItem(QString("Gyroscope Y"));
        canTpdo4BCombo->addItem(QString("Gyroscope Z"));
        canTpdo4BCombo->addItem(QString("Euler angle X"));
        canTpdo4BCombo->addItem(QString("Euler angle Y"));
        canTpdo4BCombo->addItem(QString("Euler angle Z"));
        canTpdo4BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo4BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo4BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo4BCombo->addItem(QString("Magnetometer X"));
        canTpdo4BCombo->addItem(QString("Magnetometer Y"));
        canTpdo4BCombo->addItem(QString("Magnetometer Z"));
        canTpdo4BCombo->addItem(QString("Quaternion W"));
        canTpdo4BCombo->addItem(QString("Quaternion X"));
        canTpdo4BCombo->addItem(QString("Quaternion Y"));
        canTpdo4BCombo->addItem(QString("Quaternion Z"));
        canTpdo4BCombo->addItem(QString("Accelerometer X"));
        canTpdo4BCombo->addItem(QString("Accelerometer Y"));
        canTpdo4BCombo->addItem(QString("Accelerometer Z"));
        canTpdo4BCombo->addItem(QString("Angular Velocity X"));
        canTpdo4BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo4BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo4BCombo->addItem(QString("Pressure"));
        canTpdo4BCombo->addItem(QString("Altitude"));
        canTpdo4BCombo->addItem(QString("Temperature"));
        connect(canTpdo4BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 9"), l, 0);
        gl4->addWidget(canTpdo5ACombo = new QComboBox(), l, 1); ++l;
        canTpdo5ACombo->addItem(QString("Gyroscope X"));
        canTpdo5ACombo->addItem(QString("Gyroscope Y"));
        canTpdo5ACombo->addItem(QString("Gyroscope Z"));
        canTpdo5ACombo->addItem(QString("Euler angle X"));
        canTpdo5ACombo->addItem(QString("Euler angle Y"));
        canTpdo5ACombo->addItem(QString("Euler angle Z"));
        canTpdo5ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo5ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo5ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo5ACombo->addItem(QString("Magnetometer X"));
        canTpdo5ACombo->addItem(QString("Magnetometer Y"));
        canTpdo5ACombo->addItem(QString("Magnetometer Z"));
        canTpdo5ACombo->addItem(QString("Quaternion W"));
        canTpdo5ACombo->addItem(QString("Quaternion X"));
        canTpdo5ACombo->addItem(QString("Quaternion Y"));
        canTpdo5ACombo->addItem(QString("Quaternion Z"));
        canTpdo5ACombo->addItem(QString("Accelerometer X"));
        canTpdo5ACombo->addItem(QString("Accelerometer Y"));
        canTpdo5ACombo->addItem(QString("Accelerometer Z"));
        canTpdo5ACombo->addItem(QString("Angular Velocity X"));
        canTpdo5ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo5ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo5ACombo->addItem(QString("Pressure"));
        canTpdo5ACombo->addItem(QString("Altitude"));
        canTpdo5ACombo->addItem(QString("Temperature"));
        connect(canTpdo5ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 10"), l, 0);
        gl4->addWidget(canTpdo5BCombo = new QComboBox(), l, 1); ++l;
        canTpdo5BCombo->addItem(QString("Gyroscope X"));
        canTpdo5BCombo->addItem(QString("Gyroscope Y"));
        canTpdo5BCombo->addItem(QString("Gyroscope Z"));
        canTpdo5BCombo->addItem(QString("Euler angle X"));
        canTpdo5BCombo->addItem(QString("Euler angle Y"));
        canTpdo5BCombo->addItem(QString("Euler angle Z"));
        canTpdo5BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo5BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo5BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo5BCombo->addItem(QString("Magnetometer X"));
        canTpdo5BCombo->addItem(QString("Magnetometer Y"));
        canTpdo5BCombo->addItem(QString("Magnetometer Z"));
        canTpdo5BCombo->addItem(QString("Quaternion W"));
        canTpdo5BCombo->addItem(QString("Quaternion X"));
        canTpdo5BCombo->addItem(QString("Quaternion Y"));
        canTpdo5BCombo->addItem(QString("Quaternion Z"));
        canTpdo5BCombo->addItem(QString("Accelerometer X"));
        canTpdo5BCombo->addItem(QString("Accelerometer Y"));
        canTpdo5BCombo->addItem(QString("Accelerometer Z"));
        canTpdo5BCombo->addItem(QString("Angular Velocity X"));
        canTpdo5BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo5BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo5BCombo->addItem(QString("Pressure"));
        canTpdo5BCombo->addItem(QString("Altitude"));
        canTpdo5BCombo->addItem(QString("Temperature"));
        connect(canTpdo5BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 11"), l, 0);
        gl4->addWidget(canTpdo6ACombo = new QComboBox(), l, 1); ++l;
        canTpdo6ACombo->addItem(QString("Gyroscope X"));
        canTpdo6ACombo->addItem(QString("Gyroscope Y"));
        canTpdo6ACombo->addItem(QString("Gyroscope Z"));
        canTpdo6ACombo->addItem(QString("Euler angle X"));
        canTpdo6ACombo->addItem(QString("Euler angle Y"));
        canTpdo6ACombo->addItem(QString("Euler angle Z"));
        canTpdo6ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo6ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo6ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo6ACombo->addItem(QString("Magnetometer X"));
        canTpdo6ACombo->addItem(QString("Magnetometer Y"));
        canTpdo6ACombo->addItem(QString("Magnetometer Z"));
        canTpdo6ACombo->addItem(QString("Quaternion W"));
        canTpdo6ACombo->addItem(QString("Quaternion X"));
        canTpdo6ACombo->addItem(QString("Quaternion Y"));
        canTpdo6ACombo->addItem(QString("Quaternion Z"));
        canTpdo6ACombo->addItem(QString("Accelerometer X"));
        canTpdo6ACombo->addItem(QString("Accelerometer Y"));
        canTpdo6ACombo->addItem(QString("Accelerometer Z"));
        canTpdo6ACombo->addItem(QString("Angular Velocity X"));
        canTpdo6ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo6ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo6ACombo->addItem(QString("Pressure"));
        canTpdo6ACombo->addItem(QString("Altitude"));
        canTpdo6ACombo->addItem(QString("Temperature"));
        connect(canTpdo6ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 12"), l, 0);
        gl4->addWidget(canTpdo6BCombo = new QComboBox(), l, 1); ++l;
        canTpdo6BCombo->addItem(QString("Gyroscope X"));
        canTpdo6BCombo->addItem(QString("Gyroscope Y"));
        canTpdo6BCombo->addItem(QString("Gyroscope Z"));
        canTpdo6BCombo->addItem(QString("Euler angle X"));
        canTpdo6BCombo->addItem(QString("Euler angle Y"));
        canTpdo6BCombo->addItem(QString("Euler angle Z"));
        canTpdo6BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo6BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo6BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo6BCombo->addItem(QString("Magnetometer X"));
        canTpdo6BCombo->addItem(QString("Magnetometer Y"));
        canTpdo6BCombo->addItem(QString("Magnetometer Z"));
        canTpdo6BCombo->addItem(QString("Quaternion W"));
        canTpdo6BCombo->addItem(QString("Quaternion X"));
        canTpdo6BCombo->addItem(QString("Quaternion Y"));
        canTpdo6BCombo->addItem(QString("Quaternion Z"));
        canTpdo6BCombo->addItem(QString("Accelerometer X"));
        canTpdo6BCombo->addItem(QString("Accelerometer Y"));
        canTpdo6BCombo->addItem(QString("Accelerometer Z"));
        canTpdo6BCombo->addItem(QString("Angular Velocity X"));
        canTpdo6BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo6BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo6BCombo->addItem(QString("Pressure"));
        canTpdo6BCombo->addItem(QString("Altitude"));
        canTpdo6BCombo->addItem(QString("Temperature"));
        connect(canTpdo6BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 13"), l, 0);
        gl4->addWidget(canTpdo7ACombo = new QComboBox(), l, 1); ++l;
        canTpdo7ACombo->addItem(QString("Gyroscope X"));
        canTpdo7ACombo->addItem(QString("Gyroscope Y"));
        canTpdo7ACombo->addItem(QString("Gyroscope Z"));
        canTpdo7ACombo->addItem(QString("Euler angle X"));
        canTpdo7ACombo->addItem(QString("Euler angle Y"));
        canTpdo7ACombo->addItem(QString("Euler angle Z"));
        canTpdo7ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo7ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo7ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo7ACombo->addItem(QString("Magnetometer X"));
        canTpdo7ACombo->addItem(QString("Magnetometer Y"));
        canTpdo7ACombo->addItem(QString("Magnetometer Z"));
        canTpdo7ACombo->addItem(QString("Quaternion W"));
        canTpdo7ACombo->addItem(QString("Quaternion X"));
        canTpdo7ACombo->addItem(QString("Quaternion Y"));
        canTpdo7ACombo->addItem(QString("Quaternion Z"));
        canTpdo7ACombo->addItem(QString("Accelerometer X"));
        canTpdo7ACombo->addItem(QString("Accelerometer Y"));
        canTpdo7ACombo->addItem(QString("Accelerometer Z"));
        canTpdo7ACombo->addItem(QString("Angular Velocity X"));
        canTpdo7ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo7ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo7ACombo->addItem(QString("Pressure"));
        canTpdo7ACombo->addItem(QString("Altitude"));
        canTpdo7ACombo->addItem(QString("Temperature"));
        connect(canTpdo7ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 14"), l, 0);
        gl4->addWidget(canTpdo7BCombo = new QComboBox(), l, 1); ++l;
        canTpdo7BCombo->addItem(QString("Gyroscope X"));
        canTpdo7BCombo->addItem(QString("Gyroscope Y"));
        canTpdo7BCombo->addItem(QString("Gyroscope Z"));
        canTpdo7BCombo->addItem(QString("Euler angle X"));
        canTpdo7BCombo->addItem(QString("Euler angle Y"));
        canTpdo7BCombo->addItem(QString("Euler angle Z"));
        canTpdo7BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo7BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo7BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo7BCombo->addItem(QString("Magnetometer X"));
        canTpdo7BCombo->addItem(QString("Magnetometer Y"));
        canTpdo7BCombo->addItem(QString("Magnetometer Z"));
        canTpdo7BCombo->addItem(QString("Quaternion W"));
        canTpdo7BCombo->addItem(QString("Quaternion X"));
        canTpdo7BCombo->addItem(QString("Quaternion Y"));
        canTpdo7BCombo->addItem(QString("Quaternion Z"));
        canTpdo7BCombo->addItem(QString("Accelerometer X"));
        canTpdo7BCombo->addItem(QString("Accelerometer Y"));
        canTpdo7BCombo->addItem(QString("Accelerometer Z"));
        canTpdo7BCombo->addItem(QString("Angular Velocity X"));
        canTpdo7BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo7BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo7BCombo->addItem(QString("Pressure"));
        canTpdo7BCombo->addItem(QString("Altitude"));
        canTpdo7BCombo->addItem(QString("Temperature"));
        connect(canTpdo7BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 15"), l, 0);
        gl4->addWidget(canTpdo8ACombo = new QComboBox(), l, 1); ++l;
        canTpdo8ACombo->addItem(QString("Gyroscope X"));
        canTpdo8ACombo->addItem(QString("Gyroscope Y"));
        canTpdo8ACombo->addItem(QString("Gyroscope Z"));
        canTpdo8ACombo->addItem(QString("Euler angle X"));
        canTpdo8ACombo->addItem(QString("Euler angle Y"));
        canTpdo8ACombo->addItem(QString("Euler angle Z"));
        canTpdo8ACombo->addItem(QString("Lin. acceleration X"));
        canTpdo8ACombo->addItem(QString("Lin. acceleration Y"));
        canTpdo8ACombo->addItem(QString("Lin. acceleration Z"));
        canTpdo8ACombo->addItem(QString("Magnetometer X"));
        canTpdo8ACombo->addItem(QString("Magnetometer Y"));
        canTpdo8ACombo->addItem(QString("Magnetometer Z"));
        canTpdo8ACombo->addItem(QString("Quaternion W"));
        canTpdo8ACombo->addItem(QString("Quaternion X"));
        canTpdo8ACombo->addItem(QString("Quaternion Y"));
        canTpdo8ACombo->addItem(QString("Quaternion Z"));
        canTpdo8ACombo->addItem(QString("Accelerometer X"));
        canTpdo8ACombo->addItem(QString("Accelerometer Y"));
        canTpdo8ACombo->addItem(QString("Accelerometer Z"));
        canTpdo8ACombo->addItem(QString("Angular Velocity X"));
        canTpdo8ACombo->addItem(QString("Angular Velocity Y"));
        canTpdo8ACombo->addItem(QString("Angular Velocity Z"));
        canTpdo8ACombo->addItem(QString("Pressure"));
        canTpdo8ACombo->addItem(QString("Altitude"));
        canTpdo8ACombo->addItem(QString("Temperature"));
        connect(canTpdo8ACombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));

        gl4->addWidget(new QLabel("Channel 16"), l, 0);
        gl4->addWidget(canTpdo8BCombo = new QComboBox(), l, 1); ++l;
        canTpdo8BCombo->addItem(QString("Gyroscope X"));
        canTpdo8BCombo->addItem(QString("Gyroscope Y"));
        canTpdo8BCombo->addItem(QString("Gyroscope Z"));
        canTpdo8BCombo->addItem(QString("Euler angle X"));
        canTpdo8BCombo->addItem(QString("Euler angle Y"));
        canTpdo8BCombo->addItem(QString("Euler angle Z"));
        canTpdo8BCombo->addItem(QString("Lin. acceleration X"));
        canTpdo8BCombo->addItem(QString("Lin. acceleration Y"));
        canTpdo8BCombo->addItem(QString("Lin. acceleration Z"));
        canTpdo8BCombo->addItem(QString("Magnetometer X"));
        canTpdo8BCombo->addItem(QString("Magnetometer Y"));
        canTpdo8BCombo->addItem(QString("Magnetometer Z"));
        canTpdo8BCombo->addItem(QString("Quaternion W"));
        canTpdo8BCombo->addItem(QString("Quaternion X"));
        canTpdo8BCombo->addItem(QString("Quaternion Y"));
        canTpdo8BCombo->addItem(QString("Quaternion Z"));
        canTpdo8BCombo->addItem(QString("Accelerometer X"));
        canTpdo8BCombo->addItem(QString("Accelerometer Y"));
        canTpdo8BCombo->addItem(QString("Accelerometer Z"));
        canTpdo8BCombo->addItem(QString("Angular Velocity X"));
        canTpdo8BCombo->addItem(QString("Angular Velocity Y"));
        canTpdo8BCombo->addItem(QString("Angular Velocity Z"));
        canTpdo8BCombo->addItem(QString("Pressure"));
        canTpdo8BCombo->addItem(QString("Altitude"));
        canTpdo8BCombo->addItem(QString("Temperature"));
        connect(canTpdo8BCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanMapping(int)));
    }

    l = 0;
    selectedDataGl->addWidget(new QLabel("LPBUS data mode:"), l, 0);
    selectedDataGl->addWidget(lpBusDataModeCombo = new QComboBox(), l, 1); ++l;
    lpBusDataModeCombo->addItem(QString("32-bit floating point"));
    lpBusDataModeCombo->addItem(QString("16-bit integer"));
    connect(lpBusDataModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLpBusDataMode(int)));

    selectedDataGl->addWidget(new QLabel("Enabled data:"), l, 0);
    selectedDataGl->addWidget(selectAcc = new QCheckBox("Raw accelerometer"), l, 1); ++l;
    selectedDataGl->addWidget(selectMag = new QCheckBox("Raw magnetometer"), l, 1); ++l;
    selectedDataGl->addWidget(selectGyro = new QCheckBox("Raw gyroscope"), l, 1); ++l;
    selectedDataGl->addWidget(selectAngularVelocity = new QCheckBox("Angular velocity"), l, 1); ++l;
    selectedDataGl->addWidget(selectQuaternion = new QCheckBox("Quaternion"), l, 1); ++l;
    selectedDataGl->addWidget(selectEuler = new QCheckBox("Euler angle"), l, 1); ++l;
    selectedDataGl->addWidget(selectLinAcc = new QCheckBox("Lin. acceleration"), l, 1); ++l;
    selectedDataGl->addWidget(selectPressure = new QCheckBox("Bar. pressure"), l, 1); ++l;
    selectedDataGl->addWidget(selectAltitude = new QCheckBox("Altitude"), l, 1); ++l;
    selectedDataGl->addWidget(selectTemperature = new QCheckBox("Temperature"), l, 1); ++l;

    connect(selectAcc, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAcc(int)));
    connect(selectMag, SIGNAL(stateChanged(int)), this, SLOT(updateSelectMag(int)));
    connect(selectGyro, SIGNAL(stateChanged(int)), this, SLOT(updateSelectGyro(int)));
    connect(selectQuaternion, SIGNAL(stateChanged(int)), this, SLOT(updateSelectQuaternion(int)));
    connect(selectEuler, SIGNAL(stateChanged(int)), this, SLOT(updateSelectEuler(int)));
    connect(selectLinAcc, SIGNAL(stateChanged(int)), this, SLOT(updateSelectLinAcc(int)));
    connect(selectPressure, SIGNAL(stateChanged(int)), this, SLOT(updateSelectPressure(int)));
    connect(selectAltitude, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAltitude(int)));
    connect(selectTemperature, SIGNAL(stateChanged(int)), this, SLOT(updateSelectTemperature(int)));
    connect(selectAngularVelocity, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAngularVelocity(int)));

    treeItem = new QTreeWidgetItem(tree, QStringList(QString("Sensor")));

    QWidget *w5 = new QWidget();
    w5->setLayout(gl5);
    QTreeWidgetItem* statusTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Status")));
    QTreeWidgetItem* statusTreeWidget = new QTreeWidgetItem(statusTreeItem);
    statusTreeItem->setExpanded(true);
    tree->setItemWidget(statusTreeWidget, 0, w5);

    QWidget *w0 = new QWidget();
    w0->setLayout(gl);
    QTreeWidgetItem* connectionTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("ID / sampling rate")));
    QTreeWidgetItem* connectionTreeWidget = new QTreeWidgetItem(connectionTreeItem);
    tree->setItemWidget(connectionTreeWidget, 0, w0);

    QWidget *w1 = new QWidget();
    w1->setLayout(gl1);
    QTreeWidgetItem* rangeTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Range")));
    QTreeWidgetItem* rangeTreeWidget = new QTreeWidgetItem(rangeTreeItem);
    tree->setItemWidget(rangeTreeWidget, 0, w1);

    QWidget *w2 = new QWidget();
    w2->setLayout(gl2);
    QTreeWidgetItem* filterTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Filter")));
    QTreeWidgetItem* filterTreeWidget = new QTreeWidgetItem(filterTreeItem);
    tree->setItemWidget(filterTreeWidget, 0, w2);

    if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        QWidget *w5 = new QWidget();
        w5->setLayout(gl6);
        QTreeWidgetItem* uartTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("UART (RS-232/TTL)")));
        QTreeWidgetItem* uartTreeWidget = new QTreeWidgetItem(uartTreeItem);
        tree->setItemWidget(uartTreeWidget, 0, w5);
    }

    if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        QWidget *w4 = new QWidget();
        w4->setLayout(gl4);
        QTreeWidgetItem* canTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("CAN bus")));
        QTreeWidgetItem* canTreeWidget = new QTreeWidgetItem(canTreeItem);
        tree->setItemWidget(canTreeWidget, 0, w4);
    }

    QWidget *w3 = new QWidget();
    w3->setLayout(selectedDataGl);
    QTreeWidgetItem* dataTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Data")));
    QTreeWidgetItem* dataTreeWidget = new QTreeWidgetItem(dataTreeItem);
    tree->setItemWidget(dataTreeWidget, 0, w3);

    heaveMotionEnabled = false;

    updateData();
}

void SensorGuiContainer::setupSensorGuiContainer()
{
    if (containerSetup) return;

    bool legacyFirmware = isFirmwareVersionLessThanEqualTo(2, 0, 5);
    if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        if (legacyFirmware)
        {
            baudRateCombo->addItem(QString("19200 bps"));
            baudRateCombo->addItem(QString("57600 bps"));
            baudRateCombo->addItem(QString("115200 bps"));
            baudRateCombo->addItem(QString("921600 bps"));
            baudRateCombo->setCurrentIndex(2);
        }
        else
        {
            baudRateCombo->addItem(QString("19200 bps"));
            baudRateCombo->addItem(QString("38400 bps"));
            baudRateCombo->addItem(QString("57600 bps"));
            baudRateCombo->addItem(QString("115200 bps"));
            baudRateCombo->addItem(QString("230400 bps"));
            baudRateCombo->addItem(QString("256000 bps"));
            baudRateCombo->addItem(QString("460800 bps"));
            baudRateCombo->addItem(QString("921600 bps"));
            baudRateCombo->setCurrentIndex(3);

        }
        connect(baudRateCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateBaudRateIndex(int)));

        uartFormatCombo->addItem(QString("LPBUS (binary)"));
        uartFormatCombo->addItem(QString("ASCII (CSV)"));
        connect(uartFormatCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateUartFormatIndex(int)));
    }

    legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);
    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        magRangeCombo->addItem(QString("4 Gauss"));
        magRangeCombo->addItem(QString("8 Gauss"));
        magRangeCombo->addItem(QString("12 Gauss"));
        magRangeCombo->addItem(QString("16 Gauss"));
    }
    else
    {
        magRangeCombo->addItem(QString("130 uT"));
        magRangeCombo->addItem(QString("190 uT"));
        magRangeCombo->addItem(QString("250 uT"));
        magRangeCombo->addItem(QString("400 uT"));
        magRangeCombo->addItem(QString("560 uT"));
        magRangeCombo->addItem(QString("810 uT"));
    }
    connect(magRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateMagRange(int)));

    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        gyrRangeCombo->addItem(QString("125 dps"));
        gyrRangeCombo->addItem(QString("245 dps"));
        gyrRangeCombo->addItem(QString("500 dps"));
        gyrRangeCombo->addItem(QString("1000 dps"));
        gyrRangeCombo->addItem(QString("2000 dps"));
    }
    else
    {
        gyrRangeCombo->addItem(QString("250 dps"));
        gyrRangeCombo->addItem(QString("500 dps"));
        gyrRangeCombo->addItem(QString("2000 dps"));
    }
    connect(gyrRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrRange(int)));

    containerSetup = true;
}

void SensorGuiContainer::checkOptionalFeatures(void) {
    int i;

    if (!containerSetup) return;

    sensor->getConfigurationPrm(PRM_HEAVEMOTION_ENABLED, &i);

    if (i == SELECT_HEAVEMOTION_ENABLED && heaveMotionEnabled == false) {
        selectedDataGl->addWidget(selectHeaveMotion = new QCheckBox("Heave motion"), selectedDataGl->rowCount() + 1, 1);

        connect(selectHeaveMotion, SIGNAL(stateChanged(int)), this, SLOT(updateSelectHeaveMotion(int)));

        if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
            canTpdo1ACombo->addItem(QString("Heave height"));
            canTpdo1BCombo->addItem(QString("Heave height"));
            canTpdo2ACombo->addItem(QString("Heave height"));
            canTpdo2BCombo->addItem(QString("Heave height"));
            canTpdo3ACombo->addItem(QString("Heave height"));
            canTpdo3BCombo->addItem(QString("Heave height"));
            canTpdo4ACombo->addItem(QString("Heave height"));
            canTpdo4BCombo->addItem(QString("Heave height"));
            canTpdo5ACombo->addItem(QString("Heave height"));
            canTpdo5BCombo->addItem(QString("Heave height"));
            canTpdo6ACombo->addItem(QString("Heave height"));
            canTpdo6BCombo->addItem(QString("Heave height"));
            canTpdo7ACombo->addItem(QString("Heave height"));
            canTpdo7BCombo->addItem(QString("Heave height"));
            canTpdo8ACombo->addItem(QString("Heave height"));
            canTpdo8BCombo->addItem(QString("Heave height"));
        }

        heaveMotionEnabled = true;
    }
}

void SensorGuiContainer::updateData(void) {
    int i;
    std::string s;
    char cStr[64];
    int omId;
    int a[32];
    bool legacyFirmware;

    if (!containerSetup) return;

    checkOptionalFeatures();

    indexItem->blockSignals(true);
    parameterSetCombo->blockSignals(true);
    thresholdEnableCombo->blockSignals(true);
    accRangeCombo->blockSignals(true);
    magRangeCombo->blockSignals(true);
    gyrRangeCombo->blockSignals(true);
    filterModeCombo->blockSignals(true);
    selectQuaternion->blockSignals(true);
    selectEuler->blockSignals(true);
    selectLinAcc->blockSignals(true);
    selectGyro->blockSignals(true);
    selectAcc->blockSignals(true);
    selectMag->blockSignals(true);
    selectPressure->blockSignals(true);
    selectAltitude->blockSignals(true);
    selectTemperature->blockSignals(true);
    lowPassCombo->blockSignals(true);
    linAccCompModeCombo->blockSignals(true);
    centriCompModeCombo->blockSignals(true);
    lpBusDataModeCombo->blockSignals(true);

    if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        // canProtocolCombo->blockSignals(false);	
        canBaudrateCombo->blockSignals(true);
        canHeartbeatCombo->blockSignals(true);
        canTpdo1ACombo->blockSignals(true);
        canTpdo1BCombo->blockSignals(true);
        canTpdo2ACombo->blockSignals(true);
        canTpdo2BCombo->blockSignals(true);
        canTpdo3ACombo->blockSignals(true);
        canTpdo3BCombo->blockSignals(true);
        canTpdo4ACombo->blockSignals(true);
        canTpdo4BCombo->blockSignals(true);
        canTpdo5ACombo->blockSignals(true);
        canTpdo5BCombo->blockSignals(true);
        canTpdo6ACombo->blockSignals(true);
        canTpdo6BCombo->blockSignals(true);
        canTpdo7ACombo->blockSignals(true);
        canTpdo7BCombo->blockSignals(true);
        canTpdo8ACombo->blockSignals(true);
        canTpdo8BCombo->blockSignals(true);
        canChannelModeCombo->blockSignals(true);
        canPointModeCombo->blockSignals(true);
        canStartIdSpin->blockSignals(true);
    }

    sensor->getConfigurationPrm(PRM_OPENMAT_ID, &omId);
    sensor->getConfigurationPrm(PRM_DEVICE_ID, cStr);
    // sensor->getConfigurationPrm(PRM_DEVICE_TYPE, &i);

    switch (deviceType) {
    case DEVICE_LPMS_B:
        treeItem->setText(0, QString("LPMS-B ID=%1 (").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_BLE:
        treeItem->setText(0, QString("LPMS-BLE ID=%1 (").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_U:
        treeItem->setText(0, QString("LPMS-U ID=%1 (USB ID: ").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_C:
        treeItem->setText(0, QString("LPMS-CU ID=%1 (CAN ID: ").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_B2:
        treeItem->setText(0, QString("LPMS-B2 ID=%1 (").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_U2:
        treeItem->setText(0, QString("LPMS-U2 ID=%1 (").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_C2:
        treeItem->setText(0, QString("LPMS-CU2 ID=%1 (CAN ID: ").arg(omId) + QString(cStr) + ")");
        break;

    case DEVICE_LPMS_RS232:
        treeItem->setText(0, QString("LPMS-RS232 ID=%1 (COM PORT: ").arg(omId) + QString(cStr) + ")");
        break;
    }

    addressItem->setText(QString(cStr));

    sensor->getConfigurationPrm(PRM_OPENMAT_ID, &i);
    indexItem->setCurrentIndex(i);

    if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        sensor->getConfigurationPrm(PRM_UART_BAUDRATE, &i);

        baudRateCombo->setCurrentIndex(i);
        /*
        legacyFirmware = isFirmwareVersionLessThanEqualTo(2, 0, 5);
        if (legacyFirmware)
        {
            baudRateCombo->setCurrentIndex(i);
        }
        else
        {
            switch (i) {
                case SELECT_LPMS_UART_BAUDRATE_19200:
                    baudRateCombo->setCurrentIndex(0);
                break;

                case SELECT_LPMS_UART_BAUDRATE_38400:
                    baudRateCombo->setCurrentIndex(1);
                break;

                case SELECT_LPMS_UART_BAUDRATE_57600:
                    baudRateCombo->setCurrentIndex(2);
                break;

                case SELECT_LPMS_UART_BAUDRATE_115200:
                    baudRateCombo->setCurrentIndex(3);
                break;

                case SELECT_LPMS_UART_BAUDRATE_230400:
                    baudRateCombo->setCurrentIndex(4);
                break;

                case SELECT_LPMS_UART_BAUDRATE_256000:
                    baudRateCombo->setCurrentIndex(5);
                break;

                case SELECT_LPMS_UART_BAUDRATE_460800:
                    baudRateCombo->setCurrentIndex(6);
                break;

                case SELECT_LPMS_UART_BAUDRATE_921600:
                    baudRateCombo->setCurrentIndex(7);
                break;
            }
        }
        */
        sensor->getConfigurationPrm(PRM_UART_FORMAT, &i);
        switch (i) {
        case SELECT_LPMS_UART_FORMAT_LPBUS:
            uartFormatCombo->setCurrentIndex(0);
            break;

        case SELECT_LPMS_UART_FORMAT_CSV:
            uartFormatCombo->setCurrentIndex(1);
            break;
        }
    }

    sensor->getConfigurationPrm(PRM_FIRMWARE_VERSION, cStr);
    firmwareItem->setText(QString(cStr));

    sensor->getConfigurationPrm(PRM_PARAMETER_SET, &i);
    switch (i) {
    case SELECT_IMU_SLOW:
        parameterSetCombo->setCurrentIndex(0);
        break;

    case SELECT_IMU_MEDIUM:
        parameterSetCombo->setCurrentIndex(1);
        break;

    case SELECT_IMU_FAST:
        parameterSetCombo->setCurrentIndex(2);
        break;

    case SELECT_IMU_DYNAMIC:
        parameterSetCombo->setCurrentIndex(3);
        break;
    }

    sensor->getConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, &i);
    switch (i) {
    case SELECT_IMU_GYR_THRESH_ENABLED:
        thresholdEnableCombo->setCurrentIndex(1);
        break;

    case SELECT_IMU_GYR_THRESH_DISABLED:
        thresholdEnableCombo->setCurrentIndex(0);
        break;
    }

    sensor->getConfigurationPrm(PRM_GYR_AUTOCALIBRATION, &i);
    switch (i) {
    case SELECT_GYR_AUTOCALIBRATION_ENABLED:
        gyrAutocalibrationCombo->setCurrentIndex(1);
        break;

    case SELECT_GYR_AUTOCALIBRATION_DISABLED:
        gyrAutocalibrationCombo->setCurrentIndex(0);
        break;
    }

    sensor->getConfigurationPrm(PRM_FILTER_MODE, &i);
    switch (i) {
    case SELECT_FM_GYRO_ONLY:
        filterModeCombo->setCurrentIndex(0);
        break;

    case SELECT_FM_GYRO_ACC:
        filterModeCombo->setCurrentIndex(1);
        break;

    case SELECT_FM_GYRO_ACC_MAG:
        filterModeCombo->setCurrentIndex(2);
        break;

    case SELECT_FM_MADGWICK_GYRO_ACC:
        filterModeCombo->setCurrentIndex(3);
        break;

    case SELECT_FM_MADGWICK_GYRO_ACC_MAG:
        filterModeCombo->setCurrentIndex(4);
        break;
    }

    switch (sensor->getSensorStatus()) {
    case 0:
        runningItem->setText("<font color='red'>stopped</font>");
        break;

    case 1:
        runningItem->setText("<font color='green'>started</font>");
        break;

    case 2:
        runningItem->setText("<font color='blue'>initializing..</font>");
        break;

    case 3:
        runningItem->setText("<font color='red'>error..</font>");
        break;

    case 4:
        runningItem->setText("<font color='blue'>uploading..</font>");
        break;

    default:
        runningItem->setText("<font color='red'>stopped</font>");
        break;
    }

    switch (sensor->getConnectionStatus()) {
    case 0:
        break;

    case 1:
        statusItem->setText("<font color='green'>OK</font>");
        break;

    case 2:
        statusItem->setText("<font color='blue'>in progress</font>");
        break;

    case 3:
        statusItem->setText("<font color='red'>failed</font>");
        break;

    case 4:
        statusItem->setText("<font color='red'>interrupted</font>");
        break;
    }

    static int updateBatteryCount = 8000;
    if (deviceType == DEVICE_LPMS_B2 &&
        sensor->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
        updateBatteryCount++ > 10000)
    {
        updateBatteryCount = 0;
        if (sensor->getSensorStatus() == SENSOR_STATUS_RUNNING)
            batteryItem->setText(QString::number(sensor->getBatteryLevel()) + "%");
    }

    legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);
    sensor->getConfigurationPrm(PRM_GYR_RANGE, &i);

    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        switch (i) {
        case SELECT_GYR_RANGE_125DPS:
            if (gyrRangeCombo->currentIndex() != 0)
                gyrRangeCombo->setCurrentIndex(0);
            break;

        case SELECT_GYR_RANGE_245DPS:
            if (gyrRangeCombo->currentIndex() != 1)
                gyrRangeCombo->setCurrentIndex(1);
            break;

        case SELECT_GYR_RANGE_500DPS:
            if (gyrRangeCombo->currentIndex() != 2)
                gyrRangeCombo->setCurrentIndex(2);
            break;

        case SELECT_GYR_RANGE_1000DPS:
            if (gyrRangeCombo->currentIndex() != 3)
                gyrRangeCombo->setCurrentIndex(3);
            break;

        case SELECT_GYR_RANGE_2000DPS:
            if (gyrRangeCombo->currentIndex() != 4)
                gyrRangeCombo->setCurrentIndex(4);
            break;
        }
    }
    else
    {
        switch (i) {
        case SELECT_GYR_RANGE_250DPS:
            if (gyrRangeCombo->currentIndex() != 0)
            gyrRangeCombo->setCurrentIndex(0);
            break;

        case SELECT_GYR_RANGE_500DPS:
            if (gyrRangeCombo->currentIndex() != 1)
            gyrRangeCombo->setCurrentIndex(1);
            break;

        case SELECT_GYR_RANGE_2000DPS:
            if (gyrRangeCombo->currentIndex() != 2)
            gyrRangeCombo->setCurrentIndex(2);
            break;
        }
    }
    sensor->getConfigurationPrm(PRM_ACC_RANGE, &i);

    switch (i) {
    case SELECT_ACC_RANGE_2G:
        accRangeCombo->setCurrentIndex(0);
        break;

    case SELECT_ACC_RANGE_4G:
        accRangeCombo->setCurrentIndex(1);
        break;

    case SELECT_ACC_RANGE_8G:
        accRangeCombo->setCurrentIndex(2);
        break;

    case SELECT_ACC_RANGE_16G:
        accRangeCombo->setCurrentIndex(3);
        break;
    }

    sensor->getConfigurationPrm(PRM_MAG_RANGE, &i);
    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        switch (i) {
        case SELECT_MAG_RANGE_4GAUSS:
            if (magRangeCombo->currentIndex() != 0)
                magRangeCombo->setCurrentIndex(0);
            break;
        case SELECT_MAG_RANGE_8GAUSS:
            if (magRangeCombo->currentIndex() != 1)
                magRangeCombo->setCurrentIndex(1);
            break;
        case SELECT_MAG_RANGE_12GAUSS:
            if (magRangeCombo->currentIndex() != 2)
                magRangeCombo->setCurrentIndex(2);
            break;
        case SELECT_MAG_RANGE_16GAUSS:
            if (magRangeCombo->currentIndex() != 3)
                magRangeCombo->setCurrentIndex(3);
            break;
        }
    }
    else
    {
        switch (i) {
        case SELECT_MAG_RANGE_130UT:
            if (magRangeCombo->currentIndex() != 0)
                magRangeCombo->setCurrentIndex(0);
            break;

        case SELECT_MAG_RANGE_190UT:
            if (magRangeCombo->currentIndex() != 1)
                magRangeCombo->setCurrentIndex(1);
            break;

        case SELECT_MAG_RANGE_250UT:
            if (magRangeCombo->currentIndex() != 2)
                magRangeCombo->setCurrentIndex(2);
            break;

        case SELECT_MAG_RANGE_400UT:
            if (magRangeCombo->currentIndex() != 3)
                magRangeCombo->setCurrentIndex(3);
            break;

        case SELECT_MAG_RANGE_560UT:
            if (magRangeCombo->currentIndex() != 4)
                magRangeCombo->setCurrentIndex(4);
            break;

        case SELECT_MAG_RANGE_810UT:
            if (magRangeCombo->currentIndex() != 5)
                magRangeCombo->setCurrentIndex(5);
            break;
        }
    }

    if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        sensor->getConfigurationPrm(PRM_CAN_BAUDRATE, &i);
        switch (i) {
        case SELECT_CAN_BAUDRATE_1000KBPS:
            canBaudrateCombo->setCurrentIndex(0);
            break;

        case SELECT_CAN_BAUDRATE_500KBPS:
            canBaudrateCombo->setCurrentIndex(1);
            break;

        case SELECT_CAN_BAUDRATE_250KBPS:
            canBaudrateCombo->setCurrentIndex(2);
            break;

        case SELECT_CAN_BAUDRATE_125KBPS:
            canBaudrateCombo->setCurrentIndex(3);
            break;
        }

        sensor->getConfigurationPrm(PRM_CAN_MAPPING, a);
        canTpdo1ACombo->setCurrentIndex(a[0]);
        canTpdo1BCombo->setCurrentIndex(a[1]);
        canTpdo2ACombo->setCurrentIndex(a[2]);
        canTpdo2BCombo->setCurrentIndex(a[3]);
        canTpdo3ACombo->setCurrentIndex(a[4]);
        canTpdo3BCombo->setCurrentIndex(a[5]);
        canTpdo4ACombo->setCurrentIndex(a[6]);
        canTpdo4BCombo->setCurrentIndex(a[7]);
        canTpdo5ACombo->setCurrentIndex(a[8]);
        canTpdo5BCombo->setCurrentIndex(a[9]);
        canTpdo6ACombo->setCurrentIndex(a[10]);
        canTpdo6BCombo->setCurrentIndex(a[11]);
        canTpdo7ACombo->setCurrentIndex(a[12]);
        canTpdo7BCombo->setCurrentIndex(a[13]);
        canTpdo8ACombo->setCurrentIndex(a[14]);
        canTpdo8BCombo->setCurrentIndex(a[15]);

        sensor->getConfigurationPrm(PRM_CAN_HEARTBEAT, &i);
        switch (i) {
        case SELECT_LPMS_CAN_HEARTBEAT_005:
            canHeartbeatCombo->setCurrentIndex(0);
            break;

        case SELECT_LPMS_CAN_HEARTBEAT_010:
            canHeartbeatCombo->setCurrentIndex(1);
            break;

        case SELECT_LPMS_CAN_HEARTBEAT_020:
            canHeartbeatCombo->setCurrentIndex(2);
            break;

        case SELECT_LPMS_CAN_HEARTBEAT_050:
            canHeartbeatCombo->setCurrentIndex(3);
            break;

        case SELECT_LPMS_CAN_HEARTBEAT_100:
            canHeartbeatCombo->setCurrentIndex(4);
            break;
        }

        sensor->getConfigurationPrm(PRM_CAN_START_ID, &i);
        canStartIdSpin->setValue(i);

        sensor->getConfigurationPrm(PRM_CAN_POINT_MODE, &i);
        switch (i) {
        case SELECT_CAN_POINT_MODE_FLOATING:
            canPointModeCombo->setCurrentIndex(0);
            break;

        case SELECT_CAN_POINT_MODE_FIXED:
            canPointModeCombo->setCurrentIndex(1);
            break;
        }

        sensor->getConfigurationPrm(PRM_CAN_CHANNEL_MODE, &i);
        switch (i) {
        case SELECT_CAN_CHANNEL_MODE_CANOPEN:
            canChannelModeCombo->setCurrentIndex(0);
            break;

        case SELECT_CAN_CHANNEL_MODE_SEQUENTIAL:
            canChannelModeCombo->setCurrentIndex(1);
            break;
        }
    }

    sensor->getConfigurationPrm(PRM_SAMPLING_RATE, &i);
    switch (i) {
    case SELECT_STREAM_FREQ_5HZ:
        samplingRateCombo->setCurrentIndex(0);
        break;

    case SELECT_STREAM_FREQ_10HZ:
        samplingRateCombo->setCurrentIndex(1);
        break;

    case SELECT_STREAM_FREQ_25HZ:
    case 30: // Firmware 1.3.4 legacy bug fix
        samplingRateCombo->setCurrentIndex(2);
        break;

    case SELECT_STREAM_FREQ_50HZ:
        samplingRateCombo->setCurrentIndex(3);
        break;

    case SELECT_STREAM_FREQ_100HZ:
        samplingRateCombo->setCurrentIndex(4);
        break;

    case SELECT_STREAM_FREQ_200HZ:
        samplingRateCombo->setCurrentIndex(5);
        break;

    case SELECT_STREAM_FREQ_400HZ:
    case 300: // Firmware 1.3.4 legacy bug fix
        samplingRateCombo->setCurrentIndex(6);
        break;

    default:
        samplingRateCombo->setCurrentIndex(6);
        break;
    }

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &i);
    if ((i & SELECT_LPMS_ACC_OUTPUT_ENABLED) != 0) {
        selectAcc->setCheckState(Qt::Checked);
    }
    else {
        selectAcc->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_MAG_OUTPUT_ENABLED) != 0) {
        selectMag->setCheckState(Qt::Checked);
    }
    else {
        selectMag->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_GYRO_OUTPUT_ENABLED) != 0) {
        selectGyro->setCheckState(Qt::Checked);
    }
    else {
        selectGyro->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_QUAT_OUTPUT_ENABLED) != 0) {
        selectQuaternion->setCheckState(Qt::Checked);
    }
    else {
        selectQuaternion->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_EULER_OUTPUT_ENABLED) != 0) {
        selectEuler->setCheckState(Qt::Checked);
    }
    else {
        selectEuler->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_LINACC_OUTPUT_ENABLED) != 0) {
        selectLinAcc->setCheckState(Qt::Checked);
    }
    else {
        selectLinAcc->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
        selectPressure->setCheckState(Qt::Checked);
    }
    else {
        selectPressure->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
        selectAltitude->setCheckState(Qt::Checked);
    }
    else {
        selectAltitude->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
        selectTemperature->setCheckState(Qt::Checked);
    }
    else {
        selectTemperature->setCheckState(Qt::Unchecked);
    }

    if ((i & SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
        selectAngularVelocity->setCheckState(Qt::Checked);
    }
    else {
        selectAngularVelocity->setCheckState(Qt::Unchecked);
    }

    if (heaveMotionEnabled == true) {
        selectHeaveMotion->blockSignals(true);

        if ((i & SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
            selectHeaveMotion->setCheckState(Qt::Checked);
        }
        else {
            selectHeaveMotion->setCheckState(Qt::Unchecked);
        }

        selectHeaveMotion->blockSignals(false);
    }

    sensor->getConfigurationPrm(PRM_LOW_PASS, &i);
    switch (i) {
    case SELECT_LPMS_LP_OFF:
        lowPassCombo->setCurrentIndex(0);
        break;

    case SELECT_LPMS_LP_01:
        lowPassCombo->setCurrentIndex(1);
        break;

    case SELECT_LPMS_LP_005:
        lowPassCombo->setCurrentIndex(2);
        break;

    case SELECT_LPMS_LP_001:
        lowPassCombo->setCurrentIndex(3);
        break;

    case SELECT_LPMS_LP_0005:
        lowPassCombo->setCurrentIndex(4);
        break;

    case SELECT_LPMS_LP_0001:
        lowPassCombo->setCurrentIndex(5);
        break;
    }

    sensor->getConfigurationPrm(PRM_LIN_ACC_COMP_MODE, &i);
    switch (i) {
    case SELECT_LPMS_LIN_ACC_COMP_MODE_OFF:
        linAccCompModeCombo->setCurrentIndex(0);
        break;

    case SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK:
        linAccCompModeCombo->setCurrentIndex(1);
        break;

    case SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM:
        linAccCompModeCombo->setCurrentIndex(2);
        break;

    case SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG:
        linAccCompModeCombo->setCurrentIndex(3);
        break;

    case SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA:
        linAccCompModeCombo->setCurrentIndex(4);
        break;
    }

    sensor->getConfigurationPrm(PRM_CENTRI_COMP_MODE, &i);
    switch (i) {
    case SELECT_LPMS_CENTRI_COMP_MODE_OFF:
        centriCompModeCombo->setCurrentIndex(0);
        break;

    case SELECT_LPMS_CENTRI_COMP_MODE_ON:
        centriCompModeCombo->setCurrentIndex(1);
        break;
    }

    sensor->getConfigurationPrm(PRM_LPBUS_DATA_MODE, &i);
    switch (i) {
    case SELECT_LPMS_LPBUS_DATA_MODE_32:
        lpBusDataModeCombo->setCurrentIndex(0);
        break;

    case SELECT_LPMS_LPBUS_DATA_MODE_16:
        lpBusDataModeCombo->setCurrentIndex(1);
        break;
    }

    selectAcc->blockSignals(false);
    selectMag->blockSignals(false);
    selectQuaternion->blockSignals(false);
    selectEuler->blockSignals(false);
    selectLinAcc->blockSignals(false);
    selectGyro->blockSignals(false);
    lowPassCombo->blockSignals(false);
    indexItem->blockSignals(false);
    parameterSetCombo->blockSignals(false);
    thresholdEnableCombo->blockSignals(false);
    accRangeCombo->blockSignals(false);
    magRangeCombo->blockSignals(false);
    gyrRangeCombo->blockSignals(false);
    filterModeCombo->blockSignals(false);
    selectPressure->blockSignals(false);
    selectAltitude->blockSignals(false);
    selectTemperature->blockSignals(false);
    linAccCompModeCombo->blockSignals(false);
    centriCompModeCombo->blockSignals(false);
    lpBusDataModeCombo->blockSignals(false);

    if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        canBaudrateCombo->blockSignals(false);
        canHeartbeatCombo->blockSignals(false);
        canTpdo1ACombo->blockSignals(false);
        canTpdo1BCombo->blockSignals(false);
        canTpdo2ACombo->blockSignals(false);
        canTpdo2BCombo->blockSignals(false);
        canTpdo3ACombo->blockSignals(false);
        canTpdo3BCombo->blockSignals(false);
        canTpdo4ACombo->blockSignals(false);
        canTpdo4BCombo->blockSignals(false);
        canTpdo5ACombo->blockSignals(false);
        canTpdo5BCombo->blockSignals(false);
        canTpdo6ACombo->blockSignals(false);
        canTpdo6BCombo->blockSignals(false);
        canTpdo7ACombo->blockSignals(false);
        canTpdo7BCombo->blockSignals(false);
        canTpdo8ACombo->blockSignals(false);
        canTpdo8BCombo->blockSignals(false);
        canChannelModeCombo->blockSignals(false);
        canPointModeCombo->blockSignals(false);
        canStartIdSpin->blockSignals(false);
    }
}

void SensorGuiContainer::updateOpenMATIndex(int i)
{
    if (!containerSetup) return;

    sensor->setConfigurationPrm(PRM_OPENMAT_ID, indexItem->currentIndex());
}

void SensorGuiContainer::updateBaudRateIndex(int i)
{
    if (!containerSetup) return;

    sensor->setConfigurationPrm(PRM_UART_BAUDRATE, i);

    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(2, 0, 5);
    // if (legacyFirmware)
    // {
        // switch (baudRateCombo->currentIndex()) {
        // switch (i) {
        // case 0:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, 0);
            // break;
        // case 1:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, 1);
            // break;
        // case 2:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, 2);
            // break;
        // case 3:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, 3);
            // break;
        // }

    // }
    // else
    // {
        // switch (baudRateCombo->currentIndex()) {
        // case 0:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_19200);
            // break;

        // case 1:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_38400);
            // break;

        // case 2:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_57600);
            // break;

        // case 3:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_115200);
            // break;

        // case 4:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_230400);
            // break;

        // case 5:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_256000);
            // break;

        // case 6:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_460800);
            // break;

        // case 7:
            // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, SELECT_LPMS_UART_BAUDRATE_921600);
            // break;
        // }
    // }
}

void SensorGuiContainer::updateGyrThresholdEnable(int i)
{
    if (!containerSetup) return;

    switch (thresholdEnableCombo->currentIndex()) {
    case 1:
        sensor->setConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_ENABLED);
        break;

    case 0:
        sensor->setConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
        break;
    }
}

void SensorGuiContainer::updateGyrAutocalibration(int i)
{
    if (!containerSetup) return;

    switch (gyrAutocalibrationCombo->currentIndex()) {
    case 1:
        sensor->setConfigurationPrm(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
        break;

    case 0:
        sensor->setConfigurationPrm(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_DISABLED);
        break;
    }
}

void SensorGuiContainer::updateFilterPreset(int i)
{
    if (!containerSetup) return;

    switch (parameterSetCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_SLOW);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_MEDIUM);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_FAST);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_DYNAMIC);
        break;
    }
}

void SensorGuiContainer::updateFilterMode(int i)
{
    if (!containerSetup) return;

    switch (filterModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ONLY);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC_MAG);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_MADGWICK_GYRO_ACC);
        break;

    case 4:
        sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_MADGWICK_GYRO_ACC_MAG);
        break;
    }
}

void SensorGuiContainer::updateGyrRange(int i)
{
    if (!containerSetup) return;
    
    bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1,9,9);
    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        switch (gyrRangeCombo->currentIndex()) {
        case 0:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_125DPS);
            break;

        case 1:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_245DPS);
            break;

        case 2:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_500DPS);
            break;

        case 3:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_1000DPS);
            break;

        case 4:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_2000DPS);
            break;
        }

    } else {
        switch (gyrRangeCombo->currentIndex()) {
        case 0:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_250DPS);
            break;

        case 1:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_500DPS);
            break;

        case 2:
            sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_2000DPS);
            break;
        }
    }
}

void SensorGuiContainer::updateAccRange(int i)
{
    if (!containerSetup) return;

    switch (accRangeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_2G);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_4G);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_8G);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_16G);
        break;
    }
}

void SensorGuiContainer::updateMagRange(int i)
{
    if (!containerSetup)
        return;

    bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);
    if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    {
        switch (magRangeCombo->currentIndex()) {

        case 0:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_4GAUSS);
            break;
        case 1:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_8GAUSS);
            break;
        case 2:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_12GAUSS);
            break;
        case 3:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_16GAUSS);
            break;
        }
    }
    else
    {
        switch (magRangeCombo->currentIndex()) {
        case 0:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_130UT);
            break;

        case 1:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_190UT);
            break;

        case 2:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_250UT);
            break;

        case 3:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_400UT);
            break;

        case 4:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_560UT);
            break;

        case 5:
            sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_810UT);
            break;
        }
    }
}

void SensorGuiContainer::updateCanProtocol(int i)
{
}

void SensorGuiContainer::updateCanBaudrate(int i)
{
    if (!containerSetup)
        return;
    switch (canBaudrateCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_500KBPS);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_250KBPS);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS);
        break;
    }
}

void SensorGuiContainer::updateLowPass(int i)
{
    if (!containerSetup)
        return;
    switch (lowPassCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_OFF);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_01);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_005);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_001);
        break;

    case 4:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_0005);
        break;

    case 5:
        sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_0001);
        break;
    }
}

void SensorGuiContainer::updatesamplingRate(int i)
{
    if (!containerSetup)
        return;
    bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1,3,4);
    switch (samplingRateCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);
        break;

    case 2:
        if (legacyFirmware)
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, 30);
        else
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_25HZ);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_50HZ);
        break;

    case 4:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);
        break;

    case 5:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_200HZ);
        break;

    case 6:
        if (legacyFirmware)
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, 300);
        else
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_400HZ);
        break;
    }
}

SensorGuiContainer::~SensorGuiContainer(void)
{
}

LpmsSensorI* SensorGuiContainer::getSensor(void)
{
    return sensor;
}

void SensorGuiContainer::updateSelectGyro(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_GYRO_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectQuaternion(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectEuler(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_EULER_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectLinAcc(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_LINACC_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectAcc(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectMag(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectPressure(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectAltitude(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectTemperature(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectAngularVelocity(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateSelectHeaveMotion(int i)
{
    if (!containerSetup)
        return;
    int p = 0;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

void SensorGuiContainer::updateCanMapping(int i)
{
    if (!containerSetup)
        return;
    int a[32];

    a[0] = canTpdo1ACombo->currentIndex();
    a[1] = canTpdo1BCombo->currentIndex();
    a[2] = canTpdo2ACombo->currentIndex();
    a[3] = canTpdo2BCombo->currentIndex();
    a[4] = canTpdo3ACombo->currentIndex();
    a[5] = canTpdo3BCombo->currentIndex();
    a[6] = canTpdo4ACombo->currentIndex();
    a[7] = canTpdo4BCombo->currentIndex();
    a[8] = canTpdo5ACombo->currentIndex();
    a[9] = canTpdo5BCombo->currentIndex();
    a[10] = canTpdo6ACombo->currentIndex();
    a[11] = canTpdo6BCombo->currentIndex();
    a[12] = canTpdo7ACombo->currentIndex();
    a[13] = canTpdo7BCombo->currentIndex();
    a[14] = canTpdo8ACombo->currentIndex();
    a[15] = canTpdo8BCombo->currentIndex();

    sensor->setConfigurationPrm(PRM_CAN_MAPPING, a);
}

void SensorGuiContainer::updateCanHeartbeat(int i)
{
    if (!containerSetup)
        return;
    switch (canHeartbeatCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_005);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_010);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_020);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_050);
        break;

    case 4:
        sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_100);
        break;
    }
}

void SensorGuiContainer::updateLinAccCompMode(int i)
{
    if (!containerSetup)
        return;
    switch (linAccCompModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_OFF);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK);
        break;

    case 2:
        sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM);
        break;

    case 3:
        sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG);
        break;

    case 4:
        sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA);
        break;
    }
}

void SensorGuiContainer::updateCentriCompMode(int i)
{
    if (!containerSetup)
        return;
    switch (centriCompModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_OFF);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_ON);
        break;
    }
}

void SensorGuiContainer::updateCanStartId(int i)
{
    if (!containerSetup)
        return;
    sensor->setConfigurationPrm(PRM_CAN_START_ID, canStartIdSpin->value());
}

void SensorGuiContainer::updateCanPointMode(int i)
{
    if (!containerSetup)
        return;
    switch (canPointModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FLOATING);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FIXED);
        break;
    }
}

void SensorGuiContainer::updateCanChannelMode(int i)
{
    if (!containerSetup)
        return;
    switch (canChannelModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_CANOPEN);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_SEQUENTIAL);
        break;
    }
}

void SensorGuiContainer::updateLpBusDataMode(int i)
{
    if (!containerSetup)
        return;
    switch (lpBusDataModeCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_32);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_16);
        break;
    }
}

void SensorGuiContainer::updateUartFormatIndex(int i)
{
    if (!containerSetup)
        return;
    switch (uartFormatCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_UART_FORMAT, SELECT_LPMS_UART_FORMAT_LPBUS);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_UART_FORMAT, SELECT_LPMS_UART_FORMAT_CSV);
        break;
    }
}

bool SensorGuiContainer::isFirmwareVersionLessThanEqualTo(int i, int j, int k)
{
    // UGLY HACK
    // not valid if i,j,k > 9
    char cStr[64];
    sensor->getConfigurationPrm(PRM_FIRMWARE_VERSION, cStr);
    int target = i * 100 + j * 10 + k;
    int currentFirmware = (cStr[0] - '0') * 100 + (cStr[2] - '0') * 10 + (cStr[4] - '0');
    if (currentFirmware <= target)
        return true;
    return false;
}