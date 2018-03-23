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

SensorGuiContainer::SensorGuiContainer(LpemgSensorI* sensor, QTreeWidget* tree) :
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
    gl5->addWidget(new QLabel("Device MAC:"), l, 0); ++l;
    gl5->addWidget(new QLabel("Firmware version:"), l, 0); ++l;
    if (deviceType == DEVICE_LPMS_B2)
        gl5->addWidget(new QLabel("Battery:"), l, 0); ++l;


    l = 0;
    gl->addWidget(new QLabel("Sensor ID:"), l, 0); ++l;
    gl->addWidget(new QLabel("Transmission rate:"), l, 0); ++l;

    // l = 0;
    // if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        // gl6->addWidget(new QLabel("Baud rate:"), l, 0); ++l;
        // gl6->addWidget(new QLabel("Data format:"), l, 0); ++l;
    // }

    // l = 0;
    // gl1->addWidget(new QLabel("GYR range:"), l, 0); ++l;
    // gl1->addWidget(new QLabel("ACC range:"), l, 0); ++l;
    // gl1->addWidget(new QLabel("MAG range:"), l, 0); ++l;

    // l = 0;
    // gl2->addWidget(new QLabel("Filter mode:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("MAG correction:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("Lin. ACC correction:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("Rot. ACC correction:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("GYR threshhold:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("GYR autocalibration:"), l, 0); ++l;
    // gl2->addWidget(new QLabel("Low-pass filter:"), l, 0); ++l;

    l = 0;
    gl5->addWidget(statusItem = new QLabel(""), l, 1); ++l;
    gl5->addWidget(runningItem = new QLabel(""), l, 1); ++l;
    gl5->addWidget(addressItem = new QLabel(""), l, 1); ++l;
    gl5->addWidget(firmwareItem = new QLabel("n/a"), l, 1); ++l;
    if (deviceType == DEVICE_LPMS_B2)
        gl5->addWidget(batteryItem = new QLabel(""), l, 1); ++l;

    l = 0;
    gl->addWidget(indexItem = new QComboBox(), l, 1); ++l;
    gl->addWidget(samplingRateCombo = new QComboBox(), l, 1); ++l;

    // l = 0;
    // gl6->addWidget(baudRateCombo = new QComboBox(), l, 1); ++l;
    // gl6->addWidget(uartFormatCombo = new QComboBox(), l, 1); ++l;

    // l = 0;
    // gl1->addWidget(gyrRangeCombo = new QComboBox(), l, 1); ++l;
    // gl1->addWidget(accRangeCombo = new QComboBox(), l, 1); ++l;
    // gl1->addWidget(magRangeCombo = new QComboBox(), l, 1); ++l;

    // l = 0;
    // gl2->addWidget(filterModeCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(parameterSetCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(linAccCompModeCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(centriCompModeCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(thresholdEnableCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(gyrAutocalibrationCombo = new QComboBox(), l, 1); ++l;
    // gl2->addWidget(lowPassCombo = new QComboBox(), l, 1); ++l;

    for (int i = 0; i < 129; i++) indexItem->addItem(QString("%1").arg(i));
    connect(indexItem, SIGNAL(currentIndexChanged(int)), this, SLOT(updateOpenMATIndex(int)));
    /*
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
    */

    samplingRateCombo->addItem(QString("5 Hz"));
    samplingRateCombo->addItem(QString("10 Hz"));
    // samplingRateCombo->addItem(QString("25 Hz"));
	samplingRateCombo->addItem(QString("50 Hz"));
	samplingRateCombo->addItem(QString("100 Hz"));
	samplingRateCombo->addItem(QString("200 Hz"));
	// samplingRateCombo->addItem(QString("400 Hz"));

    connect(samplingRateCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updatesamplingRate(int)));

    // QComboBox* syncCombo = new QComboBox();
    // syncCombo->addItem(QString("On"));
    // syncCombo->addItem(QString("Off"));

    // parameterSetCombo->addItem(QString("Weak"));
    // parameterSetCombo->addItem(QString("Medium"));
    // parameterSetCombo->addItem(QString("Strong"));
    // parameterSetCombo->addItem(QString("Dynamic"));
    // connect(parameterSetCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFilterPreset(int)));

    // linAccCompModeCombo->addItem(QString("Off"));
    // linAccCompModeCombo->addItem(QString("Weak"));
    // linAccCompModeCombo->addItem(QString("Medium"));
    // linAccCompModeCombo->addItem(QString("Strong"));
    // linAccCompModeCombo->addItem(QString("Ultra"));
    // connect(linAccCompModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLinAccCompMode(int)));

    // centriCompModeCombo->addItem(QString("Disable"));
    // centriCompModeCombo->addItem(QString("Enable"));
    // connect(centriCompModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCentriCompMode(int)));

    // filterModeCombo->addItem(QString("Gyr only"));
    // filterModeCombo->addItem(QString("Gyr + Acc (Kalman)"));
    // filterModeCombo->addItem(QString("Gyr + Acc + Mag (Kalman)"));
    // filterModeCombo->addItem(QString("Gyr + Acc (DCM)"));
    // filterModeCombo->addItem(QString("Gyr + Acc + Mag (DCM)"));
    // connect(filterModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFilterMode(int)));

    // thresholdEnableCombo->addItem(QString("Disable"));
    // thresholdEnableCombo->addItem(QString("Enable"));
    // connect(thresholdEnableCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrThresholdEnable(int)));

    // gyrAutocalibrationCombo->addItem(QString("Disable"));
    // gyrAutocalibrationCombo->addItem(QString("Enable"));
    // connect(gyrAutocalibrationCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrAutocalibration(int)));

    // lowPassCombo->addItem(QString("off"));
    // lowPassCombo->addItem(QString("0.1"));
    // lowPassCombo->addItem(QString("0.05"));
    // lowPassCombo->addItem(QString("0.01"));
    // lowPassCombo->addItem(QString("0.005"));
    // lowPassCombo->addItem(QString("0.001"));
    // connect(lowPassCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLowPass(int)));

    // accRangeCombo->addItem(QString("2G"));
    // accRangeCombo->addItem(QString("4G"));
    // accRangeCombo->addItem(QString("8G"));
    // accRangeCombo->addItem(QString("16G"));
    // connect(accRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateAccRange(int)));
	
	// gyrRangeCombo->addItem(QString("125 dps"));
	// gyrRangeCombo->addItem(QString("245 dps"));
	// gyrRangeCombo->addItem(QString("500 dps"));
	// gyrRangeCombo->addItem(QString("1000 dps"));
	// gyrRangeCombo->addItem(QString("2000 dps"));
    // connect(gyrRangeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateGyrRange(int)));	

    l = 0;
    selectedDataGl->addWidget(new QLabel("LPBUS data mode:"), l, 0);
    selectedDataGl->addWidget(lpBusDataModeCombo = new QComboBox(), l, 1); ++l;
    lpBusDataModeCombo->addItem(QString("32-bit floating point"));
    lpBusDataModeCombo->addItem(QString("16-bit integer"));
    connect(lpBusDataModeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(updateLpBusDataMode(int)));

    selectedDataGl->addWidget(new QLabel("Enabled data:"), l, 0);
    selectedDataGl->addWidget(selectPressure = new QCheckBox("EMG data"), l, 1); ++l;
    selectedDataGl->addWidget(selectAltitude = new QCheckBox("MMG data"), l, 1); ++l;		
    selectedDataGl->addWidget(selectAcc = new QCheckBox("Raw accelerometer"), l, 1); ++l;
    // selectedDataGl->addWidget(selectMag = new QCheckBox("Raw magnetometer"), l, 1); ++l;
    selectedDataGl->addWidget(selectGyro = new QCheckBox("Raw gyroscope"), l, 1); ++l;
    // selectedDataGl->addWidget(selectAngularVelocity = new QCheckBox("Angular velocity"), l, 1); ++l;
    // selectedDataGl->addWidget(selectQuaternion = new QCheckBox("Quaternion"), l, 1); ++l;
    // selectedDataGl->addWidget(selectEuler = new QCheckBox("Euler angle"), l, 1); ++l;
    // selectedDataGl->addWidget(selectLinAcc = new QCheckBox("Lin. acceleration"), l, 1); ++l;
    // selectedDataGl->addWidget(selectPressure = new QCheckBox("Bar. pressure"), l, 1); ++l;
    // selectedDataGl->addWidget(selectAltitude = new QCheckBox("Altitude"), l, 1); ++l;
    // selectedDataGl->addWidget(selectTemperature = new QCheckBox("Temperature"), l, 1); ++l;

	// connect(selectEmg, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAcc(int)));
    // connect(selectMmg, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAcc(int)));	
    connect(selectAcc, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAcc(int)));
    // connect(selectMag, SIGNAL(stateChanged(int)), this, SLOT(updateSelectMag(int)));
    connect(selectGyro, SIGNAL(stateChanged(int)), this, SLOT(updateSelectGyro(int)));
    // connect(selectQuaternion, SIGNAL(stateChanged(int)), this, SLOT(updateSelectQuaternion(int)));
    // connect(selectEuler, SIGNAL(stateChanged(int)), this, SLOT(updateSelectEuler(int)));
    // connect(selectLinAcc, SIGNAL(stateChanged(int)), this, SLOT(updateSelectLinAcc(int)));
    connect(selectPressure, SIGNAL(stateChanged(int)), this, SLOT(updateSelectPressure(int)));
    connect(selectAltitude, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAltitude(int)));
    // connect(selectTemperature, SIGNAL(stateChanged(int)), this, SLOT(updateSelectTemperature(int)));
    // connect(selectAngularVelocity, SIGNAL(stateChanged(int)), this, SLOT(updateSelectAngularVelocity(int)));

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

    // QWidget *w1 = new QWidget();
    // w1->setLayout(gl1);
    // QTreeWidgetItem* rangeTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Range")));
    // QTreeWidgetItem* rangeTreeWidget = new QTreeWidgetItem(rangeTreeItem);
    // tree->setItemWidget(rangeTreeWidget, 0, w1);

    // QWidget *w2 = new QWidget();
    // w2->setLayout(gl2);
    // QTreeWidgetItem* filterTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Filter")));
    // QTreeWidgetItem* filterTreeWidget = new QTreeWidgetItem(filterTreeItem);
    // tree->setItemWidget(filterTreeWidget, 0, w2);

    // if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE) {
        // QWidget *w5 = new QWidget();
        // w5->setLayout(gl6);
        // QTreeWidgetItem* uartTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("UART (RS-232/TTL)")));
        // QTreeWidgetItem* uartTreeWidget = new QTreeWidgetItem(uartTreeItem);
        // tree->setItemWidget(uartTreeWidget, 0, w5);
    // }

    // if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        // QWidget *w4 = new QWidget();
        // w4->setLayout(gl4);
        // QTreeWidgetItem* canTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("CAN bus")));
        // QTreeWidgetItem* canTreeWidget = new QTreeWidgetItem(canTreeItem);
        // tree->setItemWidget(canTreeWidget, 0, w4);
    // }

    QWidget *w3 = new QWidget();
    w3->setLayout(selectedDataGl);
    QTreeWidgetItem* dataTreeItem = new QTreeWidgetItem(treeItem, QStringList(QString("Data")));
    QTreeWidgetItem* dataTreeWidget = new QTreeWidgetItem(dataTreeItem);
    tree->setItemWidget(dataTreeWidget, 0, w3);

    // heaveMotionEnabled = false;

    updateData();
}

void SensorGuiContainer::setupSensorGuiContainer()
{
    if (containerSetup)
        return;
    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(2, 0, 5);

    // legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);

    containerSetup = true;
}

void SensorGuiContainer::checkOptionalFeatures(void) {
    if (!containerSetup)
        return;
}

void SensorGuiContainer::updateData(void) {
    if (!containerSetup)
        return;
    int i;
    std::string s;
    char cStr[64];
    int omId;
    // int a[32];
    // bool legacyFirmware;

    checkOptionalFeatures();

    indexItem->blockSignals(true);
    // parameterSetCombo->blockSignals(true);
    // thresholdEnableCombo->blockSignals(true);
    // accRangeCombo->blockSignals(true);
    // magRangeCombo->blockSignals(true);
    // gyrRangeCombo->blockSignals(true);
    // filterModeCombo->blockSignals(true);
    // selectQuaternion->blockSignals(true);
    // selectEuler->blockSignals(true);
    // selectLinAcc->blockSignals(true);
    selectGyro->blockSignals(true);
    selectAcc->blockSignals(true);
    // selectMag->blockSignals(true);
    selectPressure->blockSignals(true);
    selectAltitude->blockSignals(true);
    // selectTemperature->blockSignals(true);
    // lowPassCombo->blockSignals(true);
    // linAccCompModeCombo->blockSignals(true);
    // centriCompModeCombo->blockSignals(true);
    lpBusDataModeCombo->blockSignals(true);

    // if (deviceType == DEVICE_LPMS_U || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C || deviceType == DEVICE_LPMS_C2) {
        // canProtocolCombo->blockSignals(false);	
        // canBaudrateCombo->blockSignals(true);
        // canHeartbeatCombo->blockSignals(true);
        // canTpdo1ACombo->blockSignals(true);
        // canTpdo1BCombo->blockSignals(true);
        // canTpdo2ACombo->blockSignals(true);
        // canTpdo2BCombo->blockSignals(true);
        // canTpdo3ACombo->blockSignals(true);
        // canTpdo3BCombo->blockSignals(true);
        // canTpdo4ACombo->blockSignals(true);
        // canTpdo4BCombo->blockSignals(true);
        // canTpdo5ACombo->blockSignals(true);
        // canTpdo5BCombo->blockSignals(true);
        // canTpdo6ACombo->blockSignals(true);
        // canTpdo6BCombo->blockSignals(true);
        // canTpdo7ACombo->blockSignals(true);
        // canTpdo7BCombo->blockSignals(true);
        // canTpdo8ACombo->blockSignals(true);
        // canTpdo8BCombo->blockSignals(true);
        // canChannelModeCombo->blockSignals(true);
        // canPointModeCombo->blockSignals(true);
        // canStartIdSpin->blockSignals(true);
    // }

    sensor->getConfigurationPrm(PRM_OPENMAT_ID, &omId);
    sensor->getConfigurationPrm(PRM_DEVICE_ID, cStr);
    //sensor->getConfigurationPrm(PRM_DEVICE_TYPE, &i);

    switch (deviceType) {
    // case DEVICE_LPMS_B:
        // treeItem->setText(0, QString("LPMS-B ID=%1 (").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_BLE:
        // treeItem->setText(0, QString("LPMS-BLE ID=%1 (").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_U:
        // treeItem->setText(0, QString("LPMS-U ID=%1 (USB ID: ").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_C:
        // treeItem->setText(0, QString("LPMS-CU ID=%1 (CAN ID: ").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_B2:
        // treeItem->setText(0, QString("LPMS-B2 ID=%1 (").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_U2:
        // treeItem->setText(0, QString("LPMS-U2 ID=%1 (").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_C2:
        // treeItem->setText(0, QString("LPMS-CU2 ID=%1 (CAN ID: ").arg(omId) + QString(cStr) + ")");
        // break;

    // case DEVICE_LPMS_RS232:
        // treeItem->setText(0, QString("LPMS-RS232 ID=%1 (COM PORT: ").arg(omId) + QString(cStr) + ")");
        // break;

    case DEVICE_LPEMG_B:
        treeItem->setText(0, QString("LPEMG-B ID=%1 (").arg(omId) + QString(cStr) + ")");
        break;
    }

    addressItem->setText(QString(cStr));

    sensor->getConfigurationPrm(PRM_OPENMAT_ID, &i);
    indexItem->setCurrentIndex(i);

    // if (deviceType != DEVICE_LPMS_B && deviceType != DEVICE_LPMS_B2 && deviceType != DEVICE_LPMS_BLE && deviceType != DEVICE_LPEMG_B) {
        // sensor->getConfigurationPrm(PRM_UART_BAUDRATE, &i);

        // baudRateCombo->setCurrentIndex(i);

        // sensor->getConfigurationPrm(PRM_UART_FORMAT, &i);
        // switch (i) {
        // case SELECT_LPMS_UART_FORMAT_LPBUS:
            // uartFormatCombo->setCurrentIndex(0);
            // break;

        // case SELECT_LPMS_UART_FORMAT_CSV:
            // uartFormatCombo->setCurrentIndex(1);
            // break;
        // }
    // }

    sensor->getConfigurationPrm(PRM_FIRMWARE_VERSION, cStr);
    firmwareItem->setText(QString(cStr));

    // sensor->getConfigurationPrm(PRM_PARAMETER_SET, &i);
    // switch (i) {
    // case SELECT_IMU_SLOW:
        // parameterSetCombo->setCurrentIndex(0);
        // break;

    // case SELECT_IMU_MEDIUM:
        // parameterSetCombo->setCurrentIndex(1);
        // break;

    // case SELECT_IMU_FAST:
        // parameterSetCombo->setCurrentIndex(2);
        // break;

    // case SELECT_IMU_DYNAMIC:
        // parameterSetCombo->setCurrentIndex(3);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, &i);
    // switch (i) {
    // case SELECT_IMU_GYR_THRESH_ENABLED:
        // thresholdEnableCombo->setCurrentIndex(1);
        // break;

    // case SELECT_IMU_GYR_THRESH_DISABLED:
        // thresholdEnableCombo->setCurrentIndex(0);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_GYR_AUTOCALIBRATION, &i);
    // switch (i) {
    // case SELECT_GYR_AUTOCALIBRATION_ENABLED:
        // gyrAutocalibrationCombo->setCurrentIndex(1);
        // break;

    // case SELECT_GYR_AUTOCALIBRATION_DISABLED:
        // gyrAutocalibrationCombo->setCurrentIndex(0);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_FILTER_MODE, &i);
    // switch (i) {
    // case SELECT_FM_GYRO_ONLY:
        // filterModeCombo->setCurrentIndex(0);
        // break;

    // case SELECT_FM_GYRO_ACC:
        // filterModeCombo->setCurrentIndex(1);
        // break;

    // case SELECT_FM_GYRO_ACC_MAG:
        // filterModeCombo->setCurrentIndex(2);
        // break;

    // case SELECT_FM_MADGWICK_GYRO_ACC:
        // filterModeCombo->setCurrentIndex(3);
        // break;

    // case SELECT_FM_MADGWICK_GYRO_ACC_MAG:
        // filterModeCombo->setCurrentIndex(4);
        // break;
    // }

    switch (sensor->getSensorStatus()) {
    case 0:
        runningItem->setText("<font color='black'>stopped</font>");
        break;

    case 1:
        runningItem->setText("<font color='green'>started</font>");
        break;

    case 2:
        runningItem->setText("<font color='blue'>calibrating..</font>");
        break;

    case 3:
        runningItem->setText("<font color='blue'>error..</font>");
        break;

    case 4:
        runningItem->setText("<font color='blue'>uploading..</font>");
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
        // if (sensor->getSensorStatus() == SENSOR_STATUS_RUNNING)
            // FIXME
            // batteryItem->setText(QString::number(sensor->getBatteryLevel()) + "%");
    }

    // legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);
    // sensor->getConfigurationPrm(PRM_GYR_RANGE, &i);

    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // switch (i) {
        // case SELECT_GYR_RANGE_125DPS:
            // if (gyrRangeCombo->currentIndex() != 0)
                // gyrRangeCombo->setCurrentIndex(0);
            // break;

        // case SELECT_GYR_RANGE_245DPS:
            // if (gyrRangeCombo->currentIndex() != 1)
                // gyrRangeCombo->setCurrentIndex(1);
            // break;

        // case SELECT_GYR_RANGE_500DPS:
            // if (gyrRangeCombo->currentIndex() != 2)
                // gyrRangeCombo->setCurrentIndex(2);
            // break;

        // case SELECT_GYR_RANGE_1000DPS:
            // if (gyrRangeCombo->currentIndex() != 3)
                // gyrRangeCombo->setCurrentIndex(3);
            // break;

        // case SELECT_GYR_RANGE_2000DPS:
            // if (gyrRangeCombo->currentIndex() != 4)
                // gyrRangeCombo->setCurrentIndex(4);
            // break;
        // }
    // }
    // else
    // {
        // switch (i) {
        // case SELECT_GYR_RANGE_250DPS:
            // if (gyrRangeCombo->currentIndex() != 0)
            // gyrRangeCombo->setCurrentIndex(0);
            // break;

        // case SELECT_GYR_RANGE_500DPS:
            // if (gyrRangeCombo->currentIndex() != 1)
            // gyrRangeCombo->setCurrentIndex(1);
            // break;

        // case SELECT_GYR_RANGE_2000DPS:
            // if (gyrRangeCombo->currentIndex() != 2)
            // gyrRangeCombo->setCurrentIndex(2);
            // break;
        // }
    // }
    // sensor->getConfigurationPrm(PRM_ACC_RANGE, &i);

    // switch (i) {
    // case SELECT_ACC_RANGE_2G:
        // accRangeCombo->setCurrentIndex(0);
        // break;

    // case SELECT_ACC_RANGE_4G:
        // accRangeCombo->setCurrentIndex(1);
        // break;

    // case SELECT_ACC_RANGE_8G:
        // accRangeCombo->setCurrentIndex(2);
        // break;

    // case SELECT_ACC_RANGE_16G:
        // accRangeCombo->setCurrentIndex(3);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_MAG_RANGE, &i);
    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // switch (i) {
        // case SELECT_MAG_RANGE_4GAUSS:
            // if (magRangeCombo->currentIndex() != 0)
                // magRangeCombo->setCurrentIndex(0);
            // break;
        // case SELECT_MAG_RANGE_8GAUSS:
            // if (magRangeCombo->currentIndex() != 1)
                // magRangeCombo->setCurrentIndex(1);
            // break;
        // case SELECT_MAG_RANGE_12GAUSS:
            // if (magRangeCombo->currentIndex() != 2)
                // magRangeCombo->setCurrentIndex(2);
            // break;
        // case SELECT_MAG_RANGE_16GAUSS:
            // if (magRangeCombo->currentIndex() != 3)
                // magRangeCombo->setCurrentIndex(3);
            // break;
        // }
    // }
    // else
    // {
        // switch (i) {
        // case SELECT_MAG_RANGE_130UT:
            // if (magRangeCombo->currentIndex() != 0)
                // magRangeCombo->setCurrentIndex(0);
            // break;

        // case SELECT_MAG_RANGE_190UT:
            // if (magRangeCombo->currentIndex() != 1)
                // magRangeCombo->setCurrentIndex(1);
            // break;

        // case SELECT_MAG_RANGE_250UT:
            // if (magRangeCombo->currentIndex() != 2)
                // magRangeCombo->setCurrentIndex(2);
            // break;

        // case SELECT_MAG_RANGE_400UT:
            // if (magRangeCombo->currentIndex() != 3)
                // magRangeCombo->setCurrentIndex(3);
            // break;

        // case SELECT_MAG_RANGE_560UT:
            // if (magRangeCombo->currentIndex() != 4)
                // magRangeCombo->setCurrentIndex(4);
            // break;

        // case SELECT_MAG_RANGE_810UT:
            // if (magRangeCombo->currentIndex() != 5)
                // magRangeCombo->setCurrentIndex(5);
            // break;
        // }
    // }

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

    // if ((i & SELECT_LPMS_MAG_OUTPUT_ENABLED) != 0) {
        // selectMag->setCheckState(Qt::Checked);
    // }
    // else {
        // selectMag->setCheckState(Qt::Unchecked);
    // }

    if ((i & SELECT_LPMS_GYRO_OUTPUT_ENABLED) != 0) {
        selectGyro->setCheckState(Qt::Checked);
    }
    else {
        selectGyro->setCheckState(Qt::Unchecked);
    }

    // if ((i & SELECT_LPMS_QUAT_OUTPUT_ENABLED) != 0) {
        // selectQuaternion->setCheckState(Qt::Checked);
    // }
    // else {
        // selectQuaternion->setCheckState(Qt::Unchecked);
    // }

    // if ((i & SELECT_LPMS_EULER_OUTPUT_ENABLED) != 0) {
        // selectEuler->setCheckState(Qt::Checked);
    // }
    // else {
        // selectEuler->setCheckState(Qt::Unchecked);
    // }

    // if ((i & SELECT_LPMS_LINACC_OUTPUT_ENABLED) != 0) {
        // selectLinAcc->setCheckState(Qt::Checked);
    // }
    // else {
        // selectLinAcc->setCheckState(Qt::Unchecked);
    // }

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

    // if ((i & SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
        // selectTemperature->setCheckState(Qt::Checked);
    // }
    // else {
        // selectTemperature->setCheckState(Qt::Unchecked);
    // }

    // if ((i & SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
        // selectAngularVelocity->setCheckState(Qt::Checked);
    // }
    // else {
        // selectAngularVelocity->setCheckState(Qt::Unchecked);
    // }

    // if (heaveMotionEnabled == true) {
        // selectHeaveMotion->blockSignals(true);

        // if ((i & SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
            // selectHeaveMotion->setCheckState(Qt::Checked);
        // }
        // else {
            // selectHeaveMotion->setCheckState(Qt::Unchecked);
        // }

        // selectHeaveMotion->blockSignals(false);
    // }

    // sensor->getConfigurationPrm(PRM_LOW_PASS, &i);
    // switch (i) {
    // case SELECT_LPMS_LP_OFF:
        // lowPassCombo->setCurrentIndex(0);
        // break;

    // case SELECT_LPMS_LP_01:
        // lowPassCombo->setCurrentIndex(1);
        // break;

    // case SELECT_LPMS_LP_005:
        // lowPassCombo->setCurrentIndex(2);
        // break;

    // case SELECT_LPMS_LP_001:
        // lowPassCombo->setCurrentIndex(3);
        // break;

    // case SELECT_LPMS_LP_0005:
        // lowPassCombo->setCurrentIndex(4);
        // break;

    // case SELECT_LPMS_LP_0001:
        // lowPassCombo->setCurrentIndex(5);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_LIN_ACC_COMP_MODE, &i);
    // switch (i) {
    // case SELECT_LPMS_LIN_ACC_COMP_MODE_OFF:
        // linAccCompModeCombo->setCurrentIndex(0);
        // break;

    // case SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK:
        // linAccCompModeCombo->setCurrentIndex(1);
        // break;

    // case SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM:
        // linAccCompModeCombo->setCurrentIndex(2);
        // break;

    // case SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG:
        // linAccCompModeCombo->setCurrentIndex(3);
        // break;

    // case SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA:
        // linAccCompModeCombo->setCurrentIndex(4);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_CENTRI_COMP_MODE, &i);
    // switch (i) {
    // case SELECT_LPMS_CENTRI_COMP_MODE_OFF:
        // centriCompModeCombo->setCurrentIndex(0);
        // break;

    // case SELECT_LPMS_CENTRI_COMP_MODE_ON:
        // centriCompModeCombo->setCurrentIndex(1);
        // break;
    // }

    // sensor->getConfigurationPrm(PRM_LPBUS_DATA_MODE, &i);
    // switch (i) {
    // case SELECT_LPMS_LPBUS_DATA_MODE_32:
        // lpBusDataModeCombo->setCurrentIndex(0);
        // break;

    // case SELECT_LPMS_LPBUS_DATA_MODE_16:
        // lpBusDataModeCombo->setCurrentIndex(1);
        // break;
    // }

    selectAcc->blockSignals(false);
    // selectMag->blockSignals(false);
    // selectQuaternion->blockSignals(false);
    // selectEuler->blockSignals(false);
    // selectLinAcc->blockSignals(false);
    selectGyro->blockSignals(false);
    // lowPassCombo->blockSignals(false);
    indexItem->blockSignals(false);
    // parameterSetCombo->blockSignals(false);
    // thresholdEnableCombo->blockSignals(false);
    // accRangeCombo->blockSignals(false);
    // magRangeCombo->blockSignals(false);
    // gyrRangeCombo->blockSignals(false);
    // filterModeCombo->blockSignals(false);
    selectPressure->blockSignals(false);
    selectAltitude->blockSignals(false);
    // selectTemperature->blockSignals(false);
    // linAccCompModeCombo->blockSignals(false);
    // centriCompModeCombo->blockSignals(false);
    lpBusDataModeCombo->blockSignals(false);
}

void SensorGuiContainer::updateOpenMATIndex(int i)
{
    if (!containerSetup)
        return;
    sensor->setConfigurationPrm(PRM_OPENMAT_ID, indexItem->currentIndex());
}

// void SensorGuiContainer::updateBaudRateIndex(int i)
// {
    // if (!containerSetup)
        // return;

    // sensor->setConfigurationPrm(PRM_UART_BAUDRATE, i);
    // /*
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
    // */
// }

// void SensorGuiContainer::updateGyrThresholdEnable(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (thresholdEnableCombo->currentIndex()) {
    // case 1:
        // sensor->setConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_ENABLED);
        // break;

    // case 0:
        // sensor->setConfigurationPrm(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
        // break;
    // }
// }

// void SensorGuiContainer::updateGyrAutocalibration(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (gyrAutocalibrationCombo->currentIndex()) {
    // case 1:
        // sensor->setConfigurationPrm(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
        // break;

    // case 0:
        // sensor->setConfigurationPrm(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_DISABLED);
        // break;
    // }
// }

// void SensorGuiContainer::updateFilterPreset(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (parameterSetCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_SLOW);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_MEDIUM);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_FAST);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_DYNAMIC);
        // break;
    // }
// }

// void SensorGuiContainer::updateFilterMode(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (filterModeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ONLY);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_GYRO_ACC_MAG);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_MADGWICK_GYRO_ACC);
        // break;

    // case 4:
        // sensor->setConfigurationPrm(PRM_FILTER_MODE, SELECT_FM_MADGWICK_GYRO_ACC_MAG);
        // break;
    // }
// }

// void SensorGuiContainer::updateGyrRange(int i)
// {
    // if (!containerSetup)
        // return;
    
    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1,9,9);
    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // switch (gyrRangeCombo->currentIndex()) {
        // case 0:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_125DPS);
            // break;

        // case 1:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_245DPS);
            // break;

        // case 2:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_500DPS);
            // break;

        // case 3:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_1000DPS);
            // break;

        // case 4:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_2000DPS);
            // break;
        // }

    // }
    // else
    // {
        // switch (gyrRangeCombo->currentIndex()) {
        // case 0:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_250DPS);
            // break;

        // case 1:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_500DPS);
            // break;

        // case 2:
            // sensor->setConfigurationPrm(PRM_GYR_RANGE, SELECT_GYR_RANGE_2000DPS);
            // break;
        // }
    // }
// }

// void SensorGuiContainer::updateAccRange(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (accRangeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_2G);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_4G);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_8G);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_ACC_RANGE, SELECT_ACC_RANGE_16G);
        // break;
    // }
// }

// void SensorGuiContainer::updateMagRange(int i)
// {
    // if (!containerSetup)
        // return;

    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1, 9, 9);
    // if (deviceType == DEVICE_LPMS_B2 || deviceType == DEVICE_LPMS_U2 || deviceType == DEVICE_LPMS_C2 || !legacyFirmware)
    // {
        // switch (magRangeCombo->currentIndex()) {

        // case 0:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_4GAUSS);
            // break;
        // case 1:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_8GAUSS);
            // break;
        // case 2:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_12GAUSS);
            // break;
        // case 3:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_16GAUSS);
            // break;
        // }
    // }
    // else
    // {
        // switch (magRangeCombo->currentIndex()) {
        // case 0:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_130UT);
            // break;

        // case 1:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_190UT);
            // break;

        // case 2:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_250UT);
            // break;

        // case 3:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_400UT);
            // break;

        // case 4:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_560UT);
            // break;

        // case 5:
            // sensor->setConfigurationPrm(PRM_MAG_RANGE, SELECT_MAG_RANGE_810UT);
            // break;
        // }
    // }
// }

// void SensorGuiContainer::updateCanProtocol(int i)
// {
// }

// void SensorGuiContainer::updateCanBaudrate(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (canBaudrateCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_500KBPS);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_250KBPS);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS);
        // break;
    // }
// }

// void SensorGuiContainer::updateLowPass(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (lowPassCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_OFF);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_01);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_005);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_001);
        // break;

    // case 4:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_0005);
        // break;

    // case 5:
        // sensor->setConfigurationPrm(PRM_LOW_PASS, SELECT_LPMS_LP_0001);
        // break;
    // }
// }

void SensorGuiContainer::updatesamplingRate(int i)
{
    if (!containerSetup)
        return;
    // bool legacyFirmware = isFirmwareVersionLessThanEqualTo(1,3,4);
    switch (samplingRateCombo->currentIndex()) {
    case 0:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);
        break;

    case 1:
        sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);
        break;

    case 2:
        // if (legacyFirmware)
            // sensor->setConfigurationPrm(PRM_SAMPLING_RATE, 30);
        // else
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_25HZ);
        // break;

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
        // if (legacyFirmware)
            // sensor->setConfigurationPrm(PRM_SAMPLING_RATE, 300);
        // else
            sensor->setConfigurationPrm(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_400HZ);
        // break;
    }
}

SensorGuiContainer::~SensorGuiContainer(void)
{
}

LpemgSensorI* SensorGuiContainer::getSensor(void)
{
    return sensor;
}

void SensorGuiContainer::updateSelectGyro(int i)
{
	int p = 0;
	
    if (!containerSetup)
        return;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_GYRO_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

// void SensorGuiContainer::updateSelectQuaternion(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_QUAT_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

// void SensorGuiContainer::updateSelectEuler(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_EULER_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

// void SensorGuiContainer::updateSelectLinAcc(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_LINACC_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

void SensorGuiContainer::updateSelectAcc(int i)
{
    int p = 0;	
	
    if (!containerSetup)
        return;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

// void SensorGuiContainer::updateSelectMag(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

void SensorGuiContainer::updateSelectPressure(int i)
{
	int p = 0;
	
    if (!containerSetup)
        return;

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
	int p = 0;
	
    if (!containerSetup)
        return;

    sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    if (i == Qt::Checked) {
        p |= SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
    }
    else {
        p &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
    }

    sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
}

// void SensorGuiContainer::updateSelectTemperature(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

// void SensorGuiContainer::updateSelectAngularVelocity(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

// void SensorGuiContainer::updateSelectHeaveMotion(int i)
// {
    // if (!containerSetup)
        // return;
    // int p = 0;

    // sensor->getConfigurationPrm(PRM_SELECT_DATA, &p);

    // if (i == Qt::Checked) {
        // p |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
    // }
    // else {
        // p &= ~SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
    // }

    // sensor->setConfigurationPrm(PRM_SELECT_DATA, p);
// }

// void SensorGuiContainer::updateCanMapping(int i)
// {
    // if (!containerSetup)
        // return;
    // int a[32];

    // a[0] = canTpdo1ACombo->currentIndex();
    // a[1] = canTpdo1BCombo->currentIndex();
    // a[2] = canTpdo2ACombo->currentIndex();
    // a[3] = canTpdo2BCombo->currentIndex();
    // a[4] = canTpdo3ACombo->currentIndex();
    // a[5] = canTpdo3BCombo->currentIndex();
    // a[6] = canTpdo4ACombo->currentIndex();
    // a[7] = canTpdo4BCombo->currentIndex();
    // a[8] = canTpdo5ACombo->currentIndex();
    // a[9] = canTpdo5BCombo->currentIndex();
    // a[10] = canTpdo6ACombo->currentIndex();
    // a[11] = canTpdo6BCombo->currentIndex();
    // a[12] = canTpdo7ACombo->currentIndex();
    // a[13] = canTpdo7BCombo->currentIndex();
    // a[14] = canTpdo8ACombo->currentIndex();
    // a[15] = canTpdo8BCombo->currentIndex();

    // sensor->setConfigurationPrm(PRM_CAN_MAPPING, a);
// }

// void SensorGuiContainer::updateCanHeartbeat(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (canHeartbeatCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_005);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_010);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_020);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_050);
        // break;

    // case 4:
        // sensor->setConfigurationPrm(PRM_CAN_HEARTBEAT, SELECT_LPMS_CAN_HEARTBEAT_100);
        // break;
    // }
// }

// void SensorGuiContainer::updateLinAccCompMode(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (linAccCompModeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_OFF);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK);
        // break;

    // case 2:
        // sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM);
        // break;

    // case 3:
        // sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG);
        // break;

    // case 4:
        // sensor->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA);
        // break;
    // }
// }

// void SensorGuiContainer::updateCentriCompMode(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (centriCompModeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_OFF);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_CENTRI_COMP_MODE, SELECT_LPMS_CENTRI_COMP_MODE_ON);
        // break;
    // }
// }

// void SensorGuiContainer::updateCanStartId(int i)
// {
    // if (!containerSetup)
        // return;
    // sensor->setConfigurationPrm(PRM_CAN_START_ID, canStartIdSpin->value());
// }

// void SensorGuiContainer::updateCanPointMode(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (canPointModeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FLOATING);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_CAN_POINT_MODE, SELECT_CAN_POINT_MODE_FIXED);
        // break;
    // }
// }

// void SensorGuiContainer::updateCanChannelMode(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (canChannelModeCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_CANOPEN);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_CAN_CHANNEL_MODE, SELECT_CAN_CHANNEL_MODE_SEQUENTIAL);
        // break;
    // }
// }

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

// void SensorGuiContainer::updateUartFormatIndex(int i)
// {
    // if (!containerSetup)
        // return;
    // switch (uartFormatCombo->currentIndex()) {
    // case 0:
        // sensor->setConfigurationPrm(PRM_UART_FORMAT, SELECT_LPMS_UART_FORMAT_LPBUS);
        // break;

    // case 1:
        // sensor->setConfigurationPrm(PRM_UART_FORMAT, SELECT_LPMS_UART_FORMAT_CSV);
        // break;
    // }
// }

// bool SensorGuiContainer::isFirmwareVersionLessThanEqualTo(int i, int j, int k)
// {
    // UGLY HACK
    // not valid if i,j,k > 9
    // char cStr[64];
    // sensor->getConfigurationPrm(PRM_FIRMWARE_VERSION, cStr);
    // int target = i * 100 + j * 10 + k;
    // int currentFirmware = (cStr[0] - '0') * 100 + (cStr[2] - '0') * 10 + (cStr[4] - '0');
    // if (currentFirmware <= target)
        // return true;
    // return false;
// }