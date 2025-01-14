/***********************************************************************
** (c) LP-RESEARCH Inc.
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

#include "MainWindow.h"

static LpmsDeviceList deviceList;

MainWindow::MainWindow(QWidget *parent)
{
    std::cout << "[MainWindow] Initializing program" << std::endl;

    heaveMotionEnabled = false;

    sm = LpmsSensorManagerFactory();
    sm->setThreadTiming(100);

    QSplitter *s0 = new QSplitter();

    s0->addWidget(createDeviceList());
    s0->addWidget(createGraphs());

    createMenuAndToolbar();

    rescanD = new RescanDialog(sm, comboDeviceList, &deviceList, this);

    s0->setStretchFactor(0, 3);
    s0->setStretchFactor(1, 5);

    QHBoxLayout *h0 = new QHBoxLayout();
    h0->addWidget(s0);

    QWidget* cw = new QWidget();
    cw->setLayout(h0);
    setCentralWidget(cw);

    this->setMinimumSize(800, 600);
    showMaximized();

    setWindowTitle("LpmsControl-V"+ QString(LPMS_CONTROL_VERSION) + " GUI");

    isRunning = false;
    isConnecting = false;
    calibratingMag = false;

    // mbcom.startServer();

    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
    timer->start(1);

    textUpdateCounter = 0;
    maIsCalibrating = false;
    mode = MODE_GRAPH_WIN;

    isMagEllipsoidCalOn = false;
    isMagPlanarCalOn = false;

    rePlayer = new MotionPlayer();
    softwareSyncCount = 0;
    mm.reset();
}

MainWindow::~MainWindow()
{
    delete sm;
}

QWidget *MainWindow::createDeviceList(void)
{
    lpmsTree = new QTreeWidget();

    lpmsTree->setColumnCount(1);
    lpmsTree->setHeaderLabel(QString("Connected devices"));
    currentLpms = 0;
    lpmsTree->selectionModel()->setCurrentIndex(QModelIndex(), QItemSelectionModel::Clear);
    lpmsTree->setVerticalScrollMode(QTreeView::ScrollPerPixel);

    connect(lpmsTree, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(updateCurrentLpms(QTreeWidgetItem *, QTreeWidgetItem *)));

    return (QWidget*)lpmsTree;
}

QGroupBox *MainWindow::createGraphs(void)
{
    graphLayout = new QVBoxLayout();
    cubeWindowContainer = new CubeWindowSelector();
    graphWindow = new GraphWindow();
    fieldMapWindow = new FieldMapContainer();

    graphLayout->addWidget(graphWindow);
    graphLayout->addWidget(cubeWindowContainer);
    graphLayout->addWidget(fieldMapWindow);

    gb = new QGroupBox("Data view");
    gb->setLayout(graphLayout);

    cubeWindowContainer->hide();
    fieldMapWindow->hide();

    return gb;
}

void MainWindow::createMenuAndToolbar(void)
{
    toolbar = new QToolBar("Toolbar");
    addToolBar(Qt::TopToolBarArea, toolbar);
    toolbar->setMovable(false);
    toolbar->setFloatable(false);
    QMenu* connectMenu = menuBar()->addMenu("&Connect");

    QAction* connectAction = new QAction(QIcon("./icons/bolt_32x32.png"), "&Connect", this);
    QAction* disconnectAction = new QAction(QIcon("./icons/x_28x28.png"), "&Disconnect", this);
    QAction* addRemoveAction = new QAction(QIcon("./icons/plus_32x32.png"), "&Add / remove sensor", this);
    QAction* exitAction = new QAction("E&xit program", this);

    QVBoxLayout *v3 = new QVBoxLayout();
    comboDeviceList = new QComboBox();
    v3->addWidget(new QLabel("Preferred devices:"));
    v3->addWidget(comboDeviceList);
    QWidget *w3 = new QWidget();
    w3->setLayout(v3);
    w3->setFixedWidth(200);
    toolbar->addWidget(w3);

    if (sm->isCanPresent() == true) {
        QVBoxLayout *v4 = new QVBoxLayout();
        canBaudrateList = new QComboBox();
        canBaudrateList->addItem("125 kbit");
        canBaudrateList->addItem("250 kbit");
        canBaudrateList->addItem("500 kbit");
        canBaudrateList->addItem("1000 kbit");
        canBaudrateList->setCurrentIndex(0);
        v4->addWidget(new QLabel("CAN baudrate:"));
        v4->addWidget(canBaudrateList);
        QWidget *w4 = new QWidget();
        w4->setLayout(v4);
        w4->setFixedWidth(150);
        toolbar->addWidget(w4);
        connect(canBaudrateList, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCanBaudrate(int)));
    }

    QVBoxLayout *v6 = new QVBoxLayout();
    rs232BaudrateList = new QComboBox();
    rs232BaudrateList->addItem("19200 bps");
    rs232BaudrateList->addItem("38400 bps");
    rs232BaudrateList->addItem("57600 bps");
    rs232BaudrateList->addItem("115200 bps");
    rs232BaudrateList->addItem("230400 bps");
    rs232BaudrateList->addItem("256000 bps");
    rs232BaudrateList->addItem("460800 bps");
    rs232BaudrateList->addItem("921600 bps");
    rs232BaudrateList->setCurrentIndex(3);
    v6->addWidget(new QLabel("RS232 baudrate:"));
    v6->addWidget(rs232BaudrateList);
    QWidget *w6 = new QWidget();
    w6->setLayout(v6);
    w6->setFixedWidth(150);
    toolbar->addWidget(w6);
    connect(rs232BaudrateList, SIGNAL(currentIndexChanged(int)), this, SLOT(updateRs232Baudrate(int)));

    connectMenu->addAction(connectAction);
    connectMenu->addAction(disconnectAction);
    connectMenu->addAction(addRemoveAction);
    connectMenu->addSeparator();
    connectMenu->addAction(exitAction);

    toolbar->addAction(connectAction);
    toolbar->addAction(disconnectAction);
    toolbar->addAction(addRemoveAction);

    QMenu* measurementMenu = menuBar()->addMenu("&Measurement");
    startAction = new QAction(QIcon("./icons/play_24x32.png"), "&Start measurement", this);
    QAction* browseAction = new QAction(QIcon("./icons/folder_stroke_32x32.png"), "&Browse record file", this);
    QAction* stopAction = new QAction("Stop measurement", this);
    saveAction = new QAction(QIcon("./icons/layers_32x28.png"), "&Record data", this);

    measurementMenu->addAction(startAction);
    measurementMenu->addAction(browseAction);
    measurementMenu->addAction(saveAction);

    toolbar->addSeparator();
    QVBoxLayout *v = new QVBoxLayout();
    recordFileEdit = new QLineEdit();
    recordFileEdit->setReadOnly(true);
    recordFileEdit->setText("Not set, please browse..");
    recordFileSet = false;
    v->addWidget(new QLabel("Record filename:"));
    v->addWidget(recordFileEdit);
    QWidget *w = new QWidget();
    w->setLayout(v);
    w->setFixedWidth(200);

    toolbar->addAction(startAction);
    toolbar->addAction(saveAction);
    toolbar->addWidget(w);
    toolbar->addAction(browseAction);

    replayAction = new QAction(QIcon("./icons/loop_alt2_32x28.png"), "&Playback data", this);
    QAction* browseReplayAction = new QAction(QIcon("./icons/folder_stroke_32x32.png"), "&Browse replay file", this);

    toolbar->addSeparator();
    QVBoxLayout *v4 = new QVBoxLayout();
    playbackFileEdit = new QLineEdit();
    playbackFileEdit->setReadOnly(true);
    playbackFileEdit->setText("Not set, please browse..");
    playbackFileSet = false;
    v4->addWidget(new QLabel("Playback filename:"));
    v4->addWidget(playbackFileEdit);
    QWidget *w4 = new QWidget();
    w4->setLayout(v4);
    w4->setFixedWidth(200);

    toolbar->addAction(replayAction);
    toolbar->addWidget(w4);
    toolbar->addAction(browseReplayAction);

    measurementMenu->addSeparator();
    measurementMenu->addAction(browseReplayAction);
    measurementMenu->addAction(replayAction);

    startFlashLogAction = new QAction("Start Flash Log", this);
    stopFlashLogAction = new QAction("Stop Flash Log", this);
    clearFlashLogAction = new QAction("Clear Flash Log", this);
    eraseFlashAction = new QAction("Full Flash Erase", this);
    saveFlashLogAction = new QAction("Save Flash Log", this);

    measurementMenu->addSeparator();
    measurementMenu->addAction(startFlashLogAction);
    measurementMenu->addAction(stopFlashLogAction);
    measurementMenu->addAction(clearFlashLogAction);
    measurementMenu->addAction(eraseFlashAction);
    measurementMenu->addAction(saveFlashLogAction);

    toolbar->addSeparator();

    QVBoxLayout *v2 = new QVBoxLayout();
    targetCombo = new QComboBox();
    targetCombo->addItem("All sensors");
    targetCombo->addItem("Selected sensor");
    v2->addWidget(new QLabel("Reset target:"));
    v2->addWidget(targetCombo);
    QWidget *w2 = new QWidget();
    w2->setLayout(v2);
    w2->setFixedWidth(150);
    toolbar->addWidget(w2);

    QVBoxLayout *v5 = new QVBoxLayout();
    resetMethodCombo = new QComboBox();
    resetMethodCombo->addItem("Object reset");
    resetMethodCombo->addItem("Heading reset");
    resetMethodCombo->addItem("Alignment reset");
    v5->addWidget(new QLabel("Reset method:"));
    v5->addWidget(resetMethodCombo);
    QWidget *w5 = new QWidget();
    w5->setLayout(v5);
    w5->setFixedWidth(150);
    toolbar->addWidget(w5);

    QAction* setOffsetAction = new QAction(QIcon("./icons/fullscreen_exit_32x32.png"), "Set offset", this);
    toolbar->addAction(setOffsetAction);
    QAction* resetOffsetAction = new QAction(QIcon("./icons/denied_32x32.png"), "Reset offset", this);
    toolbar->addAction(resetOffsetAction);

    QMenu* calibrationMenu = menuBar()->addMenu("&Calibration");

    QAction* gyroAction = new QAction("Calibrate &gyroscope", this);
    QAction* startMagAction = new QAction("Calibrate &mag. (ellipsoid fit)", this);
    QAction* startPlanarMagAction = new QAction("Calibrate &mag. (min/max fit)", this);
    QAction* stopMagAction = new QAction("Stop magnetometer calibration", this);
    QAction* resetSingleRefAction = new QAction("Reset &heading (selected)", this);
    QAction* resetAllRefAction = new QAction("Reset heading (&all)", this);
    QAction* resetSingleOrientationAction = new QAction("Reset &offset (selected)", this);
    QAction* resetAllOrientationAction = new QAction("Reset o&ffset (all)", this);
    QAction* saveCalAction = new QAction("Save &parameters to sensor", this);
    QAction* resetToFactoryAction = new QAction("Reset to factory settings", this);
    QAction* mACalculateAction = new QAction("Calibrate acc. misalignment", this);
    QAction* gyrMaCalculateAction = new QAction("Calibrate gyr. misalignment", this);
    QAction* magMaCalculateAction = new QAction("Calibrate mag. misalignment (HH-coils)", this);
    QAction* magAutoMaCalculateAction = new QAction("Calibrate mag. misalignment (auto)", this);
    QAction* saveToFileAction = new QAction("Save calibration file", this);
    QAction* loadFromFileAction = new QAction("Load calibration file", this);
    QAction* timestampResetAction = new QAction("Reset Timestamp (All sensors)", this);
    QAction* softwareSyncStartAction = new QAction("Software Sync", this);
    // QAction* softwareSyncStopAction = new QAction("Software Sync Stop", this);

    calibrationMenu->addAction(gyroAction);
    calibrationMenu->addAction(startMagAction);
    calibrationMenu->addAction(startPlanarMagAction);
    calibrationMenu->addSeparator();
    calibrationMenu->addAction(saveCalAction);
    calibrationMenu->addAction(saveToFileAction);
    calibrationMenu->addAction(loadFromFileAction);
    calibrationMenu->addSeparator();
    calibrationMenu->addAction(setOffsetAction);
    calibrationMenu->addAction(resetOffsetAction);
    calibrationMenu->addSeparator();
    calibrationMenu->addAction(timestampResetAction);
    calibrationMenu->addAction(softwareSyncStartAction);
    // calibrationMenu->addAction(softwareSyncStopAction);
    calibrationMenu->addSeparator();
    calibrationMenu->addAction(resetToFactoryAction);

    viewMenu = menuBar()->addMenu("&View");
    QAction* graphAction = new QAction(QIcon("./icons/rss_alt_32x32.png"), "Graph &window", this);
    QAction* orientationGraphAction = new QAction(QIcon("./icons/target_32x32.png"), "Orien&tation window", this);
    QAction* pressureGraphAction = new QAction(QIcon("./icons/eyedropper_32x32.png"), "Pressu&re window", this);
    QAction* threedAction = new QAction(QIcon("./icons/share_32x32.png"), "3D &visualization", this);
    QAction* fieldMapAction = new QAction(QIcon("./icons/sun_fill_32x32.png"), "&Magnetic field map", this);
    QAction* loadObjFileAction = new QAction("&Load object file", this);
    xAxisAutoScrollingAction = new QAction("&X Axis Auto Scrolling", this);
    xAxisAutoScrollingAction->setCheckable(true);
    xAxisAutoScrollingAction->setChecked(false);
    yAxisAutoScalingAction = new QAction("&Y Axis Auto Scaling", this);
    yAxisAutoScalingAction->setCheckable(true);
    yAxisAutoScalingAction->setChecked(false);
    cubeMode1Action = new QAction("3D view mode &1", this);
    cubeMode1Action->setCheckable(true);
    cubeMode1Action->setChecked(true);
    cubeMode2Action = new QAction("3D view mode &2", this);
    cubeMode2Action->setCheckable(true);
    cubeMode4Action = new QAction("3D view mode &4", this);
    cubeMode4Action->setCheckable(true);

    viewMenu->addAction(graphAction);
    viewMenu->addAction(orientationGraphAction);
    viewMenu->addAction(pressureGraphAction);

    viewMenu->addAction(threedAction);
    viewMenu->addAction(fieldMapAction);
    viewMenu->addSeparator();
    viewMenu->addAction(cubeMode1Action);
    viewMenu->addAction(cubeMode2Action);
    viewMenu->addAction(cubeMode4Action);
    viewMenu->addSeparator();
    viewMenu->addAction(xAxisAutoScrollingAction);
    viewMenu->addAction(yAxisAutoScalingAction);
    viewMenu->addSeparator();
    viewMenu->addAction(loadObjFileAction);

    toolbar->addSeparator();

    QWidget *stretchWidget = new QWidget();
    stretchWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    toolbar->addWidget(stretchWidget);

    toolbar->addSeparator();
    toolbar->addAction(graphAction);
    toolbar->addAction(orientationGraphAction);
    toolbar->addAction(pressureGraphAction);
    toolbar->addAction(threedAction);
    toolbar->addAction(fieldMapAction);

    QMenu* expertMenu = menuBar()->addMenu("&Advanced");
    QAction* firmwareAction = new QAction("Upload firmware", this);
    QAction* iapAction = new QAction("Upload IAP", this);
    // selfTestAction = new QAction("Start self test", this);
    QAction* latencyAction = new QAction("Measure latency", this);
    QAction* getFieldMapAction = new QAction("Get field map", this);
    QAction* versionAction = new QAction("Version info", this);

    expertMenu->addAction(firmwareAction);
    expertMenu->addAction(iapAction);
    expertMenu->addSeparator();
    // expertMenu->addAction(selfTestAction);
    // expertMenu->addSeparator();
    expertMenu->addAction(mACalculateAction);
    expertMenu->addAction(gyrMaCalculateAction);
    expertMenu->addAction(magMaCalculateAction);
    expertMenu->addAction(magAutoMaCalculateAction);
    expertMenu->addSeparator();
    expertMenu->addAction(versionAction);

    connect(startMagAction, SIGNAL(triggered()), this, SLOT(calibrateMag()));
    connect(startPlanarMagAction, SIGNAL(triggered()), this, SLOT(calibratePlanarMag()));
    connect(startAction, SIGNAL(triggered()), this, SLOT(startMeasurement()));
    connect(gyroAction, SIGNAL(triggered()), this, SLOT(recalibrate()));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(recordData()));
    connect(browseAction, SIGNAL(triggered()), this, SLOT(browseRecordFile()));
    connect(saveCalAction, SIGNAL(triggered()), this, SLOT(saveCalibration()));
    connect(connectAction, SIGNAL(triggered()), this, SLOT(openSensor()));
    connect(disconnectAction, SIGNAL(triggered()), this, SLOT(closeSensor()));
    connect(graphAction, SIGNAL(triggered()), this, SLOT(selectGraphWindow()));
    connect(orientationGraphAction, SIGNAL(triggered()), this, SLOT(selectGraph2Window()));
    connect(pressureGraphAction, SIGNAL(triggered()), this, SLOT(selectGraph3Window()));
    connect(threedAction, SIGNAL(triggered()), this, SLOT(selectThreeDWindow()));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));
    connect(firmwareAction, SIGNAL(triggered()), this, SLOT(uploadFirmware()));
    connect(iapAction, SIGNAL(triggered()), this, SLOT(uploadIap()));
    // connect(selfTestAction, SIGNAL(triggered()), this, SLOT(toggleSelfTest()));
    connect(latencyAction, SIGNAL(triggered()), this, SLOT(measureLatency()));
    connect(getFieldMapAction, SIGNAL(triggered()), this, SLOT(getFieldMap()));
    connect(versionAction, SIGNAL(triggered()), this, SLOT(getVersionInfo()));
    connect(fieldMapAction, SIGNAL(triggered()), this, SLOT(selectFieldMapWindow()));
    connect(resetToFactoryAction, SIGNAL(triggered()), this, SLOT(resetToFactory()));
    connect(mACalculateAction, SIGNAL(triggered()), this, SLOT(misalignmentCal()));
    connect(gyrMaCalculateAction, SIGNAL(triggered()), this, SLOT(gyrMisalignmentCal()));
    connect(magMaCalculateAction, SIGNAL(triggered()), this, SLOT(magMisalignmentCal()));
    connect(magAutoMaCalculateAction, SIGNAL(triggered()), this, SLOT(magAutoMisalignmentCal()));
    connect(loadFromFileAction, SIGNAL(triggered()), this, SLOT(loadCalibrationData()));
    connect(saveToFileAction, SIGNAL(triggered()), this, SLOT(saveCalibrationData()));
    connect(addRemoveAction, SIGNAL(triggered()), this, SLOT(addRemoveDevices()));
    connect(cubeMode1Action, SIGNAL(triggered()), this, SLOT(selectCubeMode1()));
    connect(cubeMode2Action, SIGNAL(triggered()), this, SLOT(selectCubeMode2()));
    connect(cubeMode4Action, SIGNAL(triggered()), this, SLOT(selectCubeMode4()));
    connect(replayAction, SIGNAL(triggered()), this, SLOT(startReplay()));
    connect(browseReplayAction, SIGNAL(triggered()), this, SLOT(browsePlaybackFile()));
    connect(setOffsetAction, SIGNAL(triggered()), this, SLOT(setOffset()));
    connect(resetOffsetAction, SIGNAL(triggered()), this, SLOT(resetOffset()));
    connect(timestampResetAction, SIGNAL(triggered()), this, SLOT(timestampReset()));
    connect(softwareSyncStartAction, SIGNAL(triggered()), this, SLOT(softwareSyncStart()));
    // connect(softwareSyncStopAction, SIGNAL(triggered()), this, SLOT(softwareSyncStop()));
    connect(loadObjFileAction, SIGNAL(triggered()), this, SLOT(loadObjFile()));
    connect(xAxisAutoScrollingAction, SIGNAL(triggered()), this, SLOT(xAxisAutoScrolling()));
    connect(yAxisAutoScalingAction, SIGNAL(triggered()), this, SLOT(yAxisAutoScaling()));

    connect(startFlashLogAction, SIGNAL(triggered()), this, SLOT(startFlashLog()));
    connect(stopFlashLogAction, SIGNAL(triggered()), this, SLOT(stopFlashLog()));
    connect(clearFlashLogAction, SIGNAL(triggered()), this, SLOT(clearFlashLog()));
    connect(eraseFlashAction, SIGNAL(triggered()), this, SLOT(eraseFlash()));
    connect(saveFlashLogAction, SIGNAL(triggered()), this, SLOT(saveFlashLog()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (exitWindow() == true) {
        event->accept();
    }
    else {
        event->ignore();
    }
}

void MainWindow::selectGraphWindow(void)
{
    cubeWindowContainer->hide();
    fieldMapWindow->hide();

    graphWindow->setMode(GRAPH_MODE_RAW);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

void MainWindow::selectGraph2Window(void)
{
    cubeWindowContainer->hide();
    fieldMapWindow->hide();

    graphWindow->setMode(GRAPH_MODE_ORIENTATION);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

void MainWindow::selectGraph3Window(void)
{
    cubeWindowContainer->hide();
    fieldMapWindow->hide();

    graphWindow->setMode(GRAPH_MODE_PRESSURE);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

void MainWindow::selectHeaveMotionWindow(void)
{
    cubeWindowContainer->hide();
    fieldMapWindow->hide();

    graphWindow->setMode(GRAPH_MODE_HEAVEMOTION);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

void MainWindow::selectThreeDWindow(void)
{
    graphWindow->hide();
    fieldMapWindow->hide();

    cubeWindowContainer->update();
    cubeWindowContainer->show();

    mode = MODE_THREED_WIN;
}

void MainWindow::selectFieldMapWindow(void)
{
    graphWindow->hide();
    cubeWindowContainer->hide();

    fieldMapWindow->show();

    mode = MODE_FIELDMAP_WIN;
}

void MainWindow::selectCubeMode1(void)
{
    cubeMode1Action->setChecked(true);
    cubeMode2Action->setChecked(false);
    cubeMode4Action->setChecked(false);

    cubeWindowContainer->selectCube(CUBE_VIEW_MODE_1);
}

void MainWindow::selectCubeMode2(void)
{
    cubeMode1Action->setChecked(false);
    cubeMode2Action->setChecked(true);
    cubeMode4Action->setChecked(false);

    cubeWindowContainer->selectCube(CUBE_VIEW_MODE_2);
}

void MainWindow::selectCubeMode4(void)
{
    cubeMode1Action->setChecked(false);
    cubeMode2Action->setChecked(false);
    cubeMode4Action->setChecked(true);

    cubeWindowContainer->selectCube(CUBE_VIEW_MODE_4);
}

bool MainWindow::exitWindow(void)
{
    QMessageBox msgBox;

    closeSensor();

    // mbcom.stopServer();

    return true;
}

void MainWindow::timerUpdate(void)
{
    if (softwareSyncCount > 0)
    {
        softwareSyncCount--;
        if (softwareSyncCount == 0)
            softwareSyncStop();
    }
    list<SensorGuiContainer *>::iterator it;

    int si = 0;
    ImuData imuData;

    // if (mm.measure() < GRAPH_UPDATE_PERIOD) return;

    if (rePlayer->isPlaying() == true) {

        int nData = rePlayer->hasImuData();
        for (int i = 0; i < nData; ++i)
        {
            imuData = rePlayer->getData();

            switch (mode) {
            case MODE_GRAPH_WIN:
                graphWindow->plotDataSet(imuData);
                break;

            case MODE_THREED_WIN:
                if (cubeWindowContainer->getMode() == CUBE_VIEW_MODE_1) {
                    cubeWindowContainer->getSelectedCube()->updateData(imuData);
                }
                break;
            }
        }
    }
    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {

        (*it)->updateData();

        if ((*it)->getSensor()->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
            (*it)->getSensor()->getSensorStatus() == SENSOR_STATUS_RUNNING) {
            (*it)->setupSensorGuiContainer();
            (*it)->checkOptionalFeatures();
            checkOptionalFeatures((*it)->getSensor());
            if (mode == MODE_THREED_WIN && ((cubeWindowContainer->getMode() == CUBE_VIEW_MODE_2) || (cubeWindowContainer->getMode() == CUBE_VIEW_MODE_4))) {
                
                if ((*it)->getSensor()->hasImuData() == 0) break;

                while ((*it)->getSensor()->hasImuData() > 0) {
                    imuData = (*it)->getSensor()->getCurrentData();

                    gb->setTitle("Data view (t: " + QString::number(imuData.timeStamp,'f', 1) + "s)");
                }

                cubeWindowContainer->getSelectedCube()->updateData(imuData);
            }
            else if ((*it) == currentLpms) {
#ifdef USE_CALLBACK
                char id[64];

                (*it)->getSensor()->getDeviceId(id);

                for (int i = 0; i < deviceList.nDevices; ++i) {
                    if (strcmp(deviceList.device[i].deviceId, id) == 0) {
                        imuData = deviceList.device[i].data;
                    }
                }
                switch (mode) {
                case MODE_GRAPH_WIN:
                    graphWindow->plotDataSet(imuData);
                    break;

                case MODE_THREED_WIN:
                    if (cubeWindowContainer->getMode() == CUBE_VIEW_MODE_1) {
                        cubeWindowContainer->getSelectedCube()->updateData(imuData);
                    }
                    break;

                case MODE_FIELDMAP_WIN:
                    if ((*it)->getSensor()->hasNewFieldMap() == true) {
                        (*it)->getSensor()->getFieldMap((*it)->fieldMap);
                        (*it)->getSensor()->getHardIronOffset((*it)->hardIronOffset);
                        (*it)->getSensor()->getSoftIronMatrix((*it)->softIronMatrix, &((*it)->fieldRadius));
                        fieldMapWindow->updateFieldMap((*it)->fieldMap, (*it)->hardIronOffset, (*it)->softIronMatrix, (*it)->fieldRadius, imuData.b);
                    }

                    char id[64];
                    (*it)->getSensor()->getDeviceId(id);
                    fieldMapWindow->updateCurrentField((*it)->getSensor()->getFieldNoise(), imuData.b);
                    break;
                }
#else 
                // if ((*it)->getSensor()->hasImuData() == 0) break;

                // while ((*it)->getSensor()->hasImuData() > 0) {
                    // imuData = (*it)->getSensor()->getCurrentData();
                // }
                // imuData = (*it)->getSensor()->getCurrentData();
                
                int nData = (*it)->getSensor()->hasImuData();
                for (int i = 0; i < nData; ++i)
                {
                    // Use getNextData as validation of the new interface. This won't block because we
                    // checked that data is available.
                    (*it)->getSensor()->getNextData(imuData);
                    gb->setTitle("Data view (t: " + QString::number(imuData.timeStamp,'f', 1) + "s)");

                    switch (mode) {
                    case MODE_GRAPH_WIN:
                        graphWindow->plotDataSet(imuData);
                    break;

                    case MODE_THREED_WIN:
                        if (cubeWindowContainer->getMode() == CUBE_VIEW_MODE_1) {
                            cubeWindowContainer->getSelectedCube()->updateData(imuData);
                        }
                    break;

                    case MODE_FIELDMAP_WIN:
                        if ((*it)->getSensor()->hasNewFieldMap() == true) {
                            (*it)->getSensor()->getFieldMap((*it)->fieldMap);
                            (*it)->getSensor()->getHardIronOffset((*it)->hardIronOffset);
                            (*it)->getSensor()->getSoftIronMatrix((*it)->softIronMatrix, &((*it)->fieldRadius));
                            fieldMapWindow->updateFieldMap((*it)->fieldMap, (*it)->hardIronOffset, (*it)->softIronMatrix, (*it)->fieldRadius, imuData.b);
                        }

                        char id[64];
                        (*it)->getSensor()->getDeviceId(id);
                        fieldMapWindow->updateCurrentField((*it)->getSensor()->getFieldNoise(), imuData.b);
                    break;
                    }
                }
#endif
            }
        }
    }
}

#ifdef USE_CALLBACK
void lpmsCallback(ImuData d, const char* id)
{
    for (int i = 0; i < deviceList.nDevices; ++i) {
        if (strcmp(deviceList.device[i].deviceId, id) == 0) {
            deviceList.device[i].data = d;
        }
    }
}
#endif

void MainWindow::updateCanBaudrate(int i)
{
    sm->setCanBaudrate(i);
}

void MainWindow::updateRs232Baudrate(int i)
{
    sm->setRs232Baudrate(i);
}

void MainWindow::updateCurrentLpms(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
    bool f;
    int i;
    int fi;

    std::list<SensorGuiContainer *>::iterator it;
    std::list<SensorGuiContainer *>::iterator fit;

    f = false;
    i = 0;
    fi = 0;

    if (lpmsTree->topLevelItemCount() == 0 || lpmsTree->currentItem() == NULL) {
        return;
    }

    QTreeWidgetItem *temp;
    QTreeWidgetItem *wi = lpmsTree->currentItem();
    if (wi->childCount() > 0) {
        QTreeWidgetItem *si = wi->child(0);
        temp = (QTreeWidgetItem *)lpmsTree->itemWidget(si, 0);
    }
    else {
        temp = (QTreeWidgetItem *)lpmsTree->itemWidget(wi, 0);
    }

    QTreeWidgetItem *checkItem = lpmsTree->currentItem();
    int itemIndex = lpmsTree->indexOfTopLevelItem(checkItem);

    while (itemIndex == -1) {
        checkItem = checkItem->parent();
        itemIndex = lpmsTree->indexOfTopLevelItem(checkItem);
    }

    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->updateData();
        if (itemIndex == lpmsTree->indexOfTopLevelItem((*it)->treeItem)) {
            f = true;
            fit = it;
            fi = i;
        }
        ++i;
    }

    if (f == true) {
        currentLpms = *fit;
        currentLpms->openMatId = (*fit)->getSensor()->getOpenMatId();

        graphWindow->setActiveLpms((*fit)->getSensor()->getOpenMatId());
        updateMagneticFieldMap();

    }
    else {
        if (lpmsTree->topLevelItemCount() > 0) {
            currentLpms = lpmsList.front();
            updateMagneticFieldMap();
        }
        else {
            currentLpms = 0;
            startButton->setStyleSheet("QPushButton { color: black; }");
            startButton->setText("Start measurement");
            isRunning = false;
        }
    }
    graphWindow->clearGraphs();

    currentLpms->getSensor()->getConfigurationPrm(PRM_DEVICE_TYPE, &i);
    if (i == DEVICE_LPMS_B2) {
        enableFlashMenu();
    } else {
        disableFlashMenu();
    }
}

void MainWindow::checkOptionalFeatures(LpmsSensorI* sensor)
{
    int i;

    sensor->getConfigurationPrm(PRM_HEAVEMOTION_ENABLED, &i);
    if (i == SELECT_HEAVEMOTION_ENABLED && heaveMotionEnabled == false) {
        heaveMotionEnabled = true;

        QAction* heaveMotionGraphAction = new QAction(QIcon("./icons/bars_32x32.png"), "Heave motion window", this);

        viewMenu->addSeparator();
        viewMenu->addAction(heaveMotionGraphAction);
        toolbar->addAction(heaveMotionGraphAction);
        connect(heaveMotionGraphAction, SIGNAL(triggered()), this, SLOT(selectHeaveMotionWindow()));
    }
}

void MainWindow::openSensor(void)
{
    int mode;
    bool f;
    char cStr[64];

    if (isConnecting == true) return;
    if (deviceList.getNDevices() == 0) return;
    mode = deviceList.getDeviceType(comboDeviceList->currentIndex());

    isConnecting = true;
    std::string deviceAddress = std::string(deviceList.getDeviceId(comboDeviceList->currentIndex()));

    f = false;
    std::list<SensorGuiContainer *>::iterator it;
    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->getSensor()->getDeviceId(cStr);
        if (strcmp(cStr, deviceAddress.c_str()) == 0) {
            f = true;
        }
    }

    if (f == true) {
        std::cout << "[LpmsControl] Device " << deviceAddress.c_str() << " is already connected" << std::endl;
        isConnecting = false;
        return;
    }

    stopMeasurement();
    graphWindow->clearGraphs();

    LpmsSensorI *lpmsDevice = sm->addSensor(mode, deviceAddress.c_str());

    currentLpms = new SensorGuiContainer(lpmsDevice, lpmsTree);
    // mbcom.addSensor(lpmsDevice);

    currentLpms->updateData();
   
    lpmsList.push_back(currentLpms);

    lpmsTree->insertTopLevelItem(0, currentLpms->treeItem);
    lpmsTree->setCurrentItem(currentLpms->treeItem);

    currentLpms->treeItem->setExpanded(true);

#ifdef USE_CALLBACK
    lpmsDevice->setCallback(&lpmsCallback);
#endif

    isConnecting = false;
    startMeasurement();
}

void MainWindow::closeSensor(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    stopMeasurement();

    if (currentLpms) {
        SensorGuiContainer *temp = currentLpms;

        // mbcom.removeSensor(temp->getSensor());

        sm->removeSensor(temp->getSensor());
        lpmsList.remove(temp);

        delete temp->treeItem;
        currentLpms = 0;

        delete temp;
        updateCurrentLpms();
    }
    startMeasurement();
    gb->setTitle("Data view");
}

void MainWindow::toggleSelfTest(void)
{
    int i;

    if (currentLpms == 0 || isConnecting == true) return;

    currentLpms->getSensor()->getConfigurationPrm(PRM_SELF_TEST, &i);

    if (i == SELECT_SELF_TEST_OFF) {
        selfTestAction->setText("Start self test");
        currentLpms->getSensor()->setConfigurationPrm(PRM_SELF_TEST, SELECT_SELF_TEST_ON);
    }
    else {
        selfTestAction->setText("Stop self test");
        currentLpms->getSensor()->setConfigurationPrm(PRM_SELF_TEST, SELECT_SELF_TEST_OFF);
    }
}

void MainWindow::startMeasurement(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (isConnecting == true) return;

    if (isRunning == false && lpmsList.size() > 0) {
        stopReplay();

        for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
            (*it)->getSensor()->run();
        }

        startAction->setText("Stop measurement");
        startAction->setIcon(QIcon("./icons/pause_24x32.png"));

        graphWindow->clearGraphs();

        isRunning = true;
    }
    else {
        for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
            (*it)->getSensor()->pause();
        }

        startAction->setText("Start measurement");
        startAction->setIcon(QIcon("./icons/play_24x32.png"));

        isRunning = false;
    }
}

void MainWindow::stopMeasurement(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->getSensor()->pause();
    }

    startAction->setText("Start measurement");
    startAction->setIcon(QIcon("./icons/play_24x32.png"));

    isRunning = false;
}

void MainWindow::setOffset(void)
{
    std::list<SensorGuiContainer *>::iterator it;
    int rm = 0;

    if (currentLpms == 0 || isConnecting == true) return;

    rm = resetMethodCombo->currentIndex();

    if (targetCombo->currentIndex() == 0) {
        for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
            (*it)->getSensor()->setOrientationOffset(rm);
        }
    }
    else {
        currentLpms->getSensor()->setOrientationOffset(rm);
    }
}

void MainWindow::resetOffset(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpms == 0 || isConnecting == true) return;

    if (targetCombo->currentIndex() == 0) {
        for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
            (*it)->getSensor()->resetOrientationOffset();
        }
    }
    else {
        currentLpms->getSensor()->resetOrientationOffset();
    }
}

void MainWindow::timestampReset(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpms == 0 || isConnecting == true) return;

    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->getSensor()->setTimestamp(0.0f);
    }
}

void MainWindow::softwareSyncStart(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpms == 0 || isConnecting == true) return;

    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->getSensor()->startSync();
    }
    softwareSyncCount = 1000;
}

void MainWindow::softwareSyncStop(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpms == 0 || isConnecting == true) return;

    for (it = lpmsList.begin(); it != lpmsList.end(); ++it) {
        (*it)->getSensor()->stopSync();
    }
}

void MainWindow::getGyroStaticBias(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    currentLpms->getSensor()->getGyroStaticBias();
}

void MainWindow::recalibrate(void)
{
    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    if (isRunning == false) {
        startMeasurement();
    }

    currentLpms->getSensor()->startCalibrateGyro();

    startWaitBar(60);
}

void MainWindow::uploadFirmware(void)
{
    QMessageBox msgBox;
    QMessageBox msgBox2;
    int ret;
    int i;
    bool f = true;
    QString qfilename;
    string fn;

    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    msgBox.setText("By uploading an invalid firmware file, the sensor can become "
        "in-operable. Are you sure that you understand what you are doing and "
        "would like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    qfilename = QFileDialog::getOpenFileName(this, "Open firmware file", "./", "Binaries (*.bin)");
    fn = qfilename.toStdString();

    if (fn == "") return;

    currentLpms->getSensor()->getConfigurationPrm(PRM_DEVICE_TYPE, &i);

    // if (i == DEVICE_LPMS_B || i == DEVICE_LPMS_BLE) {
        // if (!qfilename.contains("LpmsB") || !qfilename.contains("bin")) {
            // printf("[MainWindow] LPMS-B invalid firmware filename.\n");
            // f = false;

        // }
    // } else {
        // if (!qfilename.contains("LpmsCU") || !qfilename.contains("bin")) {
            // printf("[MainWindow] LPMS-CU Invalid firmware filename.\n");
            // f = false;
        // }
    // }

    if (!qfilename.contains("bin")) {
        f = false;
    }

    QFile file(qfilename);
    if (!file.open(QIODevice::ReadOnly)) {
        printf("[MainWindow] Couldn't open firmware file.\n");
        f = false;
    }
    if (file.size() < 10000 || file.size() > 100000) {
        printf("[MainWindow] Bad firmware filesize: %d.\n", (int)file.size());
        f = false;
    }
    file.close();

    if (f == false) {
        msgBox2.setText("Invalid firmware file. Please confirm that you selected the right file for upload.");
        msgBox2.setStandardButtons(QMessageBox::Ok);
        msgBox2.exec();

        return;
    }

    if (!(fn == "")) {
        currentLpms->getSensor()->uploadFirmware(fn.c_str());
    }

    uploadProgress = new QProgressDialog("Uploading data, please don't turn off your device...", "Cancel", 0, 200, this);
    uploadProgress->setWindowFlags(uploadProgress->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    uploadProgress->setWindowModality(Qt::WindowModal);
    uploadProgress->setMinimumWidth(400);
    uploadProgress->setAutoReset(false);
    uploadProgress->setCancelButton(0);
    uploadProgress->show();

    uploadTimer = new QTimer(this);
    connect(uploadTimer, SIGNAL(timeout()), this, SLOT(uploadTimerUpdate()));
    connect(uploadProgress, SIGNAL(canceled()), this, SLOT(cancelUpload()));

    uploadTimer->start(500);
}

void MainWindow::cancelUpload(void)
{
}

void MainWindow::uploadTimerUpdate(void)
{
    int p;

    if (currentLpms->getSensor()->getUploadProgress(&p) == 1) {
        uploadProgress->setValue(p);
        uploadProgress->show();
    }
    else {
        QMessageBox msgBox;

        delete uploadTimer;
        delete uploadProgress;

        msgBox.setText("Upload has been finished.");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWindow::uploadIap(void)
{
    QMessageBox msgBox;
    int ret;

    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    msgBox.setText("By uploading an invalid IAP file, the sensor can become "
        "in-operable. Are you sure that you understand what you are doing and "
        "would like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    QString qfilename = QFileDialog::getOpenFileName(this, "Open IAP file", "./", "Binaries (*.bin)");
    string fn = qfilename.toStdString();

    // if (!qfilename.contains("LpmsCRS_IAP.bin")) {
        // msgBox.setText("Invalid IAP file. Please confirm that you selected the right file for upload.");
        // msgBox.exec();

        // return;
    // }

    if (!(fn == "")) {
        currentLpms->getSensor()->uploadIap(fn.c_str());
    }

    uploadProgress = new QProgressDialog("Uploading data, please don't turn off your device...", "Cancel", 0, 200, this);
    uploadProgress->setWindowFlags(uploadProgress->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    uploadProgress->setWindowModality(Qt::WindowModal);
    uploadProgress->setMinimumWidth(400);
    uploadProgress->setAutoReset(false);
    uploadProgress->setCancelButton(0);
    uploadProgress->show();

    uploadTimer = new QTimer(this);
    connect(uploadTimer, SIGNAL(timeout()), this, SLOT(uploadTimerUpdate()));
    uploadTimer->start(500);
}

void MainWindow::recordData(void)
{
    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    if (recordFileSet == false) {
        browseRecordFile();
    }

    if (sm->isRecordingActive() == false) {
        stopMeasurement();

        if (!(globalRecordFile == "")) {
            if (sm->saveSensorData(globalRecordFile.c_str()) == true) {
                saveAction->setText("Stop recording");
                saveAction->setIcon(QIcon("./icons/x_alt_32x32.png"));
            }
        }
        startMeasurement();
    }
    else {
        sm->stopSaveSensorData();

        saveAction->setText("Record data");
        saveAction->setIcon(QIcon("./icons/layers_32x28.png"));
    }
}

void MainWindow::browseRecordFile(void)
{
    QString qFilename = QFileDialog::getSaveFileName(this, "Save sensor data", "./", "*.csv");
    string recordFilename = qFilename.toStdString();
    if (!qFilename.endsWith(".csv"))
        recordFilename += ".csv";

    if (!(recordFilename == "")) {
        globalRecordFile = recordFilename;
        recordFileEdit->setText(recordFilename.c_str());
        recordFileSet = true;
    }
}

void MainWindow::saveCalibration(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    currentLpms->getSensor()->saveCalibrationData();

    startWaitBar(5);
}

void MainWindow::addRemoveDevices(void)
{
    stopMeasurement();

    rescanD->show();
}

void MainWindow::loadCalibrationData(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    QString qfn = QFileDialog::getOpenFileName(this, "Load calibration data", "./", "");
    string fn = qfn.toStdString();

    if (!(fn == "")) {
        currentLpms->getSensor()->loadCalibrationData(fn.c_str());
    }
}

void MainWindow::saveCalibrationData(void)
{
    if (currentLpms == 0 || isConnecting == true) return;


    char cStr[64];

    currentLpms->sensor->getConfigurationPrm(PRM_DEVICE_ID, cStr);
    string sensorName(cStr); 
    sensorName.erase(std::remove(sensorName.begin(), sensorName.end(), ':'), sensorName.end());
    QString qfn = QFileDialog::getSaveFileName(this, "Save calibration data", QString::fromStdString(sensorName)+".xml", "");
    string fn = qfn.toStdString();

    if (!(fn == "")) {
        currentLpms->getSensor()->saveCalibrationData(fn.c_str());
    }
}

void MainWindow::updateLpmsFps(int v, int lpmsId)
{
    if (lpmsId == currentLpms->getSensor()->getOpenMatId()) {
        imuFpsLabel->setText(QString("%1").arg(v));
    }
}

void MainWindow::measureLatency(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    currentLpms->getSensor()->measureAvgLatency();
}

void MainWindow::getVersionInfo(void)
{
    QString openMATVersion(LPMS_CONTROL_VERSION);
    QMessageBox::about(this, "LP-RESEARCH - LpmsControl", QString("LP-RESEARCH - LpmsControl ") 
        + openMATVersion 
        + QString("\n(c) LP-Research\nhttp://www.lp-research.com\n\nRelease information: https://bitbucket.org/lpresearch/openmat/wiki/Home"));
}

void MainWindow::resetToFactory(void)
{
    QMessageBox msgBox;
    int ret;

    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    msgBox.setText("This command sets the current sensor parameter "
        "to their factory defaults. Would you like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    currentLpms->getSensor()->resetToFactorySettings();
    // currentLpms->getSensor()->hardReset();
}

void MainWindow::startWaitBar(int t)
{
    if (maIsCalibrating == true) {
        maWizard->hide();
    }

    calProgress = new QProgressDialog("Calibrating LPMS. Please wait..", QString() /* "Cancel" */, 0, t, this);
    calProgress->setWindowFlags(calProgress->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    calProgress->setWindowModality(Qt::WindowModal);
    calProgress->setMinimumWidth(400);
    calProgress->setAutoReset(false);
    calProgress->show();

    calTimer = new QTimer(this);
    connect(calTimer, SIGNAL(timeout()), this, SLOT(calTimerUpdate()));
    calTimer->start(500);

    calMaxTime = t;
    calTime = 0;
}

void MainWindow::calTimerUpdate(void)
{
    ++calTime;

    if (calTime > calMaxTime) {
        delete calProgress;
        delete calTimer;

        if (maIsCalibrating == true) {
            maWizard->show();
        }

        if (isMagEllipsoidCalOn == true) {
            magEllipsoidCalWizard->show();
            if (magEllipsoidCalWizard->currentId() == 2) isMagEllipsoidCalOn = false;
        }

        if (isMagPlanarCalOn == true) {
            magPlanarCalWizard->show();
            if (magPlanarCalWizard->currentId() == 2) isMagPlanarCalOn = false;
        }
    }
    else {
        calProgress->setValue(calTime);
    }
}

void MainWindow::magAutoMisalignmentCal(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    if (isRunning == false) startMeasurement();

    currentLpms->getSensor()->startAutoMagMisalignCal();

    startWaitBar(45);
}

void MainWindow::magMisalignmentCal(void)
{
    QMessageBox msgBox;
    int ret;

    if (currentLpms == 0 || isConnecting == true) return;

    msgBox.setText("Changing the magnetometer misalignment matrix can significantly "
        "affect the performance of your sensor. Before setting a new "
        "misalignment matrix, please make sure that you know what you "
        "are doing. Anyway, parameters will not be saved to the internal "
        "flash memory of the sensor until you choose to save calibration "
        "parameters. Also you can backup your current parameters by"
        "saving them to a file (see the Calibration menu).");

    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    maWizard = new QWizard(this);
    QList<QWizard::WizardButton> layout;
    layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton << QWizard::FinishButton;
    maWizard->setButtonLayout(layout);

    maWizard->addPage(maOrientationPage("Z-AXIS UP - Coils OFF", "Z-AXIS facing UPWARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("Z-AXIS UP - Coils ON", "Z-AXIS facing UPWARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(maOrientationPage("Z-AXIS DOWN - Coils OFF", "Z-AXIS facing DOWNARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("Z-AXIS DOWN - Coils ON", "Z-AXIS facing DOWNARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(maOrientationPage("X-AXIS UP - Coils OFF", "X-AXIS facing UPWARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("X-AXIS UP - Coils ON", "X-AXIS facing UPWARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(maOrientationPage("X-AXIS DOWN - Coils OFF", "X-AXIS facing DOWNARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("X-AXIS DOWN - Coils ON", "X-AXIS facing DOWNARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(maOrientationPage("Y-AXIS UP - Coils OFF", "Y-AXIS facing UPWARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("Y-AXIS UP - Coils ON", "Y-AXIS facing UPWARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(maOrientationPage("Y-AXIS DOWN - Coils OFF", "Y-AXIS facing DOWNWARDS. Switch Helmholtz coils OFF. "));
    maWizard->addPage(maOrientationPage("Y-AXIS DOWN - Coils ON", "Y-AXIS facing DOWNWARDS. Switch Helmholtz coils ON. "));

    maWizard->addPage(gyrMaFinishedPage());

    maWizard->setWindowTitle("Magnetometer Misalignment Calibration");
    maWizard->show();

    connect(maWizard, SIGNAL(currentIdChanged(int)), this, SLOT(magMaNewPage(int)));
    connect(maWizard, SIGNAL(finished(int)), this, SLOT(magMaFinished(int)));

    currentLpms->getSensor()->initMagMisalignCal();

    maCalibrationFinished = false;
    maIsCalibrating = true;
}

void MainWindow::magMaNewPage(int i)
{
    currentLpms->getSensor()->startGetGyrMisalign(i - 1);
    if (i == 12) maCalibrationFinished = true;
    startWaitBar(10);
}

void MainWindow::magMaFinished(int i)
{
    if (maCalibrationFinished == true) {
        currentLpms->getSensor()->calcGyrMisalignMatrix();
    }

    maIsCalibrating = false;
}

QWizardPage *MainWindow::magMaOrientationPage(const char* ts, const char* es)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle((std::string("Calibrating to orientation: ") + std::string(ts)).c_str());

    QLabel *label = new QLabel((std::string("Please put the selected LPMS on a horizontal surface, with the ") + std::string(es) + std::string("After aligning the sensor in this orientation press the next button. Please do not move the sensor for around 3s.")).c_str());
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

QWizardPage *MainWindow::magMaFinishedPage(void)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle("Calibration finished");

    QLabel *label = new QLabel("Misalignment calibration has been finished successfully.");
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

void MainWindow::gyrMisalignmentCal(void)
{
    QMessageBox msgBox;
    int ret;

    if (currentLpms == 0 || isConnecting == true) return;

    msgBox.setText("Changing the gyroscope misalignment matrix can significantly "
        "affect the performance of your sensor. Before setting a new "
        "misalignment matrix, please make sure that you know what you "
        "are doing. Anyway, parameters will not be saved to the internal "
        "flash memory of the sensor until you choose to save calibration "
        "parameters. Also you can backup your current parameters by"
        "saving them to a file (see the Calibration menu).");

    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    maWizard = new QWizard(this);
    QList<QWizard::WizardButton> layout;
    layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton << QWizard::FinishButton;
    maWizard->setButtonLayout(layout);

    maWizard->addPage(gyrMaOrientationPage("X-AXIS", "right hand"));
    maWizard->addPage(gyrMaOrientationPage("Y-AXIS", "left hand"));
    maWizard->addPage(gyrMaOrientationPage("Z-AXIS", "right hand"));

    maWizard->addPage(gyrMaFinishedPage());

    maWizard->setWindowTitle("Gyroscope Misalignment Calibration");
    maWizard->show();

    connect(maWizard, SIGNAL(currentIdChanged(int)), this, SLOT(gyrMaNewPage(int)));
    connect(maWizard, SIGNAL(finished(int)), this, SLOT(gyrMaFinished(int)));

    currentLpms->getSensor()->initGyrMisalignCal();

    maCalibrationFinished = false;
    maIsCalibrating = true;
}

void MainWindow::gyrMaNewPage(int i)
{
    switch (i) {
    case 1:
        currentLpms->getSensor()->startGetGyrMisalign(0);
        startWaitBar(10);
        break;

    case 2:
        currentLpms->getSensor()->startGetGyrMisalign(1);
        startWaitBar(10);
        break;

    case 3:
        currentLpms->getSensor()->startGetGyrMisalign(2);
        maCalibrationFinished = true;
        startWaitBar(10);
        break;
    }
}

void MainWindow::gyrMaFinished(int i)
{
    if (maCalibrationFinished == true) {
        currentLpms->getSensor()->calcGyrMisalignMatrix();
    }

    maIsCalibrating = false;
}

QWizardPage *MainWindow::gyrMaOrientationPage(const char* ts, const char* es)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle((std::string("Rotation axis: ") + std::string(ts)).c_str());

    QLabel *label = new QLabel((std::string("Please LEFT-HAND rotate the sensor at a constant angular rate of 45 rpm, with its ") + std::string(ts) + std::string(" pointing UP.")).c_str());

    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

QWizardPage *MainWindow::gyrMaFinishedPage(void)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle("Calibration finished");

    QLabel *label = new QLabel("Misalignment calibration has been finished successfully.");
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

QWizardPage *MainWindow::magEllipsoidCalPage(int pageNumber)
{
    QWizardPage *page = new QWizardPage;
    QLabel *label;

    page->setTitle(std::string("Magnetic field map").c_str());

    switch (pageNumber) {
    case 0:
        page->setTitle(std::string("Magnetic field map").c_str());
        label = new QLabel((std::string("Please continuously rotate the selected LPMS around roll, pitch and yaw axis for 30s. The calibration algorithm will create a map of your environment magnetic field and use it to calculate hard / soft iron calibration parameters.")).c_str());
        break;

    case 1:
        page->setTitle(std::string("Alignment").c_str());
        label = new QLabel((std::string("Please hold the sensor in a motionless state for 10s. The alignment of the magnetometer inclination axis and the vertical axis (gravity vector) will be calculated.")).c_str());
        break;

    case 2:
        page->setTitle(std::string("Calibration finished").c_str());
        label = new QLabel((std::string("Magnetometer calibration has been finished. Please check the field map window if the number of acquired field points during the calibration process were enough to generate a good ellipsoid fit. See the manual for more information.")).c_str());
        break;
    }

    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

void MainWindow::magEllipsoidCalNewPage(int i)
{
    switch (i) {
    case 1:
        currentLpms->getSensor()->startMagCalibration();
        magEllipsoidCalWizard->hide();
        startWaitBar(45);
        break;

    case 2:
        currentLpms->getSensor()->startMagReferenceCal();
        magEllipsoidCalWizard->hide();
        startWaitBar(10);
        break;
    }
}

void MainWindow::calibrateMag(void)
{
    QList<QWizard::WizardButton> layout;

    if (currentLpms == 0 || isConnecting == true) return;

    magEllipsoidCalWizard = new QWizard(this);

    layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton << QWizard::FinishButton;
    magEllipsoidCalWizard->setButtonLayout(layout);

    magEllipsoidCalWizard->addPage(magEllipsoidCalPage(0));
    magEllipsoidCalWizard->addPage(magEllipsoidCalPage(1));
    magEllipsoidCalWizard->addPage(magEllipsoidCalPage(2));

    magEllipsoidCalWizard->setWindowTitle("Magnetometer hard / soft iron calibration (ellipsoid)");
    magEllipsoidCalWizard->show();

    connect(magEllipsoidCalWizard, SIGNAL(currentIdChanged(int)), this, SLOT(magEllipsoidCalNewPage(int)));

    isMagEllipsoidCalOn = true;

    if (isRunning == false) startMeasurement();
}

QWizardPage *MainWindow::magPlanarCalPage(int pageNumber)
{
    QWizardPage *page = new QWizardPage;
    QLabel *label;

    page->setTitle(std::string("Magnetic field map").c_str());

    switch (pageNumber) {
    case 0:
        page->setTitle(std::string("Magnetic field map").c_str());
        label = new QLabel((std::string("Please continuously rotate the selected LPMS around roll, pitch and yaw axis for 30s. The calibration algorithm will create a map of your environment magnetic field and use it to calculate hard / soft iron calibration parameters.")).c_str());
        break;

    case 1:
        page->setTitle(std::string("Alignment").c_str());
        label = new QLabel((std::string("Please hold the sensor in a motionless state for 5s. The alignment of the magnetometer inclination axis and the vertical axis (gravity vector) will be calculated.")).c_str());
        break;

    default:
        page->setTitle(std::string("Calibration finished").c_str());
        label = new QLabel((std::string("Magnetometer calibration has been finished. Please check the field map window if the calibration process was able to generate a good fit. See the manual for more information.")).c_str());
        break;
    }

    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

void MainWindow::magPlanarCalNewPage(int i)
{
    switch (i) {
    case 1:
        currentLpms->getSensor()->startPlanarMagCalibration();
        magPlanarCalWizard->hide();
        startWaitBar(45);
        break;

    case 2:
        currentLpms->getSensor()->startMagReferenceCal();
        magPlanarCalWizard->hide();
        startWaitBar(10);
        break;
    }
}

void MainWindow::calibratePlanarMag(void)
{
    QList<QWizard::WizardButton> layout;

    if (currentLpms == 0 || isConnecting == true) return;

    magPlanarCalWizard = new QWizard(this);

    layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton << QWizard::FinishButton;
    magPlanarCalWizard->setButtonLayout(layout);

    magPlanarCalWizard->addPage(magPlanarCalPage(0));
    magPlanarCalWizard->addPage(magPlanarCalPage(1));
    magPlanarCalWizard->addPage(magPlanarCalPage(2));

    magPlanarCalWizard->setWindowTitle("Magnetometer hard / soft iron calibration (planar)");
    magPlanarCalWizard->show();

    connect(magPlanarCalWizard, SIGNAL(currentIdChanged(int)), this, SLOT(magPlanarCalNewPage(int)));

    isMagPlanarCalOn = true;

    if (isRunning == false) startMeasurement();
}



QWizardPage *MainWindow::maOrientationPage(const char* ts, const char* es)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle((std::string("Calibrating to orientation: ") + std::string(ts)).c_str());

    QLabel *label = new QLabel((std::string("Please put the selected LPMS on a horizontal surface, with the ") + std::string(es) + std::string("After aligning the sensor in this orientation press the next button. Please do not move the sensor for around 3s.")).c_str());
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

QWizardPage *MainWindow::maFinishedPage(void)
{
    QWizardPage *page = new QWizardPage;

    page->setTitle("Calibration finished");

    QLabel *label = new QLabel("Misalignment calibration has been finished successfully.");
    label->setWordWrap(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    page->setLayout(layout);

    return page;
}

void MainWindow::misalignmentCal(void)
{
    QMessageBox msgBox;
    int ret;

    if (currentLpms == 0 || isConnecting == true) return;

    msgBox.setText("Changing the misalignment matrix can significantly "
        "affect the performance of your sensor. Before setting a new "
        "misalignment matrix, please make sure that you know what you "
        "are doing. Anyway, parameters will not be saved to the internal "
        "flash memory of the sensor until you choose to save calibration "
        "parameters. Also you can backup your current parameters by"
        "saving them to a file (see the Calibration menu).");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    msgBox.setDefaultButton(QMessageBox::No);
    ret = msgBox.exec();

    switch (ret) {
    case QMessageBox::Yes:
        break;

    case QMessageBox::No:
        return;
        break;

    default:
        break;
    }

    maWizard = new QWizard(this);
    QList<QWizard::WizardButton> layout;
    layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton << QWizard::FinishButton;
    maWizard->setButtonLayout(layout);

    maWizard->addPage(maOrientationPage("Z-AXIS UP", "Z-AXIS facing UPWARDS. "));
    maWizard->addPage(maOrientationPage("Z-AXIS DOWN", "Z-AXIS facing DOWNARDS. "));

    maWizard->addPage(maOrientationPage("X-AXIS UP", "X-AXIS facing UPWARDS. "));
    maWizard->addPage(maOrientationPage("X-AXIS DOWN", "X-AXIS facing DOWNARDS. "));

    maWizard->addPage(maOrientationPage("Y-AXIS UP", "Y-AXIS facing UPWARDS. "));
    maWizard->addPage(maOrientationPage("Y-AXIS DOWN", "Y-AXIS facing DOWNARDS. "));

    maWizard->addPage(maFinishedPage());

    maWizard->setWindowTitle("Accelerometer Misalignment Calibration");
    maWizard->show();

    connect(maWizard, SIGNAL(currentIdChanged(int)), this, SLOT(maNewPage(int)));
    connect(maWizard, SIGNAL(finished(int)), this, SLOT(maFinished(int)));

    currentLpms->getSensor()->initMisalignCal();

    maCalibrationFinished = false;
    maIsCalibrating = true;
}

void MainWindow::maNewPage(int i)
{
    switch (i) {
    case 1:
        currentLpms->getSensor()->startGetMisalign(0);
        startWaitBar(5);
        break;

    case 2:
        currentLpms->getSensor()->startGetMisalign(1);
        startWaitBar(5);
        break;

    case 3:
        currentLpms->getSensor()->startGetMisalign(2);
        startWaitBar(5);
        break;

    case 4:
        currentLpms->getSensor()->startGetMisalign(3);
        startWaitBar(5);
        break;

    case 5:
        currentLpms->getSensor()->startGetMisalign(4);
        startWaitBar(5);
        break;

    case 6:
        currentLpms->getSensor()->startGetMisalign(5);
        maCalibrationFinished = true;
        startWaitBar(5);
        break;
    }
}

void MainWindow::maFinished(int i)
{
    if (maCalibrationFinished == true) {
        currentLpms->getSensor()->calcMisalignMatrix();
    }

    maIsCalibrating = false;
}

void MainWindow::updateMagneticFieldMap(void)
{
    float fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3];
    float hardIronOffset[3];
    float softIronMatrix[3][3];
    float fieldRadius;
    ImuData imuData;

    currentLpms->getSensor()->getFieldMap(fieldMap);
    currentLpms->getSensor()->getHardIronOffset(hardIronOffset);
    currentLpms->getSensor()->getSoftIronMatrix(softIronMatrix, &fieldRadius);
    imuData = currentLpms->getSensor()->getCurrentData();

    fieldMapWindow->updateFieldMap(fieldMap, hardIronOffset, softIronMatrix, fieldRadius, imuData.b);
}

void MainWindow::getFieldMap(void)
{
    if (currentLpms == 0 || isConnecting == true) return;

    currentLpms->getSensor()->acquireFieldMap();
}

void MainWindow::startReplay(void)
{
    if (rePlayer->isPlaying() == false && globalPlaybackFile != "") {
        stopMeasurement();

        graphWindow->clearGraphs();

        rePlayer->readMotionDataFile(globalPlaybackFile);
        rePlayer->play();

        replayAction->setText("Stop playback");
        replayAction->setIcon(QIcon("./icons/stop_32x32.png"));
    }
    else {
        rePlayer->pause();
        replayAction->setIcon(QIcon("./icons/loop_alt2_32x28.png"));
    }
}

void MainWindow::stopReplay(void)
{
    rePlayer->pause();
    replayAction->setIcon(QIcon("./icons/loop_alt2_32x28.png"));
}

void MainWindow::browsePlaybackFile(void)
{
    QString qFilename = QFileDialog::getOpenFileName(this, "Load sensor data", "./", "*.csv");
    string playbackFilename = qFilename.toStdString();

    if (!(playbackFilename == "")) {
        globalPlaybackFile = playbackFilename;
        playbackFileEdit->setText(playbackFilename.c_str());
        playbackFileSet = true;
    }
}

void MainWindow::loadObjFile(void)
{
    QString qFilename = QFileDialog::getOpenFileName(this, "Load 3D OBJ file", "./", "OBJ files (*.obj)");
    string objFilename = qFilename.toStdString();

    if (!(objFilename == "")) {
        cubeWindowContainer->getSelectedCube()->loadObjFile(objFilename);
    }
}

void MainWindow::xAxisAutoScrolling(void)
{
    if (xAxisAutoScrollingAction->isChecked())
    {
        graphWindow->setXAxisAutoScrolling(true);
        xAxisAutoScrollingAction->setChecked(true);
    }
    else
    {
        graphWindow->setXAxisAutoScrolling(false);
        xAxisAutoScrollingAction->setChecked(false);
    }
}

void MainWindow::yAxisAutoScaling(void)
{
    if (yAxisAutoScalingAction->isChecked())
    {
        graphWindow->setYAxisAutoScaling(true);
        yAxisAutoScalingAction->setChecked(true);
    }
    else
    {
        graphWindow->setYAxisAutoScaling(false);
        yAxisAutoScalingAction->setChecked(false);
    }
}

void MainWindow::startFlashLog(void)
{
    currentLpms->getSensor()->startFlashLogging();
}

void MainWindow::stopFlashLog(void)
{
    currentLpms->getSensor()->stopFlashLogging();

}

void MainWindow::clearFlashLog(void)
{
    currentLpms->getSensor()->clearFlashLog();
}

void MainWindow::eraseFlash(void)
{
    currentLpms->getSensor()->fullEraseFlash();
}

void MainWindow::saveFlashLog(void)
{
    if (currentLpms == 0 || isConnecting == true) {
        return;
    }

    if (currentLpms->getSensor()->isDownloadingFlashLog())
        return;

    QString qFilename = QFileDialog::getSaveFileName(this, "Save sensor data", "./", "*.csv");
    string recordFilename = qFilename.toStdString();
    if (recordFilename == "")
        return;

    if (!qFilename.endsWith(".csv"))
        recordFilename += ".csv";

    currentLpms->getSensor()->getFlashLog(recordFilename.c_str());
    
    uploadProgress = new QProgressDialog("Saving sensor flash data, please don't turn off your device...", "Cancel", 0, 200, this);
    uploadProgress->setWindowFlags(uploadProgress->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    uploadProgress->setWindowModality(Qt::WindowModal);
    uploadProgress->setMinimumWidth(400);
    uploadProgress->setAutoReset(false);
    uploadProgress->setCancelButton(0);
    uploadProgress->show();

    uploadTimer = new QTimer(this);
    connect(uploadTimer, SIGNAL(timeout()), this, SLOT(downloadFlashLogTimerUpdate()));
    connect(uploadProgress, SIGNAL(canceled()), this, SLOT(cancelDownloadFlashLog()));
    uploadTimer->start(500);
}

void MainWindow::downloadFlashLogTimerUpdate(void)
{
    int p;

    if (currentLpms->getSensor()->getDownloadFlashLogProgress(&p) == 1) {
        uploadProgress->setValue(p);
        uploadProgress->show();
    }
    else {
        QMessageBox msgBox;

        delete uploadTimer;
        delete uploadProgress;

        msgBox.setText("Flash log download finished.");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWindow::cancelDownloadFlashLog()
{
    currentLpms->getSensor()->cancelGetFlashLog();
    QMessageBox msgBox;

    delete uploadTimer;
    delete uploadProgress;

    msgBox.setText("Flash log download canceled.");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();

}

void MainWindow::enableFlashMenu()
{
    startFlashLogAction->setEnabled(true);
    stopFlashLogAction->setEnabled(true);
    clearFlashLogAction->setEnabled(true);
    eraseFlashAction->setEnabled(true);
    saveFlashLogAction->setEnabled(true);
}

void MainWindow::disableFlashMenu()
{
    startFlashLogAction->setEnabled(false);
    stopFlashLogAction->setEnabled(false);
    clearFlashLogAction->setEnabled(false);
    eraseFlashAction->setEnabled(false);
    saveFlashLogAction->setEnabled(false);
}