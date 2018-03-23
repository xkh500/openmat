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

#include <QCloseEvent>
#include <QFileDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QMenuBar>
#include <QPushButton>
#include <QSplitter>
#include <QToolBar>

#include "LpemgSensorManagerI.h"
#include "LpemgSensorI.h"
#include "LpmsDefinitions.h"

using namespace std;

static LpmsDeviceList deviceList;

MainWindow::MainWindow(QWidget *parent)
{
    std::cout << "[MainWindow] Initializing program LpemgControl" << std::endl;

    sm = LpemgSensorManagerFactory();
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

    setWindowTitle("LPEMMG Control-V"+ QString(LPMS_CONTROL_VERSION) + " GUI");

    isRunning = false;
    isConnecting = false;

    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
    timer->start(1);

    textUpdateCounter = 0;
    mode = MODE_GRAPH_WIN;

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
    currentLpemg = 0;
    lpmsTree->selectionModel()->setCurrentIndex(QModelIndex(), QItemSelectionModel::Clear);
    lpmsTree->setVerticalScrollMode(QTreeView::ScrollPerPixel);

    connect(lpmsTree, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(updateCurrentLpemg(QTreeWidgetItem *, QTreeWidgetItem *)));

    return (QWidget*)lpmsTree;
}

QGroupBox *MainWindow::createGraphs(void)
{
    graphLayout = new QVBoxLayout();
    graphWindow = new GraphWindow();

    graphLayout->addWidget(graphWindow);

    gb = new QGroupBox("Data view");
    gb->setLayout(graphLayout);

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

    // toolbar->addSeparator();

    // startFlashLogAction = new QAction("Start Flash Log", this);
    // stopFlashLogAction = new QAction("Stop Flash Log", this);
    // clearFlashLogAction = new QAction("Clear Flash Log", this);
    // eraseFlashAction = new QAction("Full Flash Erase", this);
    // saveFlashLogAction = new QAction("Save Flash Log", this);

    // measurementMenu->addSeparator();
    // measurementMenu->addAction(startFlashLogAction);
    // measurementMenu->addAction(stopFlashLogAction);
    // measurementMenu->addAction(clearFlashLogAction);
    // measurementMenu->addAction(eraseFlashAction);
    // measurementMenu->addAction(saveFlashLogAction);


    // toolbar->addSeparator();

    // QVBoxLayout *v2 = new QVBoxLayout();
    // targetCombo = new QComboBox();
    // targetCombo->addItem("All sensors");
    // targetCombo->addItem("Selected sensor");
    // v2->addWidget(new QLabel("Reset target:"));
    // v2->addWidget(targetCombo);
    // QWidget *w2 = new QWidget();
    // w2->setLayout(v2);
    // w2->setFixedWidth(150);
    // toolbar->addWidget(w2);

    // QVBoxLayout *v5 = new QVBoxLayout();
    // resetMethodCombo = new QComboBox();
    // resetMethodCombo->addItem("Object reset");
    // resetMethodCombo->addItem("Heading reset");
    // resetMethodCombo->addItem("Alignment reset");
    // v5->addWidget(new QLabel("Reset method:"));
    // v5->addWidget(resetMethodCombo);
    // QWidget *w5 = new QWidget();
    // w5->setLayout(v5);
    // w5->setFixedWidth(150);
    // toolbar->addWidget(w5);

    // QAction* setOffsetAction = new QAction(QIcon("./icons/fullscreen_exit_32x32.png"), "Set offset", this);
    // toolbar->addAction(setOffsetAction);
    // QAction* resetOffsetAction = new QAction(QIcon("./icons/denied_32x32.png"), "Reset offset", this);
    // toolbar->addAction(resetOffsetAction);

    QMenu* calibrationMenu = menuBar()->addMenu("&Calibration");

    // QAction* gyroAction = new QAction("Calibrate &gyroscope", this);
    // QAction* resetSingleRefAction = new QAction("Reset &heading (selected)", this);
    // QAction* resetAllRefAction = new QAction("Reset heading (&all)", this);
    // QAction* resetSingleOrientationAction = new QAction("Reset &offset (selected)", this);
    // QAction* resetAllOrientationAction = new QAction("Reset o&ffset (all)", this);
    QAction* saveCalAction = new QAction("Save &parameters to sensor", this);
    QAction* resetToFactoryAction = new QAction("Reset to factory settings", this);
    // QAction* saveToFileAction = new QAction("Save calibration file", this);
    // QAction* loadFromFileAction = new QAction("Load calibration file", this);
    // QAction* timestampResetAction = new QAction("Reset Timestamp (All sensors)", this);
    // QAction* softwareSyncStartAction = new QAction("Software Sync", this);
    // QAction* softwareSyncStopAction = new QAction("Software Sync Stop", this);

    // calibrationMenu->addAction(gyroAction);
    // calibrationMenu->addSeparator();
    calibrationMenu->addAction(saveCalAction);
    // calibrationMenu->addAction(saveToFileAction);
    // calibrationMenu->addAction(loadFromFileAction);
    // calibrationMenu->addSeparator();
    // calibrationMenu->addAction(setOffsetAction);
    // calibrationMenu->addAction(resetOffsetAction);
    // calibrationMenu->addSeparator();
    // calibrationMenu->addAction(timestampResetAction);
    // calibrationMenu->addAction(softwareSyncStartAction);
    // calibrationMenu->addAction(softwareSyncStopAction);
    // calibrationMenu->addSeparator();
    calibrationMenu->addAction(resetToFactoryAction);

    viewMenu = menuBar()->addMenu("&View");
    QAction* graphAction = new QAction(QIcon("./icons/rss_alt_32x32.png"), "&EMMG graph window", this);
    QAction* orientationGraphAction = new QAction(QIcon("./icons/target_32x32.png"), "&Motion graph window", this);
    QAction* threedAction = new QAction(QIcon("./icons/share_32x32.png"), "3D &visualization", this);
    // QAction* fieldMapAction = new QAction(QIcon("./icons/sun_fill_32x32.png"), "&Magnetic field map", this);
    // QAction* loadObjFileAction = new QAction("&Load object file", this);
    xAxisAutoScrollingAction = new QAction("&X Axis Auto Scrolling", this);
    xAxisAutoScrollingAction->setCheckable(true);
    xAxisAutoScrollingAction->setChecked(false);
    yAxisAutoScalingAction = new QAction("&Y Axis Auto Scaling", this);
    yAxisAutoScalingAction->setCheckable(true);
    yAxisAutoScalingAction->setChecked(false);
    // cubeMode1Action = new QAction("3D view mode &1", this);
    // cubeMode1Action->setCheckable(true);
    // cubeMode1Action->setChecked(true);
    // cubeMode2Action = new QAction("3D view mode &2", this);
    // cubeMode2Action->setCheckable(true);
    // cubeMode4Action = new QAction("3D view mode &4", this);
    // cubeMode4Action->setCheckable(true);

    viewMenu->addAction(graphAction);
    viewMenu->addAction(orientationGraphAction);
    // viewMenu->addAction(pressureGraphAction);

    // viewMenu->addAction(threedAction);
    // viewMenu->addAction(fieldMapAction);
    // viewMenu->addSeparator();
    // viewMenu->addAction(cubeMode1Action);
    // viewMenu->addAction(cubeMode2Action);
    // viewMenu->addAction(cubeMode4Action);
    viewMenu->addSeparator();
    viewMenu->addAction(xAxisAutoScrollingAction);
    viewMenu->addAction(yAxisAutoScalingAction);
    // viewMenu->addSeparator();
    // viewMenu->addAction(loadObjFileAction);

    toolbar->addSeparator();

    QWidget *stretchWidget = new QWidget();
    stretchWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    toolbar->addWidget(stretchWidget);

    toolbar->addSeparator();
    toolbar->addAction(graphAction);
    toolbar->addAction(orientationGraphAction);
    // toolbar->addAction(pressureGraphAction);
    // toolbar->addAction(threedAction);
    // toolbar->addAction(fieldMapAction);

    QMenu* expertMenu = menuBar()->addMenu("&Advanced");
    QAction* firmwareAction = new QAction("Upload &firmware", this);
    QAction* iapAction = new QAction("Upload &IAP", this);
    // selfTestAction = new QAction("Start self test", this);
    QAction* latencyAction = new QAction("Measure &latency", this);
    // QAction* getFieldMapAction = new QAction("Get field map", this);
    QAction* versionAction = new QAction("&Version info", this);

    expertMenu->addAction(firmwareAction);
    expertMenu->addAction(iapAction);
    expertMenu->addSeparator();
    // expertMenu->addAction(selfTestAction);
    // expertMenu->addSeparator();
    expertMenu->addSeparator();
    expertMenu->addAction(versionAction);

    connect(startAction, SIGNAL(triggered()), this, SLOT(startMeasurement()));
    // connect(gyroAction, SIGNAL(triggered()), this, SLOT(recalibrate()));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(recordData()));
    connect(browseAction, SIGNAL(triggered()), this, SLOT(browseRecordFile()));
    connect(saveCalAction, SIGNAL(triggered()), this, SLOT(saveCalibration()));
    connect(connectAction, SIGNAL(triggered()), this, SLOT(openSensor()));
    connect(disconnectAction, SIGNAL(triggered()), this, SLOT(closeSensor()));
    connect(graphAction, SIGNAL(triggered()), this, SLOT(selectEmmgGraphWindow()));
    connect(orientationGraphAction, SIGNAL(triggered()), this, SLOT(selectMotionGraphWindow()));
    // connect(pressureGraphAction, SIGNAL(triggered()), this, SLOT(selectGraph3Window()));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));
    connect(firmwareAction, SIGNAL(triggered()), this, SLOT(uploadFirmware()));
    connect(iapAction, SIGNAL(triggered()), this, SLOT(uploadIap()));
    // connect(selfTestAction, SIGNAL(triggered()), this, SLOT(toggleSelfTest()));
    connect(latencyAction, SIGNAL(triggered()), this, SLOT(measureLatency()));
    connect(versionAction, SIGNAL(triggered()), this, SLOT(getVersionInfo()));
    connect(resetToFactoryAction, SIGNAL(triggered()), this, SLOT(resetToFactory()));
    // connect(loadFromFileAction, SIGNAL(triggered()), this, SLOT(loadCalibrationData()));
    // connect(saveToFileAction, SIGNAL(triggered()), this, SLOT(saveCalibrationData()));
    connect(addRemoveAction, SIGNAL(triggered()), this, SLOT(addRemoveDevices()));
    // connect(setOffsetAction, SIGNAL(triggered()), this, SLOT(setOffset()));
    // connect(resetOffsetAction, SIGNAL(triggered()), this, SLOT(resetOffset()));
    // connect(timestampResetAction, SIGNAL(triggered()), this, SLOT(timestampReset()));
    // connect(softwareSyncStartAction, SIGNAL(triggered()), this, SLOT(softwareSyncStart()));
    // connect(softwareSyncStopAction, SIGNAL(triggered()), this, SLOT(softwareSyncStop()));
    // connect(loadObjFileAction, SIGNAL(triggered()), this, SLOT(loadObjFile()));
    connect(xAxisAutoScrollingAction, SIGNAL(triggered()), this, SLOT(xAxisAutoScrolling()));
    connect(yAxisAutoScalingAction, SIGNAL(triggered()), this, SLOT(yAxisAutoScaling()));
    // connect(startFlashLogAction, SIGNAL(triggered()), this, SLOT(startFlashLog()));
    // connect(stopFlashLogAction, SIGNAL(triggered()), this, SLOT(stopFlashLog()));
    // connect(clearFlashLogAction, SIGNAL(triggered()), this, SLOT(clearFlashLog()));
    // connect(eraseFlashAction, SIGNAL(triggered()), this, SLOT(eraseFlash()));
    // connect(saveFlashLogAction, SIGNAL(triggered()), this, SLOT(saveFlashLog()));
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

void MainWindow::selectEmmgGraphWindow(void)
{
    graphWindow->setMode(GraphWindow::Mode::Emmg);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

void MainWindow::selectMotionGraphWindow(void)
{
    graphWindow->setMode(GraphWindow::Mode::Motion);
    graphWindow->show();

    mode = MODE_GRAPH_WIN;
}

bool MainWindow::exitWindow(void)
{
    QMessageBox msgBox;

    closeSensor();

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
    EmgData emgData;

   // if (mm.measure() < GRAPH_UPDATE_PERIOD) return;

    for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
        SensorGuiContainer* p = *it;
        if (p->getSensor()->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED
            && p->getSensor()->getSensorStatus() == SENSOR_STATUS_RUNNING) {
            p->setupSensorGuiContainer();
            p->updateData();
            p->checkOptionalFeatures();

            if (p == currentLpemg) {
                int nData = p->getSensor()->hasEmgData();
                for (int i = 0; i < nData; ++i) {
                    p->getSensor()->getEmgData(&emgData);
                    gb->setTitle("Data view (t: " + QString::number(emgData.timeStamp,'f', 1) + "s)");
                    switch (mode) {
                    case MODE_GRAPH_WIN:
                        graphWindow->plotDataSet(emgData);
                        break;
                    }
                }
            }
        }
    }
}

void MainWindow::updateCurrentLpemg(QTreeWidgetItem* current, QTreeWidgetItem* previous)
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

    for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
        (*it)->updateData();
		
        if (itemIndex == lpmsTree->indexOfTopLevelItem((*it)->treeItem)) {
            f = true;
            fit = it;
            fi = i;
        }
		
        ++i;
    }

    if (f == true) {
        currentLpemg = *fit;
        currentLpemg->openMatId = (*fit)->getSensor()->getOpenMatId();

        graphWindow->setActiveLpms((*fit)->getSensor()->getOpenMatId());

    } else {
        if (lpmsTree->topLevelItemCount() > 0) {
            currentLpemg = lpemgList.front();
        } else {
            currentLpemg = 0;
            startButton->setStyleSheet("QPushButton { color: black; }");
            startButton->setText("Start measurement");
            isRunning = false;
        }
    }
    graphWindow->clearGraphs();

    currentLpemg->getSensor()->getConfigurationPrm(PRM_DEVICE_TYPE, &i);
    if (i == DEVICE_LPMS_B2) {
        enableFlashMenu();
    } else {
        disableFlashMenu();
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
	
	if (mode != DEVICE_LPEMG_B) return;

    isConnecting = true;
    std::string deviceAddress = std::string(deviceList.getDeviceId(comboDeviceList->currentIndex()));


    f = false;
    std::list<SensorGuiContainer *>::iterator it;
    for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
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

    LpemgSensorI* lpemgDevice = sm->addSensor(mode, deviceAddress.c_str());

    currentLpemg = new SensorGuiContainer(lpemgDevice, lpmsTree);
    // mbcom.addSensor(lpmsDevice);

    // currentLpemg->updateData();
   
    lpemgList.push_back(currentLpemg);

    lpmsTree->insertTopLevelItem(0, currentLpemg->treeItem);
    lpmsTree->setCurrentItem(currentLpemg->treeItem);

    currentLpemg->treeItem->setExpanded(true);

#ifdef USE_CALLBACK
    lpmsDevice->setCallback(&lpmsCallback);
#endif

    isConnecting = false;
    startMeasurement();
}

void MainWindow::closeSensor(void)
{
    if (currentLpemg == 0 || isConnecting == true) return;

    stopMeasurement();

    if (currentLpemg) {
        SensorGuiContainer* temp = currentLpemg;

        // mbcom.removeSensor(temp->getSensor());

        sm->removeSensor(temp->getSensor());
        lpemgList.remove(temp);

        delete temp->treeItem;
        currentLpemg = 0;

        delete temp;
        updateCurrentLpemg();
    }
    startMeasurement();
    gb->setTitle("Data view");
}

void MainWindow::toggleSelfTest(void)
{
    int i;

    if (currentLpemg == 0 || isConnecting == true) return;

    currentLpemg->getSensor()->getConfigurationPrm(PRM_SELF_TEST, &i);

    if (i == SELECT_SELF_TEST_OFF) {
        selfTestAction->setText("Start self test");
        currentLpemg->getSensor()->setConfigurationPrm(PRM_SELF_TEST, SELECT_SELF_TEST_ON);
    }
    else {
        selfTestAction->setText("Stop self test");
        currentLpemg->getSensor()->setConfigurationPrm(PRM_SELF_TEST, SELECT_SELF_TEST_OFF);
    }
}

void MainWindow::startMeasurement(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (isConnecting == true) return;

    if (isRunning == false && lpemgList.size() > 0) {

        for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
            (*it)->getSensor()->run();
        }

        startAction->setText("Stop measurement");
        startAction->setIcon(QIcon("./icons/pause_24x32.png"));

        graphWindow->clearGraphs();

        isRunning = true;
    }
    else {
        for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
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

    for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
        (*it)->getSensor()->pause();
    }

    startAction->setText("Start measurement");
    startAction->setIcon(QIcon("./icons/play_24x32.png"));

    isRunning = false;
}

void MainWindow::setOffset(void)
{
}

void MainWindow::resetOffset(void)
{
}

void MainWindow::timestampReset(void)
{
    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpemg == 0 || isConnecting == true) return;

    for (it = lpemgList.begin(); it != lpemgList.end(); ++it) {
        (*it)->getSensor()->setTimestamp(0.0f);
    }
}

void MainWindow::softwareSyncStart(void)
{
}

void MainWindow::softwareSyncStop(void)
{
}

void MainWindow::getGyroStaticBias(void)
{
}

void MainWindow::recalibrate(void)
{
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

    if (currentLpemg == 0 || isConnecting == true) {
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

    currentLpemg->getSensor()->getConfigurationPrm(PRM_DEVICE_TYPE, &i);

	/* if (i == DEVICE_LPMS_B || i == DEVICE_LPMS_BLE) {
		if (!qfilename.contains("LpmsB") || !qfilename.contains("bin")) {
			printf("[MainWindow] LPMS-B invalid firmware filename.\n");
			f = false;
		}
	} else {
		if (!qfilename.contains("LpmsCU") || !qfilename.contains("bin")) {
			printf("[MainWindow] LPMS-CU Invalid firmware filename.\n");
			f = false;
		}
	} */

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
        currentLpemg->getSensor()->uploadFirmware(fn.c_str());
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

    if (currentLpemg->getSensor()->getUploadProgress(&p) == 1) {
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

    if (currentLpemg == 0 || isConnecting == true) {
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

	/* if (!qfilename.contains("LpmsCRS_IAP.bin")) {
		msgBox.setText("Invalid IAP file. Please confirm that you selected the right file for upload.");
		msgBox.exec();

		return;
	} */
	
    if (!(fn == "")) {
        currentLpemg->getSensor()->uploadIap(fn.c_str());
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
    if (currentLpemg == 0 || isConnecting == true) {
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
    } else {
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
    if (currentLpemg == 0 || isConnecting == true) return;

    currentLpemg->getSensor()->saveCalibrationData();

    startWaitBar(5);
}

void MainWindow::addRemoveDevices(void)
{
    stopMeasurement();

    rescanD->show();
}

void MainWindow::loadCalibrationData(void)
{
    if (currentLpemg == 0 || isConnecting == true) return;

    QString qfn = QFileDialog::getOpenFileName(this, "Load calibration data", "./", "");
    string fn = qfn.toStdString();

    if (!(fn == "")) {
        currentLpemg->getSensor()->loadCalibrationData(fn.c_str());
    }
}

void MainWindow::saveCalibrationData(void)
{
    char cStr[64];

    if (currentLpemg == 0 || isConnecting == true) return;

    currentLpemg->sensor->getConfigurationPrm(PRM_DEVICE_ID, cStr);
    string sensorName(cStr); 
    sensorName.erase(std::remove(sensorName.begin(), sensorName.end(), ':'), sensorName.end());
    QString qfn = QFileDialog::getSaveFileName(this, "Save calibration data", QString::fromStdString(sensorName)+".xml", "");
    string fn = qfn.toStdString();

    if (!(fn == "")) {
        currentLpemg->getSensor()->saveCalibrationData(fn.c_str());
    }
}

void MainWindow::updateLpmsFps(int v, int lpmsId)
{
    if (lpmsId == currentLpemg->getSensor()->getOpenMatId()) {
        imuFpsLabel->setText(QString("%1").arg(v));
    }
}

void MainWindow::measureLatency(void)
{
    if (currentLpemg == 0 || isConnecting == true) return;

    currentLpemg->getSensor()->measureAvgLatency();
}

void MainWindow::getVersionInfo(void)
{
    QString openMATVersion(LPMS_CONTROL_VERSION);
    QMessageBox::about(this, "LP-RESEARCH - LpmsControl", QString("LP-RESEARCH - LPEMMG Control ") 
        + openMATVersion 
        + QString("\n(c) LP-Research\nhttp://www.lp-research.com\n\nRelease information: https://bitbucket.org/lpresearch/openmat/wiki/Home"));
}

void MainWindow::resetToFactory(void)
{
    QMessageBox msgBox;
    int ret;

    std::list<SensorGuiContainer *>::iterator it;

    if (currentLpemg == 0 || isConnecting == true) {
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

    currentLpemg->getSensor()->resetToFactorySettings();
    // currentLpemg->getSensor()->hardReset();
}

void MainWindow::startWaitBar(int t)
{
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
    }
    else {
        calProgress->setValue(calTime);
    }
}

void MainWindow::loadObjFile(void)
{
    QString qFilename = QFileDialog::getOpenFileName(this, "Load 3D OBJ file", "./", "OBJ files (*.obj)");
    string objFilename = qFilename.toStdString();
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
}

void MainWindow::stopFlashLog(void)
{
}

void MainWindow::clearFlashLog(void)
{
}

void MainWindow::eraseFlash(void)
{
}

void MainWindow::saveFlashLog(void)
{
}

void MainWindow::downloadFlashLogTimerUpdate(void)
{
}

void MainWindow::cancelDownloadFlashLog()
{
}

void MainWindow::enableFlashMenu()
{
}

void MainWindow::disableFlashMenu()
{
}