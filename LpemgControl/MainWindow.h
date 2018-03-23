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

#ifndef LP_MAIN_WINDOW
#define LP_MAIN_WINDOW


#include <QWidget>
#include <QAction>
#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QMainWindow>
#include <QRadioButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "GraphWindow.h"
#include "LpemgSensorManagerI.h"
#include "MicroMeasure.h"
#include "RescanDialog.h"
#include "SensorGuiContainer.h"

#include <string>
#include <vector>
#include <list>

#define CALIBRATION_FILE "LpmsControlConfiguration.xml"

#define MODE_GRAPH_WIN 0
//#define MODE_THREED_WIN 1
//#define MODE_FIELDMAP_WIN 2
#define MODE_GAIT_TRACKING_WIN 3

#define LPMS_CONTROL_VERSION "1.3.5 (Build 20160728)"

#define GRAPH_UPDATE_PERIOD 2000

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    QWidget *createDeviceList(void);
    QGroupBox *createGraphs(void);
    void createMenuAndToolbar(void);
    void closeEvent(QCloseEvent *event);

    void selectEmmgGraphWindow(void);
    void selectMotionGraphWindow(void);
    bool exitWindow(void);

    void timerUpdate(void);

    void updateCurrentLpemg(QTreeWidgetItem* current = 0, QTreeWidgetItem* previous = 0);

    void openSensor(void);
    void closeSensor(void);
    void toggleSelfTest(void);
    void startMeasurement(void);
    void stopMeasurement(void);

    void setOffset(void);
    void resetOffset(void);
    void timestampReset(void);
    void softwareSyncStart(void);
    void softwareSyncStop(void);
    void getGyroStaticBias(void);
    void recalibrate(void);

    void uploadFirmware(void);
    void cancelUpload(void);
    void uploadTimerUpdate(void);
    void uploadIap(void);
    void recordData(void);
    void browseRecordFile(void);

    void saveCalibration(void);
    void addRemoveDevices(void);
    void saveCalibrationData(void);
    void loadCalibrationData(void);

    void updateLpmsFps(int v, int lpmsId);
    void measureLatency(void);
    void getVersionInfo(void);
    void resetToFactory(void);

    void startWaitBar(int t);
    void calTimerUpdate(void);

    void loadObjFile(void);
    void xAxisAutoScrolling(void);
    void yAxisAutoScaling(void);

    void startFlashLog(void);
    void stopFlashLog(void);
    void clearFlashLog(void);
    void eraseFlash(void);
    void saveFlashLog(void);
    void downloadFlashLogTimerUpdate(void);
    void cancelDownloadFlashLog();
    void enableFlashMenu();
    void disableFlashMenu();

private:
    QGroupBox* gb;
    QList<QTreeWidgetItem*> lpmsTreeItems;
    QTreeWidget* lpmsTree;
    GraphWindow* graphWindow;
    SensorGuiContainer* currentLpemg;
    std::list<SensorGuiContainer*> lpemgList;
    LpemgSensorManagerI* sm;
    QLabel *imuFpsLabel;
    QLabel *bluetoothStatusLabel;
    QLabel *lpmsStatusLabel;
    QLabel *xAccLabel;
    QLabel *yAccLabel;
    QLabel *zAccLabel;
    QLabel *xGyroLabel;
    QLabel *yGyroLabel;
    QLabel *zGyroLabel;
    QLabel *xMagLabel;
    QLabel *yMagLabel;
    QLabel *zMagLabel;
    QLabel *xAngleLabel;
    QLabel *yAngleLabel;
    QLabel *zAngleLabel;
    QLabel *xQuatLabel;
    QLabel *yQuatLabel;
    QLabel *zQuatLabel;
    QLabel *wQuatLabel;
    QLabel *frameCountLabel;
    QLabel *activeLpmsLabel;
    QLabel *magRangeLabel;
    QLabel *pressureLabel;
    QLabel *xLinAccLabel;
    QLabel *yLinAccLabel;
    QLabel *zLinAccLabel;
    QLineEdit* deviceAddressEdit;
    QLineEdit* ftdiDeviceEdit;
    QPushButton* recordDataButton;
    QPushButton* startButton;
    QPushButton* connectButton;
    QPushButton* calibrateMagButton;
    QPushButton* startServerButton;
    QPushButton* graphWindowButton;
    QPushButton* graphWindow2Button;
    QPushButton* graphWindow3Button;
    QPushButton* threeDWindowButton;
    QPushButton* fieldMapWindowButton;
    bool isRunning;
    bool isConnecting;
    QSlider* accGainSl;
    QSlider* accCovarSl;
    QLineEdit* csvFileEdit;
    QRadioButton* gyrOnlyBtn;
    QRadioButton* gyrAccBtn;
    QRadioButton* accMagBtn;
    QRadioButton* gyrAccMagNsBtn;
    QRadioButton* gyrAccMagSwBtn;
    QRadioButton* gTEnableBtn;
    QRadioButton* gTDisableBtn;
    QComboBox* deviceTypeSelector;
    QComboBox* addressSelector;
    QComboBox* recordingRateCombo;
    QAction* startAction;
    QAction* saveAction;
    QAction* selfTestAction;
    QAction* cubeMode1Action;
    QAction* cubeMode2Action;
    QAction* cubeMode4Action;
    QAction* xAxisAutoScrollingAction;
    QAction* yAxisAutoScalingAction;
    QAction* startFlashLogAction;
    QAction* stopFlashLogAction;
    QAction* clearFlashLogAction;
    QAction* eraseFlashAction;
    QAction* saveFlashLogAction;
    int textUpdateCounter;
    QTimer* uploadTimer;
    QProgressDialog* uploadProgress;
    int calMaxTime;
    int calTime;
    QProgressDialog* calProgress;
    QTimer* calTimer;
    RescanDialog* rescanD;
    int mode;
    QLayout* cubeLayout;
    QLayout* graphLayout;
    QComboBox* comboDeviceList;
    std::string globalRecordFile;
    QLineEdit* recordFileEdit;
    QComboBox* targetCombo;
    QComboBox* resetMethodCombo;
    bool recordFileSet;
    MicroMeasure mm;
    QToolBar* toolbar;
    QMenu* viewMenu;
    int softwareSyncCount;
};

#endif
