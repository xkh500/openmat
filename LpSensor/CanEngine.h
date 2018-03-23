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

#ifndef CAN_ENGINE
#define CAN_ENGINE

#include <iostream>
#include <string>
#include <queue>
#include <list>
#include <thread>

#ifdef _WIN32
#include "windows.h"

#include "PCANBasic.h"
#include "PCANBasicClass.h"

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <time.h>
#include <string.h>

#include <winsock.h>
#endif
#include "util.h"
#include "LpmsCanIo.h"
#include "DeviceListItem.h"
#include "LpmsDefinitions.h"

#define CMD_LEN 2048
#define CAN_GATEWAY_TCP_PORT 19227

class CanEngine {
    const std::string TAG;
private:
#ifdef _WIN32
    std::queue<TPCANMsg> txQ;
    TPCANHandle canChannel;
    PCANBasicClass pcan;

    UINT32 port;
    SOCKET connectionSocket;
    WORD canBaudrate;
    char commandStr[CMD_LEN], replyStr[CMD_LEN];
#endif

    std::list<LpmsCanIo *> sensorList;
    std::list<LpmsCanIo *> sensorScanList;
    vector<int> activeCanId;

    bool peakCanInitialized;
    bool ixxatCanInitialized;

    bool peakCanDetected;
    bool ixxatCanDetected;

    int messageBytes;
    int headerBytes;
    int messageLen;
    int ixxatState;

    char headerData[64];
    char messageData[64];
    bool verbose;
public:
    /*! Constructor. */
    CanEngine(void);

    /*! Destructor. */
    ~CanEngine(void);

    /*! Connect to CAN interface. */
    void connect(void);

    /*! Add one sensor to the list of sensors that are maintained by the CAN engine. */
    void addSensor(LpmsCanIo *s);

    /*! Poll data from CAN bus. */
    void poll(void);

    /*! Close connection to CAN bus interface. */
    void close(void);

    /*! Poll data from transmission queue. */
    bool updateSendQueue(void);

    /*! List CAN devices connected to the system. */
    void listDevices(LpmsDeviceList *v);

    static void stopDiscovery(void);

    /*! Remove sensor from list of connected sensors. */
    void removeSensor(LpmsCanIo *s);

    bool isInterfacePresent(void);

    void setBaudrate(int i);

    void setVerbose(bool v);

    /* IXXAT related */
    bool sendCmdIxxat(char *commandPtr);
    bool getReplyIxxat(char *replyPtr, int *nBytes);
    bool connectIxxat(char *hostName, int port);
    bool initIxxat(void);

    void setSensorToCommandMode(const char* id);
private:
    void ping(int id);

    void pingCanIds();
    int getActiveDeviceId();
    void dummyRead();

    bool quitDummyReadThread;

};

#endif
