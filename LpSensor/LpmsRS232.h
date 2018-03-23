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

#ifndef LPMS_RS232
#define LPMS_RS232

#include <iostream>
#include <string>
#include <queue>
#include <vector>
using namespace std;

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"

#ifdef _WIN32
#include "windows.h"

#else
#include <mutex>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif


/*! \brief	Hardware driver class for LPMS-U sensor. */
class LpmsRS232 : public LpmsIoInterface {
    const std::string TAG;
public:
    /* Constructor. */
    LpmsRS232(CalibrationData *configData);

    /* Lists devices connected to the system. */
    static void listDevices(LpmsDeviceList *deviceList);

    /* Connects to LPMS. */
    bool connect(string deviceId);

    /* Closes sensor connection. */
    void close(void);

    /* Polls data from sensor. */
    bool pollData(void);

    /* Returns true if device has been successfully connected. */
    bool deviceStarted(void);

    /* Parses one byte of a LPBUS message. */
    bool parseModbusByte(void);

    void setRs232Baudrate(int i);

private:
    bool read(unsigned char *rxBuffer, unsigned long *bytesReceived);
    bool write(unsigned char *txBuffer, unsigned bufferLength);
    bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
    long long getConnectWait(void);

#ifdef _WIN32
    HANDLE rs232Handle;
    DCB rs232Config;
#else
    int fd;

    void set_blocking (int fd, int should_block);

    int set_interface_attribs (int fd, int speed, int parity);
    std::mutex mutexDataQueue;
#endif



    string portname;
    bool isOpen;
    string idNumber;
    int currentUartBaudrate;
};

#endif
