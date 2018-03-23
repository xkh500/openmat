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

#ifndef LPMS_U
#define LPMS_U

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
#endif

#include "ftd2xx.h"

class LpmsU : public LpmsIoInterface {
    const std::string TAG;
public:
    LpmsU(CalibrationData *configData);
    ~LpmsU();
    static void listDevices(LpmsDeviceList *deviceList);
    bool connect(string deviceId);
    void close(void);
    bool pollData(void);
    bool deviceStarted(void);
    bool parseModbusByte(void);

private:
    //#ifdef WIN32
    bool read(unsigned char *rxBuffer, unsigned long *bytesReceived);
    bool write(unsigned char *txBuffer, unsigned bufferLength);
    bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
    //#endif

    /*#ifdef __GNUC__
        bool read(unsigned char *rxBuffer, unsigned int *bytesReceived);
        bool write(unsigned char *txBuffer, unsigned bufferLength);
        bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
        #endif*/

    long long getConnectWait(void);

    FT_HANDLE ftHandle;
    bool isOpen;
    string idNumber;
};

#endif
