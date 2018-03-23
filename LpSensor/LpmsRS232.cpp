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

#include "LpmsRS232.h"

#define GYRO1X_START 2
#define ACC_START 8
#define MAG_START 14

#define RAW_DATA_LENGTH 20

#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7

#define LPMS_GOTO_BROADCAST	0
#define LPMS_GOTO_SLEEP 1
#define LPMS_RAW_DATA 61

#define LPMS_FACTORY_IMU_ID 1

LpmsRS232::LpmsRS232(CalibrationData *configData) :
TAG("LpmsRS232"),
rs232Handle(NULL),
LpmsIoInterface(configData)
{
    currentUartBaudrate = SELECT_LPMS_UART_BAUDRATE_115200;
}

long long LpmsRS232::getConnectWait(void) {
    return 1000000;
}

void LpmsRS232::listDevices(LpmsDeviceList *deviceList)
{
    int i;
    HANDLE h;

    for (i = 0; i < 257; ++i) {
        std::ostringstream pS;
        pS << "\\\\.\\COM" << i;
        h = CreateFile(pS.str().c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (h != INVALID_HANDLE_VALUE) {
            std::ostringstream pS;
            pS << "COM" << i;
            deviceList->push_back(DeviceListItem(pS.str().c_str(), DEVICE_LPMS_RS232));
            CloseHandle(h);
        }
    }
}

bool LpmsRS232::connect(string deviceId)
{
    std::ostringstream pS;
    pS << "\\\\.\\" << deviceId;

    this->idNumber = deviceId;

    isOpen = false;

    rs232Handle = CreateFile(pS.str().c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (GetCommState(rs232Handle, &rs232Config) == 0) {
        return false;
    }

    switch (currentUartBaudrate) {
    case SELECT_LPMS_UART_BAUDRATE_19200:
        rs232Config.BaudRate = 19200;
        break;

    case SELECT_LPMS_UART_BAUDRATE_38400:
        rs232Config.BaudRate = 38400;
        break;

    case SELECT_LPMS_UART_BAUDRATE_57600:
        rs232Config.BaudRate = 57600;
        break;

    case SELECT_LPMS_UART_BAUDRATE_115200:
        rs232Config.BaudRate = 115200;
        break;

    case SELECT_LPMS_UART_BAUDRATE_230400:
        rs232Config.BaudRate = 230400;
        break;

    case SELECT_LPMS_UART_BAUDRATE_256000:
        rs232Config.BaudRate = 256000;
        break;

    case SELECT_LPMS_UART_BAUDRATE_460800:
        rs232Config.BaudRate = 460800;
        break;

    case SELECT_LPMS_UART_BAUDRATE_921600:
        rs232Config.BaudRate = 921600;
        break;

    default:
        rs232Config.BaudRate = 115200;
        break;
    }
    logd(TAG, "baudrate: %d\n", rs232Config.BaudRate);
    rs232Config.StopBits = ONESTOPBIT;
    rs232Config.Parity = NOPARITY;
    rs232Config.ByteSize = 8;
    rs232Config.fInX = FALSE;
    rs232Config.fOutX = FALSE;
    rs232Config.fOutxDsrFlow = FALSE;
    rs232Config.fOutxCtsFlow = FALSE;
    rs232Config.fDtrControl = DTR_CONTROL_ENABLE;
    rs232Config.fRtsControl = RTS_CONTROL_ENABLE;
    /*
    printf("fInX: %d\n", rs232Config.fInX);
    printf("fOutX: %d\n", rs232Config.fOutX);
    printf("fOutxCtsFlow: %d\n", rs232Config.fOutxCtsFlow);
    printf("fOutxDsrFlow: %d\n", rs232Config.fOutxDsrFlow);
    printf("fDtrControl: %d\n", rs232Config.fDtrControl);
    printf("fRtsControl: %d\n", rs232Config.fRtsControl);
    */

    
    if (SetCommState(rs232Handle, &rs232Config) == 0) {
        return false;
    }

    COMMTIMEOUTS comTimeOut;
    comTimeOut.ReadIntervalTimeout = 1;
    comTimeOut.ReadTotalTimeoutMultiplier = 1;
    comTimeOut.ReadTotalTimeoutConstant = 1;
    comTimeOut.WriteTotalTimeoutMultiplier = 1;
    comTimeOut.WriteTotalTimeoutConstant = 1;

    SetCommTimeouts(rs232Handle, &comTimeOut);
    

    oneTx.clear();

    rxState = PACKET_END;
    currentState = GET_CONFIG;

    waitForAck = false;
    waitForData = false;

    ackReceived = false;
    ackTimeout = 0;

    lpmsStatus = 0;
    configReg = 0;

    imuId = 1;
    dataReceived = false;
    dataTimeout = 0;

    pCount = 0;

    isOpen = true;

    return true;
}

void LpmsRS232::close(void) {
    if (isOpen == false) return;

    isOpen = false;
    if (rs232Handle != NULL)
        CloseHandle(rs232Handle);
    rs232Handle = NULL;
    return;
}

bool LpmsRS232::write(unsigned char *txBuffer, unsigned bufferLength)
{
    unsigned long l;

    if (WriteFile(rs232Handle, txBuffer, bufferLength, &l, NULL) == 0) {
        if (verbose) logd(TAG, "Writing serial port failed\n");
        return false;
    }

    return true;
}

bool LpmsRS232::read(unsigned char *rxBuffer, unsigned long *bytesReceived) {
    if (ReadFile(rs232Handle, rxBuffer, 4, bytesReceived, NULL) == 0) {
        if (verbose) logd(TAG, "Reading serial port failed\n");
        return false;
    }

    return true;
}

bool LpmsRS232::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
    unsigned char txData[1024];
    unsigned int txLrcCheck;
    // int i;

    if (length > 1014) return false;

    txData[0] = 0x3a;
    txData[1] = address & 0xff;
    txData[2] = (address >> 8) & 0xff;
    txData[3] = function & 0xff;
    txData[4] = (function >> 8) & 0xff;
    txData[5] = length & 0xff;
    txData[6] = (length >> 8) & 0xff;

    for (unsigned int i = 0; i < length; ++i) {
        txData[7 + i] = data[i];
    }

    txLrcCheck = address;
    txLrcCheck += function;
    txLrcCheck += length;

    for (unsigned int i = 0; i < length; i++) {
        txLrcCheck += data[i];
    }

    txData[7 + length] = txLrcCheck & 0xff;
    txData[8 + length] = (txLrcCheck >> 8) & 0xff;
    txData[9 + length] = 0x0d;
    txData[10 + length] = 0x0a;

    if (write(txData, length + 11) == true) {
        // for (i=0; i<length+11; ++i) printf("%x ", txData[i]);
        return true;
    }

    return false;
}


bool LpmsRS232::parseModbusByte(void)
{
    unsigned char b;

    while (dataQueue.size() > 0) {
        b = dataQueue.front();
        dataQueue.pop();

        switch (rxState) {
        case PACKET_END:
            if (b == 0x3a) {
                rxState = PACKET_ADDRESS0;
                oneTx.clear();
            }
            break;

        case PACKET_ADDRESS0:
            currentAddress = b;
            rxState = PACKET_ADDRESS1;
            break;

        case PACKET_ADDRESS1:
            currentAddress = currentAddress + ((unsigned)b * 256);
            rxState = PACKET_FUNCTION0;
            break;

        case PACKET_FUNCTION0:
            currentFunction = b;
            rxState = PACKET_FUNCTION1;
            break;

        case PACKET_FUNCTION1:
            currentFunction = currentFunction + ((unsigned)b * 256);
            rxState = PACKET_LENGTH0;
            break;

        case PACKET_LENGTH0:
            currentLength = b;
            rxState = PACKET_LENGTH1;
            break;

        case PACKET_LENGTH1:
            currentLength = currentLength + ((unsigned)b * 256);
            rxState = PACKET_RAW_DATA;
            rawDataIndex = currentLength;
            break;

        case PACKET_RAW_DATA:
            if (rawDataIndex == 0) {
                lrcCheck = currentAddress + currentFunction + currentLength;
                for (unsigned i = 0; i < oneTx.size(); i++) {
                    lrcCheck += oneTx[i];
                }

                lrcReceived = b;
                rxState = PACKET_LRC_CHECK1;
            }
            else {
                oneTx.push_back(b);
                --rawDataIndex;
            }
            break;

        case PACKET_LRC_CHECK1:
            lrcReceived = lrcReceived + ((unsigned)b * 256);

            if (lrcReceived == lrcCheck) {
                parseFunction();
            }
            else {
                if (verbose) logd(TAG, "Checksum fail in data packet\n");
            }

            rxState = PACKET_END;
            break;

        default:
            rxState = PACKET_END;
            //return false;
            break;
        }
    }

    return true;
}

bool LpmsRS232::pollData(void)
{
    unsigned long bytesReceived;
    unsigned char rxBuffer[4096];
    bool packetOk = false;

    if (deviceStarted() == false) return false;

    if (read(rxBuffer, &bytesReceived) == false) {
        isOpen = false;
        return false;
    }

    if (bytesReceived > 1024)
    {
        if (verbose) logd(TAG, "rxBuffer overflow!\n");
    }
    for (unsigned int i = 0; i < bytesReceived; i++) {
        dataQueue.push((unsigned char)rxBuffer[i]);
    }

    return true;
}

bool LpmsRS232::deviceStarted(void)
{
    return isOpen;
}

void LpmsRS232::setRs232Baudrate(int i)
{
    currentUartBaudrate = i;
}