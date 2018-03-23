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

#include "LpmsU2.h"

#include "SiUSBXp.h"

#define GYRO1X_START 2
#define ACC_START 8
#define MAG_START 14

#define RAW_DATA_LENGTH 20
/*
#define PACKET_ADDRESS0 0
#define PACKET_ADDRESS1 1
#define PACKET_FUNCTION0 2
#define PACKET_FUNCTION1 3
#define PACKET_RAW_DATA 4
#define PACKET_LRC_CHECK0 5
#define PACKET_LRC_CHECK1 6
#define PACKET_END 7
*/

#define LPMS_GOTO_BROADCAST 0
#define LPMS_GOTO_SLEEP 1
#define LPMS_RAW_DATA 61

#define LPMS_FACTORY_IMU_ID 1

LpmsU2::LpmsU2(CalibrationData *configData) :
    TAG("LpmsU2"),
    LpmsIoInterface(configData)
{
    isOpen = false;
}

long long LpmsU2::getConnectWait(void) {
    return 3000000;
}

void LpmsU2::listDevices(LpmsDeviceList *deviceList)
{
    SI_STATUS siStatus;
    SI_DEVICE_STRING	DeviceString;
    //	FT_DEVICE_LIST_INFO_NODE *devInfo;

#ifdef _WIN32
    DWORD numDevs;
#else
    DWORD numDevs;
#endif

    siStatus = SI_GetNumDevices(&numDevs);
    if (siStatus == SI_SUCCESS) {
        logd("LpmsU2", "Number of devices is %d\n", numDevs);

        for (DWORD d = 0; d < numDevs; d++)
        {
            siStatus = SI_GetProductString(d, DeviceString, SI_RETURN_SERIAL_NUMBER);
            if (siStatus == SI_SUCCESS)
            {
                string deviceString = string(DeviceString);
                transform(deviceString.begin(), deviceString.end(), deviceString.begin(), ::toupper);
                logd("LpmsU2", "Discovered device: %s\n", deviceString.c_str());
                deviceList->push_back(DeviceListItem(deviceString.c_str(), DEVICE_LPMS_U2));
            }
        }
    }


}

bool LpmsU2::connect(string deviceId)
{
    SI_STATUS siStatus;
    unsigned long bytesReceived;
    unsigned char rxBuffer[4096];
    DWORD devercNum = 0;
    SI_DEVICE_STRING	DeviceString;

    isOpen = false;
    transform(deviceId.begin(), deviceId.end(), deviceId.begin(), ::toupper);
    this->idNumber = deviceId;

    DWORD numDevs;
    if (SI_GetNumDevices(&numDevs) != SI_SUCCESS)
        return false;
    devercNum = numDevs;
    for (DWORD d = 0; d < numDevs; d++)
    {
        siStatus = SI_GetProductString(d, DeviceString, SI_RETURN_SERIAL_NUMBER);
        if (siStatus == SI_SUCCESS)
        {
            string deviceString = string(DeviceString);
            transform(deviceString.begin(), deviceString.end(), deviceString.begin(), ::toupper);

            if (deviceId == deviceString)
            {
                devercNum = d;
                break;
            }
        }
    }
    if (devercNum == numDevs)
    {
        if (verbose) logd(TAG, "Sensor %s not found\n", deviceId.c_str());
        return false;
    }

    siStatus = SI_Open(devercNum, &cyHandle);
    siStatus = SI_SetBaudRate(cyHandle, 921600);
    siStatus = SI_SetFlowControl(cyHandle, SI_HANDSHAKE_LINE, SI_FIRMWARE_CONTROLLED, SI_HELD_INACTIVE, SI_STATUS_INPUT, SI_STATUS_INPUT, 0);
    
    if (siStatus == SI_SUCCESS) {
        setCommandMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (verbose) logd(TAG, "Connection to %s successful\n", idNumber.c_str());
        isOpen = true;

       // read(rxBuffer, &bytesReceived);
       SI_FlushBuffers(cyHandle, TRUE, TRUE);
    }
    else {
        if (verbose) logd(TAG, "Connection to %s failed\n", idNumber.c_str());
        isOpen = false;
        return false;
    }

    oneTx.clear();

    rxState = PACKET_END;
    currentState = GET_CONFIG;

    waitForAck = false;
    ackReceived = false;
    waitForData = false;
    dataReceived = false;

    pCount = 0;
    ackTimeout = 0;
    dataTimeout = 0;

    lpmsStatus = 0;
    configReg = 0;

    timestampOffset = 0.0;
    currentTimestamp = 0.0;

    return isOpen;
}

void LpmsU2::close(void)
{
    if (isOpen == false) return;

    stopStreaming();
    isOpen = false;
    SI_Close(cyHandle);

    logd(TAG, "Connection to %s closed.\n", idNumber.c_str());
}

bool LpmsU2::read(unsigned char *rxBuffer, unsigned long *bytesReceived)
{
    SI_STATUS siStatus;
    PBYTE ModemStatus;

#ifdef _WIN32
    unsigned long eventDWord;
    unsigned long txBytes;
    unsigned long rxBytes;
    unsigned long qStatus;
#else
    unsigned int eventDWord;
    unsigned int txBytes;
    unsigned int rxBytes;
#endif

    bool f = true;

    *bytesReceived = 0;

    if (isOpen == false) return false;

    siStatus = SI_CheckRXQueue(cyHandle, &rxBytes, &qStatus);

    if (siStatus != SI_SUCCESS)
    {
        if (verbose) logd(TAG, "Read failed!\n");
        return false;
    }
    if (rxBytes == 0)
        return true;

    if (rxBytes > 4096) {
        if (verbose) logd(TAG, "Buffer overflow!\n");
        rxBytes = 4096;
    }

#ifdef _WIN32
    siStatus = SI_Read(cyHandle, rxBuffer, rxBytes, bytesReceived);
#else
    siStatus = SI_Read(cyHandle, rxBuffer, rxBytes, (unsigned int *)bytesReceived);
#endif

    if (siStatus == SI_SUCCESS) {
        f = true;
    }
    else {
        f = false;
        if (verbose) logd(TAG, "Read failed timeout!\n");
    }

    return f;
}

bool LpmsU2::write(unsigned char *txBuffer, unsigned bufferLength)
{
    SI_STATUS siStatus;

#ifdef _WIN32
    unsigned long bytesWritten;
#else
    unsigned int bytesWritten;
#endif

    bool f = false;

    if (isOpen == false) return false;

    siStatus = SI_Write(cyHandle, txBuffer, bufferLength, &bytesWritten);

    if (siStatus == SI_SUCCESS) {
        f = true;
    }

    return f;
}

bool LpmsU2::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
    unsigned char txData[1024];
    unsigned int txLrcCheck;

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
        return true;
    }

    return false;
}

bool LpmsU2::parseModbusByte(void)
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
            return false;
            break;
        }
    }

    return true;
}

bool LpmsU2::pollData(void)
{
    unsigned long bytesReceived;
    unsigned char rxBuffer[4096];
    bool packetOk = false;

    if (deviceStarted() == false) return false;

    if (read(rxBuffer, &bytesReceived) == false)
    {
        //isOpen = false;
        return false;
    }

    if (bytesReceived > 0) {
        for (unsigned int i = 0; i < bytesReceived; i++) {
            dataQueue.push((unsigned char)rxBuffer[i]);
        }
    }

    return true;
}

bool LpmsU2::deviceStarted(void)
{
    return isOpen;
}
