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

#include "LpmsCanIo.h"

LpmsCanIo::LpmsCanIo(CalibrationData *configData) :
TAG("LpmsCanIo"),
LpmsIoInterface(configData)
{
    _imuId = -1;
}

LpmsCanIo::~LpmsCanIo(void)
{
    close();
}

bool LpmsCanIo::connect(std::string deviceId)
{
    LpmsIoInterface::connect(deviceId);

    prevInMC = 255;
    outMC = 0;

    rxState = PACKET_END;

    try {
        imuId = boost::lexical_cast<int>(deviceId);
    }
    catch (boost::bad_lexical_cast const&) {
        imuId = 1;
    }

    //setCommandMode();
    stringstream ss;
    ss << deviceId;
    int id;
    ss >> id;
    sendModbusData(id, GOTO_COMMAND_MODE, 0, NULL);
    return true;
}

bool LpmsCanIo::sendModbusData(unsigned address, unsigned function,
    unsigned length, unsigned char *data)
{
    TPCANMsg sendMsg;
    unsigned char txData[1024];
    unsigned int txLrcCheck;
    int v = 0;

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

    sendMsg.ID = AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + imuId;
    sendMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;

    int p = (length + 11) / 8;
    int r = (length + 11) % 8;

    sendMsg.LEN = 8;
    for (int i = 0; i < p; i++) {
        for (int j = 0; j < 8; j++) {
            sendMsg.DATA[j] = txData[i * 8 + j];
        }

        /* printf("Msg. push %x %x %x %x %x %x %x %x\n", sendMsg.DATA[0], sendMsg.DATA[1], sendMsg.DATA[2], sendMsg.DATA[3], sendMsg.DATA[4], sendMsg.DATA[5], sendMsg.DATA[6], sendMsg.DATA[7]); */

        txQ.push(sendMsg);
    }

    if (r > 0) {
        sendMsg.LEN = r;
        for (int j = 0; j < r; j++) {
            sendMsg.DATA[j] = txData[p * 8 + j];
        }

        /* printf("Msg. push %x %x %x %x %x %x %x %x\n", sendMsg.DATA[0], sendMsg.DATA[1], sendMsg.DATA[2], sendMsg.DATA[3], sendMsg.DATA[4], sendMsg.DATA[5], sendMsg.DATA[6], sendMsg.DATA[7]); */

        txQ.push(sendMsg);
    }

    return true;
}

bool LpmsCanIo::getTxMessage(std::queue<TPCANMsg> *topTxQ)
{
    int i = txQ.size();
    if (i <= 0) return false;

    topTxQ->push(txQ.front());
    txQ.pop();

    return true;
}

bool LpmsCanIo::parseCanMsg(TPCANMsg m)
{
    const float r2d = 57.2958f;
    //int v = 0;
    int l = m.LEN;

    //configData->getParameter(PRM_OPENMAT_ID, &v);
    /*
    if (m.ID != AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + imuId) {
        return false;
    }
    */
    // HACK:
    // CAN id only valid for 1-127 range.
    // update imuId only if id is correctly updated in sensor
    if (m.ID == AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + _imuId) {
        imuId = _imuId;
        _imuId = -1;
    }
    if (m.ID != AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION + imuId) {
        return false;
    }

    for (int i = 0; i < m.LEN; i++) {
        dataQueue.push((unsigned char)m.DATA[i]);
    }

    return true;
}

bool LpmsCanIo::parseModbusByte(void)
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
                // cout << "[LpmsCanIo] Finished processing packet: " << currentFunction << endl;
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