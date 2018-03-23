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

#include "CanEngine.h"

#define MAX_RX_BYTES 4096
static volatile bool isStopDiscovery = false;

CanEngine::CanEngine(void):
    TAG("CanEngine")
{
    verbose = true;
    peakCanInitialized = false;
    peakCanDetected = false;

    ixxatCanDetected = false;
    ixxatCanInitialized = false;

    messageBytes = 0;
    headerBytes = 0;
    messageLen = 0;
    ixxatState = 0;
    canBaudrate = PCAN_BAUD_125K;
    // initIxxat();
}

CanEngine::~CanEngine(void)
{
    close();
}

void CanEngine::listDevices(LpmsDeviceList *v)
{
    if (verbose) logd(TAG, "ListDevice\n");
    if (peakCanInitialized == false && ixxatCanInitialized == false) return;
    pcan.Reset(canChannel);
    activeCanId.clear();
    pingCanIds();
    /*
    pcan.Reset(canChannel);
    activeCanId.clear();
    pingCanIds();
    */

    //quitDummyReadThread = false;
    //std::thread dummyReadThread(&CanEngine::poll , this);

    for (unsigned i = 0; i < activeCanId.size(); ++i)
    {
        if (verbose) logd(TAG, "Scanning id: %d\n", activeCanId[i]);

        //quitDummyReadThread = false;
        //std::thread dummyReadThread(&CanEngine::dummyRead, this);
        
        CalibrationData c;
        std::stringstream ss;
        bool f = false;

        ss << activeCanId[i];
        c.setParameter(PRM_DEVICE_ID, ss.str());

        LpmsCanIo sensor(&c);
        sensor.connect(ss.str());

        pcan.Reset(canChannel);
        setSensorToCommandMode(ss.str().c_str());
        MicroMeasure mm;
        mm.reset();
        while (mm.measure() < 1000000) {
            poll();
            sensor.checkState();
        }
        //quitDummyReadThread = true;;
        addSensor(&sensor);

        this_thread::sleep_for(chrono::milliseconds(1000));
        sensor.getFirmwareVersion();
        mm.reset();
        while (mm.measure() < 1000000) {
            poll();
            sensor.checkState();
        }

        sensor.setCommandMode();

        mm.reset();
        while (mm.measure() < 500000) {
            poll();
            sensor.checkState();

            //if (sensor.isWaitForAck() == false) {
            if (sensor.getConfigData()->firmwareVersionDig2 == 2)
            {
                v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C2));
                if (verbose) logd(TAG, "Discovered device: %s\n", ss.str().c_str());
                break;
            }
            else if (sensor.getConfigData()->firmwareVersionDig2 == 1)
            {
                v->push_back(DeviceListItem(ss.str().c_str(), DEVICE_LPMS_C));
                if (verbose) logd(TAG, "Discovered device: %s\n", ss.str().c_str());
                break;
            }
            //}
        }

        //sensor.setStreamMode();
        mm.reset();

        while (mm.measure() < 500000) {
            poll();
            sensor.checkState();
            updateSendQueue();
        }
        removeSensor(&sensor);
    }

    //quitDummyReadThread = true;
    //dummyReadThread.join();

}


void CanEngine::stopDiscovery(void)
{
    isStopDiscovery = true;
}

bool CanEngine::initIxxat(void)
{
    char hostAddr[64] = "192.168.0.10";
    char replyStr[MAX_RX_BYTES];
    int n;

    connectIxxat(hostAddr, 19227);

    if (verbose) logd(TAG, "IXXAT Device initialization\n");

    sendCmdIxxat("d ver\r\n");
    getReplyIxxat(replyStr, &n);

    sendCmdIxxat("c init 20\r\n");
    getReplyIxxat(replyStr, &n);
    if (strncmp(replyStr, "I OK", 4) != 0) {
        if (verbose) logd(TAG, "IXXAT init error!\n");
        return false;
    }

    sendCmdIxxat("c start\r\n");
    getReplyIxxat(replyStr, &n);
    if (strncmp(replyStr, "I OK", 4) != 0) {
        if (verbose) logd(TAG, "CAN start error!\n");
        return false;
    }

    sendCmdIxxat("c filter disable\r\n");
    getReplyIxxat(replyStr, &n);

    if (verbose) logd(TAG, "IXXAT initialization OK\n");

    ixxatCanDetected = true;
    ixxatCanInitialized = true;

    unsigned long iMode = 1;
    ioctlsocket(connectionSocket, FIONBIO, &iMode);

    return true;
}

bool CanEngine::connectIxxat(char *hostName, int port)
{
    WSADATA wsda;
    struct hostent *host;
    SOCKADDR_IN addr;
    int ret;

    WSAStartup(MAKEWORD(1, 1), &wsda);

    if (verbose) logd(TAG, "Creating socket for IXXAT interface\n");

    connectionSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (connectionSocket == SOCKET_ERROR) {
        if (verbose) logd(TAG, "Socket creation error %d!\n", WSAGetLastError());

        return false;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(hostName);

    if (addr.sin_addr.s_addr == INADDR_NONE) {
        host = NULL;
        if (verbose) logd(TAG, "Resolving host\n");
        host = gethostbyname(hostName);

        if (host == NULL) {
            if (verbose) logd(TAG, "Unknown host: %s!\n", hostName);
            return false;
        }

        memcpy(&addr.sin_addr, host->h_addr_list[0], host->h_length);
    }

    if (verbose) logd(TAG, "Connecting to %s:%d\n", hostName, port);

    ret = ::connect(connectionSocket, (struct sockaddr *) &addr, sizeof(addr));

    if (ret == SOCKET_ERROR) {
        if (verbose) logd(TAG, "Socket error %d!\n", WSAGetLastError());
        return false;
    }

    if (verbose) logd(TAG, "Connection with IXXAT interface OK\n");

    ixxatCanDetected = true;

    return true;
}

bool CanEngine::sendCmdIxxat(char *commandPtr)
{
    int bytesSent;
    char command[CMD_LEN];

    memset(command, 0, CMD_LEN);

    strcpy(command, commandPtr);

    bytesSent = send(connectionSocket, command, strlen(command), 0);

    if (bytesSent <= 0) {
        if (verbose) logd(TAG, "IXXAT send failed!\n");
        return false;
    }

    return true;
}

bool CanEngine::getReplyIxxat(char *replyPtr, int *nBytes)
{
    int bytesRecv;

    bytesRecv = recv(connectionSocket, replyPtr, MAX_RX_BYTES, 0);

    if (bytesRecv == SOCKET_ERROR) {
        return false;
    }

    replyPtr[bytesRecv] = 0;
    *nBytes = bytesRecv;

    // if (verbose) logd(TAG, "IXXAT receive reply: %s\n", replyPtr);  

    return 0;
}

void CanEngine::connect(void)
{
    TPCANStatus r;

    canChannel = PCAN_USBBUS1;

    if (peakCanInitialized == false) {
        r = pcan.Initialize(canChannel, PCAN_BAUD_125K);
        if (r != PCAN_ERROR_OK) {
            std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;
            return;
        }

        boost::uint8_t v = PCAN_PARAMETER_ON;
        r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);
    }

    peakCanDetected = true;
    peakCanInitialized = true;
}

void CanEngine::addSensor(LpmsCanIo *s)
{
    sensorList.push_back(s);

}

void CanEngine::removeSensor(LpmsCanIo *s)
{
    sensorList.remove(s);
}

void CanEngine::poll() {
    TPCANStatus r;
    TPCANMsg m;
    TPCANTimestamp t;
    char replyStr[MAX_RX_BYTES + 10];
    int n = 0;
    char messageIdStr[16];
    int messageId = 0;
    char messageByteStr[16];
    int messageByte[16];
    int v = 0;
    int nB = 0;
    char lengthStr[16];


    if (peakCanInitialized == true) {
        r = pcan.Read(canChannel, &m, &t);

        if (r == PCAN_ERROR_OK) {
            BOOST_FOREACH(LpmsCanIo *c, sensorList) {
                c->parseCanMsg(m);
            }
            //logd(TAG, "CAN Status: %x\n", r);
            //return;
        }
        
     updateSendQueue();
    }


    if (ixxatCanInitialized == true) {
        getReplyIxxat(replyStr, &n);

        for (int i = 0; i < n; ++i) {
            if (replyStr[i] == 'M' && ixxatState == 0) {
                ixxatState = 1;
                headerBytes = 0;
            }
            else if (replyStr[i] == 'E' && ixxatState == 0) {
            }

            if (ixxatState == 1) {
                headerData[headerBytes] = replyStr[i];
                ++headerBytes;

                if (headerBytes == 11) {
                    lengthStr[0] = headerData[4];
                    lengthStr[1] = 0;
                    messageLen = atoi(lengthStr);

                    for (int j = 6; j < 10; ++j) messageIdStr[j - 6] = headerData[j];
                    messageIdStr[4] = 0;

                    messageId = 0;
                    for (int k = 0; k < 4; ++k) {
                        switch (messageIdStr[k]) {
                        case 'A': v = 10; break;
                        case 'B': v = 11; break;
                        case 'C': v = 12; break;
                        case 'D': v = 13; break;
                        case 'E': v = 14; break;
                        case 'F': v = 15; break;
                        default:
                            messageByteStr[0] = messageIdStr[k];
                            messageByteStr[1] = 0;
                            v = atoi(messageByteStr);
                            break;
                        }
                        switch (k) {
                        case 0: messageId += v * 4096; break;
                        case 1: messageId += v * 256; break;
                        case 2: messageId += v * 16; break;
                        case 3: messageId += v; break;
                        }
                    }

                    ixxatState = 2;
                    messageBytes = 0;
                }
            }
            else if (ixxatState == 2) {
                messageData[messageBytes] = replyStr[i];
                ++messageBytes;

                if (messageBytes == (messageLen * 3 - 1)) {
                    for (int j = 0; j < messageBytes; j += 3) {
                        for (int k = 0; k < 2; ++k) {
                            switch (messageData[j + k]) {
                            case 'A': v = 10; break;
                            case 'B': v = 11; break;
                            case 'C': v = 12; break;
                            case 'D': v = 13; break;
                            case 'E': v = 14; break;
                            case 'F': v = 15; break;
                            default:
                                messageByteStr[0] = messageData[j + k];
                                messageByteStr[1] = 0;
                                v = atoi(messageByteStr);
                                break;
                            }
                            if (k == 1) messageByte[j / 3] += v; else messageByte[j / 3] = v * 16;
                        }
                    }

                    /* printf("Parsed message ID %d LEN %d: %x %x %x %x %x %x %x %x\n",
                        messageId,
                        messageLen,
                        messageByte[0],
                        messageByte[1],
                        messageByte[2],
                        messageByte[3],
                        messageByte[4],
                        messageByte[5],
                        messageByte[6],
                        messageByte[7]); */

                    m.ID = messageId;
                    m.MSGTYPE = PCAN_MESSAGE_STANDARD;
                    m.LEN = messageLen;
                    for (int j = 0; j < 8; ++j) m.DATA[j] = messageByte[j];

                    BOOST_FOREACH(LpmsCanIo *c, sensorList) c->parseCanMsg(m);

                    ixxatState = 0;
                }
            }
        }
    }
}

void CanEngine::close() {
    if (peakCanInitialized == true) {
        pcan.Uninitialize(canChannel);
    }

    peakCanInitialized = false;
}

bool CanEngine::updateSendQueue(void)
{
    if (peakCanInitialized == false && ixxatCanInitialized == false) 
        return false;
    
    TPCANStatus r;
    TPCANMsg m;
    char commandStr[64];

    BOOST_FOREACH(LpmsCanIo *c, sensorList) {
        int rT = 0;
        while (c->getTxMessage(&txQ) == true && rT < 1024) ++rT;
    }

    long timeout = 0;

    if (txQ.size() > 0) {
        m = txQ.front();

        /* printf("Msg. write %x %x %x %x %x %x %x %x\n",
            m.DATA[0],
            m.DATA[1],
            m.DATA[2],
            m.DATA[3],
            m.DATA[4],
            m.DATA[5],
            m.DATA[6],
            m.DATA[7]); */

        if (pcan.GetStatus(canChannel) == PCAN_ERROR_ANYBUSERR)
            return true;
        if (peakCanInitialized == true) {
            r = pcan.Write(canChannel, &m);
        }

        if (ixxatCanInitialized == true) {
            sprintf(commandStr, "M SD%x %x", m.LEN, m.ID);
            sendCmdIxxat(commandStr);
            for (int i = 0; i < m.LEN; ++i) {
                sprintf(commandStr, " %x", m.DATA[i]);
                sendCmdIxxat(commandStr);
            }
            sprintf(commandStr, "\r\n");
            sendCmdIxxat(commandStr);
        }

        if (pcan.GetStatus(canChannel) == PCAN_ERROR_ANYBUSERR)
        {
            while (pcan.GetStatus(canChannel) == PCAN_ERROR_ANYBUSERR)
                this_thread::sleep_for(chrono::milliseconds(1));
        }
        else
            std::this_thread::sleep_for(std::chrono::microseconds(10000));

        if (r == PCAN_ERROR_QXMTFULL) {
        }

        timeout = 0;
        txQ.pop();
    }

    return true;
}

bool CanEngine::isInterfacePresent(void)
{
    TPCANStatus r;
    canChannel = PCAN_USBBUS1;

    if (peakCanInitialized == false) {
        r = pcan.Initialize(canChannel, canBaudrate);

        if (r != PCAN_ERROR_OK) {
            std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;

            return false;
        }

        pcan.Uninitialize(canChannel);
    }
    return true;
}

void CanEngine::setBaudrate(int i)
{
    TPCANStatus r;
    canChannel = PCAN_USBBUS1;
    if (peakCanInitialized)
        pcan.Uninitialize(canChannel);

    switch (i) {
    case 0:
        canBaudrate = PCAN_BAUD_125K;
        std::cout << "[CanEngine] Baudrate changed to 125K." << std::endl;
        break;

    case 1:
        canBaudrate = PCAN_BAUD_250K;
        std::cout << "[CanEngine] Baudrate changed to 250K." << std::endl;
        break;

    case 2:
        canBaudrate = PCAN_BAUD_500K;
        std::cout << "[CanEngine] Baudrate changed to 500K." << std::endl;
        break;

    case 3:
        canBaudrate = PCAN_BAUD_1M;
        std::cout << "[CanEngine] Baudrate changed to 1M." << std::endl;
        break;
    }
    r = pcan.Initialize(canChannel, canBaudrate);
    
    if (r != PCAN_ERROR_OK) {
        std::cout << "[CanEngine] Peak Systems CAN Interface not detected." << std::endl;

        peakCanInitialized = false;
    }
    pcan.Reset(canChannel);
    boost::uint8_t v = PCAN_PARAMETER_ON;
    r = pcan.SetValue(canChannel, PCAN_BUSOFF_AUTORESET, &v, 1);

    peakCanInitialized = true;
}

void CanEngine::setVerbose(bool v)
{
    verbose = v;
}

void CanEngine::ping(int id)
{
    if (peakCanInitialized == false && ixxatCanInitialized == false) return;
    TPCANStatus r;
    TPCANMsg sendMsg;
    int function = 6;
    int length = 0;
    unsigned int txLrcCheck;
    sendMsg.ID = id + AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION;
    sendMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    sendMsg.LEN = 8;
    sendMsg.DATA[0] = 0x3a;
    sendMsg.DATA[1] = id & 0xff;
    sendMsg.DATA[2] = (id >> 8) & 0xff;
    sendMsg.DATA[3] = function & 0xff;
    sendMsg.DATA[4] = (function >> 8) & 0xff;
    sendMsg.DATA[5] = length & 0xff;
    sendMsg.DATA[6] = (length >> 8) & 0xff;
    txLrcCheck = id;
    txLrcCheck += function;
    txLrcCheck += length;
    sendMsg.DATA[7 + length] = txLrcCheck & 0xff;
    if (peakCanInitialized == true) {
        r = pcan.Write(canChannel, &sendMsg);
    }

    if (r == PCAN_ERROR_ANYBUSERR)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //if (canBaudrate == PCAN_BAUD_125K)
    //    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    for (int i = 0; i < 8; ++i)
        sendMsg.DATA[i] = 0;
    sendMsg.DATA[0] = (txLrcCheck >> 8) & 0xff;
    sendMsg.DATA[1] = 0x0d;
    sendMsg.DATA[2] = 0x0a;
    if (peakCanInitialized == true) {
        r = pcan.Write(canChannel, &sendMsg);
    }
}

void CanEngine::setSensorToCommandMode(const char* id)
{
    if (peakCanInitialized == false && ixxatCanInitialized == false)
        return;

    int address;
    stringstream ss;
    ss << string(id);
    ss >> address;
    unsigned int txLrcCheck;
    int v = 0;
    int function = 6;
    int length = 0;
    TPCANStatus r;
    TPCANMsg sendMsg;
    sendMsg.ID = address;
    sendMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    sendMsg.LEN = 8;

    sendMsg.DATA[0] = 0x3a;
    sendMsg.DATA[1] = address & 0xff;
    sendMsg.DATA[2] = (address >> 8) & 0xff;
    sendMsg.DATA[3] = function & 0xff;
    sendMsg.DATA[4] = (function >> 8) & 0xff;
    sendMsg.DATA[5] = length & 0xff;
    sendMsg.DATA[6] = (length >> 8) & 0xff;
    txLrcCheck = address;
    txLrcCheck += function;
    txLrcCheck += length;
    sendMsg.DATA[7 + length] = txLrcCheck & 0xff;
    if (peakCanInitialized == true) {
        r = pcan.Write(canChannel, &sendMsg);
    }
    while (pcan.GetStatus(canChannel) != PCAN_ERROR_OK)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //if (canBaudrate == PCAN_BAUD_125K)
    //    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (int i = 0; i < 8; ++i)
        sendMsg.DATA[i] = 0;
    sendMsg.DATA[0] = (txLrcCheck >> 8) & 0xff;
    sendMsg.DATA[1] = 0x0d;
    sendMsg.DATA[2] = 0x0a;
    if (peakCanInitialized == true) {
        r = pcan.Write(canChannel, &sendMsg);
    }

}

int CanEngine::getActiveDeviceId()
{
    TPCANStatus r;
    TPCANMsg m;
    TPCANTimestamp t;

    if (peakCanInitialized == true)
    {
        r = pcan.Read(canChannel, &m, &t);
        if (r != PCAN_ERROR_OK) {
            return -1;
        }
    }
    if (m.LEN == 0)
        return -1;
    return m.ID;
}


void CanEngine::pingCanIds()
{
    if (peakCanInitialized == false && ixxatCanInitialized == false) return;
    TPCANStatus r = PCAN_ERROR_OK;
    TPCANMsg m;
    TPCANTimestamp t;
    int i = 1;

    pcan.Reset(canChannel);
    for (int i = 1; i < 128; ++i)
    {
        if (peakCanInitialized == true)
        {
            ping(i);
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    }

    pcan.Reset(canChannel);
    for (int i = 1; i < 128; ++i)
    {
        if (peakCanInitialized == true)
        {
            //while (pcan.GetStatus(canChannel) != PCAN_ERROR_OK)
            //    this_thread::sleep_for(chrono::milliseconds(10));
            ping(i);
            this_thread::sleep_for(chrono::milliseconds(10));
            r = PCAN_ERROR_OK;
            while (r != PCAN_ERROR_QRCVEMPTY)
            {
                r = pcan.Read(canChannel, &m, &t);
            }
            if (m.LEN != 0)
            {
                int id = m.ID - 1300;
                if (id > 0 && id < 128)
                {
                    if (find(activeCanId.begin(), activeCanId.end(), id) == activeCanId.end())
                    {
                        activeCanId.push_back(id);
                    }
                }
            }
        }
    }
}

void CanEngine::dummyRead()
{
    logd(TAG, "dummy read started\n");
    /*
    while (!quitDummyReadThread)
    {
        poll();
    }
    */
    /*
    TPCANStatus r = PCAN_ERROR_OK;
    TPCANMsg m;
    TPCANTimestamp t;
    int i = 1;
    while (!quitDummyReadThread)
    {
        if (peakCanInitialized == true)
        {
            if (i < 128)
            {
                ping(i);
            }
            while (r != PCAN_ERROR_QRCVEMPTY)
                r = pcan.Read(canChannel, &m, &t);
            //if (r != PCAN_ERROR_OK) {
                //logd(TAG, "CAN Status: %x\n", r);
            //    continue;
            //}

            if (m.LEN != 0)
            {
                int id = m.ID - 1300;
                if (id > 0 && id < 128)
                {
                    if (find(activeCanId.begin(), activeCanId.end(), id) == activeCanId.end())
                    {
                        logd(TAG, "Found: %d \n", id);
                        activeCanId.push_back(id);
                    }
                }


                BOOST_FOREACH(LpmsCanIo *c, sensorScanList) {
                    c->parseCanMsg(m);
                }
            }
        }
    }
    logd(TAG, "dummy read stopped\n");
    */
}