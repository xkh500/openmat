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

#define LPMS_GOTO_BROADCAST 0
#define LPMS_GOTO_SLEEP 1
#define LPMS_RAW_DATA 61

#define LPMS_FACTORY_IMU_ID 1

LpmsRS232::LpmsRS232(CalibrationData *configData) :
	TAG("LpmsRS232"),
	LpmsIoInterface(configData)
{
    currentUartBaudrate = SELECT_LPMS_UART_BAUDRATE_115200;
    isOpen = false;
}
	
long long LpmsRS232::getConnectWait(void) 
{ 
    return 1000000;
}	
	
void LpmsRS232::listDevices(LpmsDeviceList* deviceList) 
{
	(void) deviceList;
}

int LpmsRS232::set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        logd(TAG, "error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY |INLCR | IGNCR | ICRNL); // shut off xon/xoff ctrl


    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading

    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        logd(TAG, "error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void LpmsRS232::set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        logd(TAG, "error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        logd(TAG, "error %d setting term attributes", errno);
}

bool LpmsRS232::connect(string deviceId)
{	
  	this->idNumber = deviceId;

    isOpen = false;
	fd = open (deviceId.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
        logd(TAG, "error %d opening %s: %s", errno, deviceId.c_str(), strerror (errno));
        return false;
	}
	int baudrate = B115200;
  	switch (currentUartBaudrate) {
    case SELECT_LPMS_UART_BAUDRATE_19200:
        baudrate = B19200;
        break;

    case SELECT_LPMS_UART_BAUDRATE_57600:
        baudrate = B57600;
        break;

    case SELECT_LPMS_UART_BAUDRATE_115200:
        baudrate = B115200;
        break;

    case SELECT_LPMS_UART_BAUDRATE_921600:
        baudrate = B921600;
        break;

    default:
        baudrate = B115200;
        break;
    }
    
	set_interface_attribs (fd, baudrate, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	//set_blocking (fd, 0);                // set no blocking

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
    isOpen = true;

    timestampOffset = 0.0;
    currentTimestamp = 0.0f;
    currentTimestampDouble = 0.0;

    return true;
}

bool LpmsRS232::write(unsigned char *txBuffer, unsigned bufferLength)
{
    int ret;
	ret = ::write(fd, txBuffer, bufferLength);           // send 7 character greeting
	return true;
}

bool LpmsRS232::read(unsigned char *rxBuffer, unsigned long *bytesReceived) {
	int bytes_avail;
	ioctl(fd, FIONREAD, &bytes_avail);

    if (bytes_avail > 4096) {
        if (verbose) logd(TAG, "Buffer overflow!\n");
        bytes_avail = 4096;
    }
	*bytesReceived = ::read(fd, rxBuffer, bytes_avail);//sizeof(rxBuffer));  // read up to 100 characters if ready to read
	return true;
}

void LpmsRS232::close(void) {	
    if (isOpen == false) return;

    isOpen = false;
	::close(fd);
	return;
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
        return true;
    }

    return false;
}

bool LpmsRS232::parseModbusByte(void)
{
	unsigned char b;

    while (dataQueue.size() > 0) {
        mutexDataQueue.lock();
        b = dataQueue.front();
        dataQueue.pop();
        mutexDataQueue.unlock();
       
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

bool LpmsRS232::pollData(void) 
{
	unsigned long bytesReceived;
    unsigned char rxBuffer[4096];
    bool packetOk = false;

    if (deviceStarted() == false) return false;

    if (read(rxBuffer, &bytesReceived) == false) {
        //isOpen = false;
        return false;
    }
    /*
    for (int i = 0; i < bytesReceived; ++i)
    {
        if (rxBuffer[i] == 0x3a) {
            printf("\n");
        }
        printf("%x ", rxBuffer[i]);
    }
    */

    mutexDataQueue.lock();
    for (unsigned int i = 0; i < bytesReceived; i++) {

        dataQueue.push((unsigned char)rxBuffer[i]);
    }
    mutexDataQueue.unlock();

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
