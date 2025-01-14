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

#ifndef LPMS_TCP
#define LPMS_TCP

#define WS_VERSION_REQUIRED 0x0101
#include <winsock2.h>
#include <windows.h>
#include <string>
#pragma comment(lib, "ws2_32.lib")

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"
#include "DeviceListItem.h"

#define kFBTCPIP_Stream 0

typedef int FBTCPIPSocketType;
typedef unsigned long kULong;

class TcpClientCom {
public:
	bool CreateSocket(int &pSocket, FBTCPIPSocketType pType, const char *pProtocol="ip", bool pNonBlocking=true) 
	{
		WSADATA wsadata;
		int error;

		error = WSAStartup(0x0202, &wsadata);

		if (error) {
			return false;
		}

		if (wsadata.wVersion != 0x0202) {
			WSACleanup();
			return false;
		}

		socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (socket_ == INVALID_SOCKET) {
			return false;
		}
		
		return true;
	}
	
	bool Connect(int &pSocket, const char *pAddr, int pPort) 
	{
		SOCKADDR_IN target;
		
		target.sin_family = AF_INET;
		target.sin_port = htons (pPort);
		target.sin_addr.s_addr = inet_addr (pAddr);	
	
		if (::connect(socket_, (SOCKADDR *)&target, sizeof(target)) == SOCKET_ERROR) {
			return false;
		} else {
			pSocket = socket_;
		
			return true;
		}
	}
	
	bool CloseSocket(int &pSocket) {
		if (socket_) closesocket(socket_);

		WSACleanup();
		
		return true;
	}
	
	bool Write(int pSocket, void *lpBuffer, int nNumberOfBytesToWrite, int *lpNumberOfBytesWritten=NULL)
	{
		send(socket_, (char *)lpBuffer, nNumberOfBytesToWrite, 0);

		return true;		
	}
	
	bool ReadBlocking(int pSocket, void *lpBuffer, int nNumberOfBytesToRead, int *lpNumberOfBytesRead=NULL, kULong pTimeOut=500)
	{
		recv(socket_, (char *)lpBuffer, nNumberOfBytesToRead, 0);
		
		return true;
	}
	
public:
	SOCKET socket_;
};

class LpmsTcp : public LpmsIoInterface {
public:
	LpmsTcp(CalibrationData *configData);
	static void listDevices(LpmsDeviceList *deviceList);
	bool connect(string deviceId);
	void close(void);
	bool pollData(void);
	bool deviceStarted(void);
	bool parseModbusByte(void);

private:
	bool read(char *rxBuffer, unsigned long *bytesReceived); 
	bool write(char *txBuffer, unsigned bufferLength);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	long long getConnectWait(void);

	std::string mNetworkAddress; 
	int mNetworkPort; 
	int mSocket;
	TcpClientCom mTCP;
	string portname;
	bool isOpen;
	string idNumber;
	MicroMeasure mm;	
};

#endif

 