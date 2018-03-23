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

#include "LpemgBluetooth.h"

#ifdef _WIN32
#include <ws2bth.h>
#include <windows.h>

#include "BluetoothAPIs.h"
#endif

#ifdef __GNUC__
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

// On Ubuntu you will need to install libbluetooth-dev
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#endif


static volatile bool isStopDiscovery = false;
#ifdef _WIN32
static HBLUETOOTH_RADIO_FIND m_bt = NULL;
static HBLUETOOTH_DEVICE_FIND m_bt_dev = NULL;
#endif

LpemgBluetooth::LpemgBluetooth(CalibrationData *configData) :
	LpemgIoInterface(configData)
{
}

LpemgBluetooth::~LpemgBluetooth(void)
{
	close();
}
	
long long LpemgBluetooth::getConnectWait(void) 
{ 
	return 6000000; 
}

#ifdef _WIN32
void LpemgBluetooth::listDevices(LpmsDeviceList *deviceList)
{
    BLUETOOTH_RADIO_INFO m_bt_info = { sizeof(BLUETOOTH_RADIO_INFO), 0, };
    BLUETOOTH_DEVICE_SEARCH_PARAMS m_search_params = {
        sizeof(BLUETOOTH_DEVICE_SEARCH_PARAMS),
        1,
        1,
        1,
        1,
        1,
        5,
        NULL
    };

    BLUETOOTH_DEVICE_INFO m_device_info = { sizeof(BLUETOOTH_DEVICE_INFO), 0, };

    HANDLE m_radio = NULL;
    HBLUETOOTH_RADIO_FIND m_bt = NULL;
    HBLUETOOTH_DEVICE_FIND m_bt_dev = NULL;
    int m_radio_id;
    int m_device_id;
    DWORD mbtinfo_ret;

    isStopDiscovery = false;

    BLUETOOTH_FIND_RADIO_PARAMS m_bt_find_radio = {sizeof(BLUETOOTH_FIND_RADIO_PARAMS)};

    std::cout << "[LpemgBluetooth] Searching for Bluetooth LPEMG devices" << std::endl;

    WORD wVersionRequested;
    WSADATA wsaData;
    wVersionRequested = MAKEWORD(2, 0);

    for (int i=0; i<5; ++i) {
        WSAStartup(wVersionRequested, &wsaData);

        while (isStopDiscovery == false) {
            m_bt = BluetoothFindFirstRadio(&m_bt_find_radio, &m_radio);

            if (m_bt == NULL)
                break;

            m_radio_id = 0;

            do {
                mbtinfo_ret = BluetoothGetRadioInfo(m_radio, &m_bt_info);

                m_search_params.hRadio = m_radio;
                ZeroMemory(&m_device_info, sizeof(BLUETOOTH_DEVICE_INFO));
                m_device_info.dwSize = sizeof(BLUETOOTH_DEVICE_INFO);
                m_bt_dev = BluetoothFindFirstDevice(&m_search_params, &m_device_info);

                m_radio_id++;
                m_device_id = 0;

                do {
                    wprintf(L"[LpemgBluetooth] Device: %d\r\n", m_device_id);
                    wprintf(L"[LpemgBluetooth] Instance Name: %s\r\n", m_device_info.szName);
                    wprintf(L"[LpemgBluetooth] Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                        m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4],
                        m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2],
                        m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);
                    wprintf(L"[LpemgBluetooth] Class: 0x%08x\r\n",
                        m_device_info.ulClassofDevice);
                    wprintf(L"[LpemgBluetooth] Connected: %s\r\n",
                        m_device_info.fConnected ? L"true" : L"false");
                    wprintf(L"[LpemgBluetooth] Authenticated: %s\r\n",
                        m_device_info.fAuthenticated ? L"true" : L"false");
                    wprintf(L"[LpemgBluetooth] Remembered: %s\r\n",
                        m_device_info.fRemembered ? L"true" : L"false");

                    if ((m_device_info.szName[0] == 'L'
                        && m_device_info.szName[1] == 'P'
                        && m_device_info.szName[2] == 'E'
                        && m_device_info.szName[3] == 'M'
                        && m_device_info.szName[4] == 'G')
                        || (m_device_info.szName[0] == 'L'
                            && m_device_info.szName[1] == 'P'
                            && m_device_info.szName[2] == 'E'
                            && m_device_info.szName[3] == 'M'
                            && m_device_info.szName[4] == 'M'
                            && m_device_info.szName[5] == 'G')) {
                        char addressString[64];
                        sprintf(addressString, "%02X:%02X:%02X:%02X:%02X:%02X",
                            m_device_info.Address.rgBytes[5], m_device_info.Address.rgBytes[4],
                            m_device_info.Address.rgBytes[3], m_device_info.Address.rgBytes[2],
                            m_device_info.Address.rgBytes[1], m_device_info.Address.rgBytes[0]);

                        if (deviceList->findDevice(addressString) == -1) {
                            deviceList->push_back(DeviceListItem(addressString, DEVICE_LPEMG_B));

                            std::cout << "[LpemgBluetooth] Discovered device: " << addressString << endl;
                        }
                    }

                    m_device_id++;

                    if (isStopDiscovery == true)
                        break;

                } while(BluetoothFindNextDevice(m_bt_dev, &m_device_info) && isStopDiscovery == false);

                if (isStopDiscovery == true) {
                    BluetoothFindDeviceClose(m_bt_dev);
                    break;
                }

            } while(BluetoothFindNextRadio(&m_bt_find_radio, &m_radio) && isStopDiscovery == false);

            if (isStopDiscovery == true) {
                BluetoothFindRadioClose(m_bt);
                break;
            }

            if(!BluetoothFindNextRadio(&m_bt_find_radio, &m_radio))
                break;

            // Sleep(1000);
        }
    }
}
#else
void LpemgBluetooth::listDevices(LpmsDeviceList *deviceList)
{
    logd("LpmsBBluetooth", "Searching for Bluetooth LPMS devices\n");
    inquiry_info *ii = NULL;
    int max_rsp, num_rsp;
    int dev_id, sock, len, flags;
    int i;
    char addr[19] = { 0 };
    char name[248] = { 0 };
    isStopDiscovery = false;

    dev_id = hci_get_route(NULL);
    sock = hci_open_dev( dev_id );

    if (dev_id < 0 || sock < 0) {
        std::cout << "[LpemgBluetooth] Error opening socket" << std::endl;
        return;
    }

    len  = 20;
    max_rsp = 255;
    flags = IREQ_CACHE_FLUSH;
    ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));

    num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
    if (num_rsp < 0) {
        std::cout << "[LpemgBluetooth] Error with hci_inquiry" << std::endl;
    }

    for (i = 0; i < num_rsp; i++) {
        ba2str(&(ii+i)->bdaddr, addr);
        memset(name, 0, sizeof(name));

        if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), name, 0) < 0)
	    strcpy(name, "[unknown]");

        if (std::string(name).find("LPEMG") != std::string::npos) {
            deviceList->push_back(DeviceListItem(addr, DEVICE_LPEMG_B));
            std::cout << "[LpemgBluetooth] Address: " << addr << " name: " << name << std::endl;
        }
        else if (std::string(name).find("LPEMMG") != std::string::npos) {
            deviceList->push_back(DeviceListItem(addr, DEVICE_LPEMG_B));
            std::cout << "[LpemgBluetooth] Address: " << addr << " name: " << name << std::endl;
        }

        if (isStopDiscovery)
            break;
    }
    free(ii);
    ::close(sock);
}
#endif

void LpemgBluetooth::stopDiscovery(void)
{
	isStopDiscovery = true;	
}

bool LpemgBluetooth::connect(string deviceId)
{
    this->bluetoothAddress = deviceId;
#ifdef _WIN32
    SOCKADDR_BTH sa = { 0 };
    int sa_len = sizeof(sa);
    WORD wVersionRequested;
    WSADATA wsaData;
    wVersionRequested = MAKEWORD(2, 0);

	if (WSAStartup(wVersionRequested, &wsaData) != 0) {
		isOpen = false;

		std::cout << "[LpemgBluetooth] Error during WSA Startup." << std::endl;

		return false;
    }

	if (SOCKET_ERROR == WSAStringToAddress(const_cast<char*>(bluetoothAddress.c_str()), AF_BTH, 
		NULL, (LPSOCKADDR) &sa, &sa_len )) {
		WSACleanup();
		isOpen = false;

		std::cout << "[LpemgBluetooth] Couldn't get Bluetooth address." << std::endl;

		return false;
    }

    sock = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
    if( SOCKET_ERROR == sock ) {
		close();

		std::cout << "[LpemgBluetooth] Couldn't create socket." << std::endl;

		return false;
    }
	
	sa.port = 1;
	
	int e = ::connect(sock, (LPSOCKADDR) &sa, sa_len);
    if (SOCKET_ERROR == e) {
		close();	

		std::cout << "[LpemgBluetooth] Couldn't connect." << std::endl;

		return false;
    }

    // Set socket to non-blocking mode.
    unsigned long iMode = 1;
    ioctlsocket(sock, FIONBIO, &iMode);
#else
    int status;
    int on;
    int rc;
    struct sockaddr_rc addr = { 0 };

    bzSocket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(deviceId.c_str(), &addr.rc_bdaddr);

    status = ::connect(bzSocket, (struct sockaddr *)&addr, sizeof(addr));

    if (status < 0) {
        std::cout << "[LpmsBBluetooth] Couldn't create socket." << std::endl;
        ::close(bzSocket);
        return false;
    }
#endif
	std::cout << "[LpemgBluetooth] Connected!" << std::endl;	
	
	mm.reset();
	
	oneTx.clear();

	rxState = PACKET_END;
	currentState = IDLE_STATE;
	
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
	
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;

#ifdef _WIN32
#else
    std::thread t(&LpemgBluetooth::runRead, this);
    t.detach();
#endif
	
	return true;
}

void LpemgBluetooth::close(void)
{
	// if (isOpen == false) return;

	isOpen = false;
	
	// std::cout << "[LPMS-B] Closing connection" << std::endl;	
#ifdef _WIN32
	shutdown(sock, SD_SEND);
	closesocket(sock);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ::close(bzSocket);
#endif
}

#ifndef _WIN32
void LpemgBluetooth::runRead(void)
{
    unsigned long bytesReceived;
    char rxBuffer[1024];

    while (isOpen == true)
    {

        bytesReceived = ::read(bzSocket, rxBuffer, 512);
        if (bytesReceived <= 0 || bytesReceived >= 1024)
        {
            cout << "[LpmsBBluetoothLinux] LPMS connection timeout has occured (device: " << bluetoothAddress.c_str() << ")." << endl;

            ::close(bzSocket);  //birdy
            isOpen = false;
        } else {
            mutexDataQueue.lock();
            for (unsigned int i=0; i < bytesReceived; i++) {
                dataQueue.push((unsigned char) rxBuffer[i]);
            }
            mutexDataQueue.unlock();
        }
    }

    ::close(bzSocket);
}
#endif

bool LpemgBluetooth::read(unsigned char *rxBuffer, unsigned long *bytesReceived)
{
#ifdef _WIN32
	float timeoutT;
	int n;

	n = recv(sock, (char *)rxBuffer, 1024, 0);
	
	if (n == SOCKET_ERROR) {
		timeoutT = (float) mm.measure() / 1e6f;
		*bytesReceived = 0;
	} else {
		*bytesReceived = (unsigned long) n;
        if (0) {
            for (int i = 0; i < (int)*bytesReceived; i++) {
                printf("%d, ", i);
                printf("%x ", rxBuffer[i]);
            }
            printf("\n ");
        }

        mm.reset();
        timeoutT = 0;
	}
	
	if (timeoutT > 10.0f) {
		std::cout << "[LpemgBluetooth] Bluetooth timeout" << std::endl;
		
		shutdown(sock, SD_SEND);
		closesocket(sock);
		
		while (!dataQueue.empty()) dataQueue.pop();
		
		return false;
	}
#endif
	return true;
}

bool LpemgBluetooth::write(char *txBuffer, unsigned bufferLength)
{
#ifdef _WIN32
	if (send(sock, txBuffer, bufferLength, 0) == SOCKET_ERROR) {
		close();

		return false;
	}
#else
    int s = ::write(bzSocket, txBuffer, bufferLength);
#endif
	return true;
}

bool LpemgBluetooth::sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data)
{
	char txData[1024];
	unsigned int txLrcCheck;
	
	if (length > 1014) return false;

	txData[0] = 0x3a;
	txData[1] = address & 0xff;
	// txData[2] = (address >> 8) & 0xff;
	txData[2] = function & 0xff;
	// txData[4] = (function >> 8) & 0xff;
	txData[3] = length & 0xff;
	txData[4] = (length >> 8) & 0xff;
	
	for (unsigned int i=0; i < length; ++i) {
		txData[5+i] = data[i];
	}
	
	txLrcCheck = address;
	txLrcCheck += function;
	txLrcCheck += length;
	
	for (unsigned int i=0; i < length; i++) {
		txLrcCheck += data[i];
	}
	
	txData[5 + length] = txLrcCheck & 0xff;
	txData[6 + length] = (txLrcCheck >> 8) & 0xff;
	txData[7 + length] = 0x0d;
	txData[8 + length] = 0x0a;
	
	if (write(txData, length+9) == true) {
		return true;
	}
	
	return false;
}

bool LpemgBluetooth::parseModbusByte(void)
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
			rxState = PACKET_FUNCTION0;
			break;

		case PACKET_FUNCTION0:
			currentFunction = b;
			rxState = PACKET_LENGTH0;				
			break;

		case PACKET_LENGTH0:
			currentLength = b;
			rxState = PACKET_LENGTH1;
		break;
				
		case PACKET_LENGTH1:
			currentLength = currentLength + ((unsigned) b * 256);
			
			if (currentLength < 256) {			
				rxState = PACKET_RAW_DATA;
				rawDataIndex = currentLength;
			} else {
				rxState = PACKET_END;
			}
		break;
				
		case PACKET_RAW_DATA:
			if (rawDataIndex == 0) {
				lrcCheck = currentAddress + currentFunction + currentLength;
				for (unsigned i=0; i<oneTx.size(); i++) lrcCheck += oneTx[i];
				lrcReceived = b;
				rxState = PACKET_LRC_CHECK1;
			} else {
				oneTx.push_back(b);
				--rawDataIndex;
			}
			break;
			
		case PACKET_LRC_CHECK1:
			lrcReceived = lrcReceived + ((unsigned) b * 256);			
			if (lrcReceived == lrcCheck) {
				parseFunction();
			} else {
				std::cout << "[LpemgBluetooth] Checksum fail in data packet" << std::endl;
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

bool LpemgBluetooth::pollData(void) 
{
#ifdef _WIN32
	unsigned long bytesReceived;
	unsigned char rxBuffer[1024];
	bool packetOk = false;
	
	if (deviceStarted() == false) return false;
	
	if (read(rxBuffer, &bytesReceived) == false) {
		isOpen = false;
		return false;
	}	
	
	for (unsigned int i=0; i < bytesReceived; i++) {
		dataQueue.push(rxBuffer[i]);
		// printf("%x ", (unsigned char)rxBuffer[i]);
	}
	// if (bytesReceived > 0) printf("\n ");
	
	return true;
#endif
}

bool LpemgBluetooth::deviceStarted(void)
{
	return isOpen;
}
