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

#ifndef LPEMG_IO_INTERFACE
#define LPEMG_IO_INTERFACE

#include "EmgData.h"
#include "CalibrationData.h"
#include "LpmsRegisterDefinitions.h"
#include "LpMatrix.h"
#include "MicroMeasure.h"
#include "LpIoBase.h"

#ifdef _WIN32
	#include "windows.h"
#endif

#include <iostream>
#include <string>
#include <queue>
#include <fstream>

#include <boost/cstdint.hpp>

// Firmware packet length
#define FIRMWARE_PACKET_LENGTH 256
#define FIRMWARE_PACKET_LENGTH_LPMS_BLE 128

// State machine definitions
#define IDLE_STATE -1
#define ACK_MAX_TIME 3000000
#ifndef MAX_UPLOAD_TIME
#define MAX_UPLOAD_TIME 20000000
#endif
#define MAX_COMMAND_RESEND 3

// Class for low-level communication with LPMS devices
class LpemgIoInterface : public LpIoBase {
public:
	// Constructor
	LpemgIoInterface(CalibrationData *configData);
	
	virtual ~LpemgIoInterface() { };	
	
	// Connects to device
	virtual bool connect(std::string deviceId) ;
	
	// Checks if device is started
	virtual bool deviceStarted(void);
	
	// Retrieves current data
	virtual void loadData(EmgData *data);
	
	// Polls data from device
	virtual bool pollData(void);
	
	// Closes data connection
	virtual void close(void);
	
	// Starts streaming data from device
	virtual void startStreaming(void);
	
	// Stops streaming data from device
	virtual void stopStreaming(void);
	
	// Retrieves connection delay
	virtual long long getConnectWait(void);
	
	// Retrieves sampling time
	virtual float getSamplingTime(void);

	// Checks current state machine state
	bool checkState(void);
	
	// Retrieves current configuraton parameters
	CalibrationData *getConfigData(void);
	
	// Checks if state machine is currently waiting for data
	// bool isWaitForData(void);
	
	// Checks if state machine is currently waiting for acknowledge
	// bool isWaitForAck(void);
	
	// Checks if currently calibrating
	bool isCalibrating(void);
	
	// Checks if error has occurred
	bool isError(void);	
	
	// Retrieves upload progress
	bool getUploadProgress(int *p);
	
	// Starts uploading firmware
	virtual bool startUploadFirmware(std::string fn);
	
	// Starts uploading IAP
	virtual bool startUploadIap(std::string fn);
	
	// Sets sensor to command mode
	bool setCommandMode(void);
	
	// Sets sensor to streaming mode
	bool setStreamMode(void);
	
	// Sets sensor to sleep mode
	bool setSleepMode(void);

	// Restores factory settings
	bool restoreFactoryValue(void);
	
	// Starts self test
	bool setSelfTest(long v);
	
	// Selects data for transmission
	bool selectData(long p);
	
	// Sets IMU ID
	bool setImuId(long v);
	
	// Sets stream frequency
	bool setStreamFrequency(long v);
	
	// Starts gyroscope calibration
	bool getFirmwareVersion(void);
	bool getConfig(void);
	bool getImuId(void);
	bool getStatus(void);
	bool getSensorData(void);
	bool writeRegisters(void);
	int getMode(void);
	float getLatestLatency(void);
	void zeroData(EmgData* d);
	bool setGyrAlignment(LpMatrix3x3f m);
	bool setGyrAlignmentBias(LpVector3f v);
	bool getGyrAlignment(void);
	bool getGyrAlignmentBias(void);
	long getConfigReg(void);
	bool getLatestData(EmgData *d);
	void clearRxBuffer(void);
	void clearDataQueue(void);
	void setTxRxImuId(int id);
	bool setLpBusDataMode(int v);
	bool setTimestamp(float v);
	bool armTimestampReset(int v);
	bool parseSensorData(void);	
	
protected:
	// Virtual functions to be overwritten by hardware dependent modules -->
	// virtual bool parseModbusByte(void);	

	// Firmware / IAP upload handling
	bool handleFirmwareFrame(void);
	bool handleIAPFrame(void);
	bool parseFunction(void);
	bool checkUploadTimeout(void);

protected:
	// Internal variables

    std::queue<unsigned char> dataQueue;
	std::queue<EmgData> emgDataQueue;
	std::vector<unsigned char> oneTx;

	int rxState;
	int rawDataIndex;	
	int pCount;
	EmgData emgData;
	CalibrationData *configData;
	std::ifstream ifs;	
	long configReg;
	long lpmsStatus;
	long long firmwarePages;
	int currentMode;
	float latestLatency;
	MicroMeasure latencyTimer;
	MicroMeasure uploadTimer;
	bool newDataFlag;
	float timestampOffset;
	float currentTimestamp;
	bool isOpen;
	int firmwarePageSize;
};	

#endif
