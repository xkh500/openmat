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

#ifndef LPMS_CAN_IO
#define LPMS_CAN_IO

#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include "ImuData.h"
#include "MicroMeasure.h"
#include "LpmsIoInterface.h"

#ifdef _WIN32
	#include "windows.h"
	#include "PCANBasic.h"
#endif

#include "LpmsRegisterDefinitions.h"

#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>

#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_DATATYPE 16 // UCHAR4
#define AEROSPACE_CAN_LPMS_MODBUS_WRAPPER_FUNCTION 1300
#define AEROSPACE_CAN_ROLL 0x138
#define AEROSPACE_CAN_PITCH 0x137
#define AEROSPACE_CAN_YAW 0x141

class LpmsCanIo : public LpmsIoInterface {
public:
	LpmsCanIo(CalibrationData *configData);
	~LpmsCanIo(void);
	bool connect(std::string deviceId);

#ifdef _WIN32
	bool getTxMessage(std::queue<TPCANMsg> *topTxQ);
	bool parseCanMsg(TPCANMsg m);
#endif

protected:
	bool updateSendQueue(void);
	bool sendModbusData(unsigned address, unsigned function, unsigned length, unsigned char *data);
	bool parseModbusByte(unsigned char b);

	int inMC;
	int prevInMC;
	int outMC;

#ifdef _WIN32
	std::queue<TPCANMsg> txQ;
#endif
};

#endif
