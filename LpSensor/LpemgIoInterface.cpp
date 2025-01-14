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

#include "LpemgIoInterface.h"

INIT_LOGGING

LpemgIoInterface::LpemgIoInterface(CalibrationData *configData) :
	configData(configData)
{
	firmwarePageSize = FIRMWARE_PACKET_LENGTH;
}

bool LpemgIoInterface::connect(std::string deviceId) 
{ 
	currentState = IDLE_STATE;
	
	waitForAck = false;
	waitForData = false;
	ackReceived = false;
	dataReceived = false;
	ackTimeout = 0;	
	dataTimeout = 0;
	
	lpmsStatus = 0;
	configReg = 0;
	imuId = 1;
	currentMode = LPMS_STREAM_MODE;
	
	latestLatency = 0.0f;
	timestampOffset = 0.0f;
	currentTimestamp = 0.0f;
	
	resendI = 0;
	cLength = 0;
	
	zeroData(&emgData);
	
	return true;
}

void LpemgIoInterface::zeroData(EmgData* d)
{
    d->timeStamp = 0.0;
    std::fill_n(d->voltage, 20, 0.0f);
    d->audio[0] = d->audio[1] = 0.f;
    d->gyro[0] = d->gyro[1] = d->gyro[2] = 0.f;
    d->acc[0] = d->acc[1] = d->acc[2] = 0.f;
}

void LpemgIoInterface::setTxRxImuId(int id)
{
	imuId = id;
}

bool LpemgIoInterface::deviceStarted(void) 
{ 
	return true;
}
	
void LpemgIoInterface::loadData(EmgData *data) 
{ 
	*data = emgData;
}

bool LpemgIoInterface::pollData(void) 
{ 
	return true; 
}

void LpemgIoInterface::close(void) 
{ 
}

void LpemgIoInterface::startStreaming(void) 
{ 
}

void LpemgIoInterface::stopStreaming(void) 
{ 
}

long long LpemgIoInterface::getConnectWait(void) 
{ 
	return 0; 
}

float LpemgIoInterface::getSamplingTime(void) 
{
	return 0.005f;
}

bool LpemgIoInterface::parseSensorData(void)
{
    // float v[20];
    float audio[2];
    float acc[3];
    float gyro[3];
	short s;
	
	const float r2d = 57.2958f;

    zeroData(&emgData);

    unsigned offs = 0;
	int i = 0;

    fromBuffer(oneTx, offs, &currentTimestamp);
    offs += 4;
	
    if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
        /* if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
            for (i = 0; i < 20; ++i) {
                fromBufferInt16(oneTx, offs, &s);
                v[i] = ((float)s) / 1000.0f;
                offs = offs + 2;
            }
			std::move(v, v + 20, emgData.voltage);
		} */
		
		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
			fromBufferInt16(oneTx, offs, &s);
			emgData.singleVoltage = ((float)s) / 1000.0f;
			offs = offs + 2;
		}
		
        if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
            for (i = 0; i < 2; ++i) {
                fromBufferInt16(oneTx, offs, &s);
                audio[i] = ((float)s) / 10000.0f;
                offs = offs + 2;
            }
			memcpy(emgData.audio, audio, 2 * sizeof(float));			
		}	
		
        if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
            for (i = 0; i < 3; ++i) {
                fromBufferInt16(oneTx, offs, &s);
                gyro[i] = ((float)s) / 100.0f;
                offs = offs + 2;
            }
			memcpy(emgData.gyro, gyro, 3 * sizeof(float));			
        }

        if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
            for (i = 0; i < 3; ++i) {
                fromBufferInt16(oneTx, offs, &s);
                acc[i] = ((float)s) / 1000.0f;
                offs = offs + 2;
            }
			memcpy(emgData.acc, acc, 3 * sizeof(float));			
        }
	} else {		
        /* if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {		
			for (i = 0; i < 20; ++i) {
				fromBuffer(oneTx, offs, &v[i]);
				offs += 4;
			}
			std::move(v, v + 20, emgData.voltage); // = v; // float(s)*3.3f / 4096.0f;			
			
			offs += 4; // unfindable length=84 bug workaround
		} */
		
        if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {		
			fromBuffer(oneTx, offs, &(emgData.singleVoltage));
			offs += 4;
		}		

        if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {		
			fromBuffer(oneTx, offs, &audio[0]);
			offs += 4;

			fromBuffer(oneTx, offs, &audio[1]);
			offs += 4;
			
			memcpy(emgData.audio, audio, 2 * sizeof(float));
		}

		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			fromBuffer(oneTx, offs, &gyro[0], &gyro[1], &gyro[2]);
			offs += 3 * 4;
			memcpy(emgData.gyro, gyro, 3 * sizeof(float));			
		}

		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			fromBuffer(oneTx, offs, &acc[0], &acc[1], &acc[2]);
			offs += 3 * 4;
			memcpy(emgData.acc, acc, 3 * sizeof(float));			
		}
	}
	
	emgData.timeStamp = currentTimestamp;

    if (emgDataQueue.size() < 64) {
        emgDataQueue.push(emgData);
    }

    return true;
}

bool LpemgIoInterface::getLatestData(EmgData *d) 
{
	if (emgDataQueue.empty() == true) return false;
	
	*d = emgDataQueue.front();
	emgDataQueue.pop();
	
	return true;
}

void LpemgIoInterface::clearDataQueue(void)
{
	while (dataQueue.empty() == false) dataQueue.pop();
}

bool LpemgIoInterface::parseFunction(void) 
{
	long l;
	long i0, i1, i2;
	int selectedData = 0;
	
	if (waitForAck == true) {
		if (isAck() == true) {
			ackReceived = true;
						
			return true;
		}

		if (isNack() == true) {
			receiveReset();
			
			return false;
		}
	}

	switch (currentFunction) {
	case GET_CONFIG:

		latestLatency = latencyTimer.measure() / 1000.0f;	
	
		if (fromBuffer(oneTx, &l)) configReg = l;		

		if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_5HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_10HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_25HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_25HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_25HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_50HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_50HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_100HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_200HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_200HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_400HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_400HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_400HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_800HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_800HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_800HZ_ENABLED\n";
		}
		// else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1600HZ_ENABLED) {
			// configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_1600HZ);
			// cout << "[CONFIG] LPMS_STREAM_FREQ_1600HZ_ENABLED\n";
		// }
		else {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_100HZ_ENABLED\n";
		}

		
		if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
			configData->setParameter(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_16);
			cout << "[CONFIG] LPMS_LPBUS_DATA_MODE_16BIT_ENABLED\n";
		} else {
			configData->setParameter(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_32);
			cout << "[CONFIG] LPMS_LPBUS_DATA_MODE_32BIT_ENABLED\n";
		}		
		
		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ACC_RAW_OUTPUT_ENABLED\n";
		} else {
			selectedData &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ACC_RAW_OUTPUT_DISABLED\n";
		}
		
		if ((configReg & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_GYRO_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_GYRO_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_GYRO_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_GYRO_OUTPUT_DISABLED\n";
		}
		
		if ((configReg & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_PRESSURE_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_PRESSURE_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_PRESSURE_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ALTITUDE_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ALTITUDE_OUTPUT_DISABLED\n";
		}		
		
		configData->setParameter(PRM_SELECT_DATA, selectedData);
		
		/* latestLatency = latencyTimer.measure() / 1000.0f;

		if (fromBuffer(oneTx, &l)) configReg = l;

		if ((configReg & LPMS_GYR_THRES_ENABLED) != 0) {
			configData->setParameter(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_ENABLED);
			cout << "[CONFIG] LPMS_GYR_THRES_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_GYR_THRESHOLD_ENABLED, SELECT_IMU_GYR_THRESH_DISABLED);
			cout << "[CONFIG] LPMS_GYR_THRES_DISABLED\n";
		}

		if ((configReg & LPMS_GYR_AUTOCAL_ENABLED) != 0) {
			configData->setParameter(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_ENABLED);
			cout << "[CONFIG] LPMS_GYR_AUTOCAL_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_GYR_AUTOCALIBRATION, SELECT_GYR_AUTOCALIBRATION_DISABLED);
			cout << "[CONFIG] LPMS_GYR_AUTOCAL_DISABLED\n";
		}

		if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_5HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_5HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_10HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_10HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_25HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_25HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_25HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_50HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_50HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_100HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_200HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_200HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_400HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_400HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_400HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_800HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_800HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_800HZ_ENABLED\n";
		}
		else if ((configReg & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_1600HZ_ENABLED) {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_1600HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_1600HZ_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_SAMPLING_RATE, SELECT_STREAM_FREQ_100HZ);
			cout << "[CONFIG] LPMS_STREAM_FREQ_100HZ_ENABLED\n";
		}

		if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_125K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_125KBPS);
			cout << "[CONFIG] LPMS_CANBUS_BAUDRATE_125K_ENABLED\n";
		}
		else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_250K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_250KBPS);
			cout << "[CONFIG] LPMS_CANBUS_BAUDRATE_250K_ENABLED\n";
		}
		else if ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_500K_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_500KBPS);
			cout << "[CONFIG] LPMS_CANBUS_BAUDRATE_500K_ENABLED\n";
		}
		else {// ((configReg & LPMS_CAN_BAUDRATE_MASK) == LPMS_CANBUS_BAUDRATE_1M_ENABLED) {
			configData->setParameter(PRM_CAN_BAUDRATE, SELECT_CAN_BAUDRATE_1000KBPS);
			cout << "[CONFIG] LPMS_CANBUS_BAUDRATE_1000K_ENABLED\n";
		}

		if ((configReg & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
			configData->setParameter(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_16);
			cout << "[CONFIG] LPMS_LPBUS_DATA_MODE_16BIT_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_LPBUS_DATA_MODE, SELECT_LPMS_LPBUS_DATA_MODE_32);
			cout << "[CONFIG] LPMS_LPBUS_DATA_MODE_32BIT_ENABLED\n";
		}

		if ((configReg & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ACC_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_ACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ACC_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_MAG_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_MAG_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_MAG_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_MAG_OUTPUT_DISABLED\n";
		} */

		/* if ((configReg & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_QUAT_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_QUAT_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_QUAT_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_QUAT_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_EULER_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_EULER_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_EULER_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_EULER_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_EULER_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_LINACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_LINACC_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_LINACC_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_LINACC_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_ANGULAR_VELOCITY_OUTPUT_DISABLED\n";
		}

		if ((configReg & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
			selectedData |= SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_HEAVEMOTION_OUTPUT_ENABLED\n";
		}
		else {
			selectedData &= ~SELECT_LPMS_HEAVEMOTION_OUTPUT_ENABLED;
			cout << "[CONFIG] LPMS_HEAVEMOTION_OUTPUT_DISABLED\n";
		}

		configData->setParameter(PRM_SELECT_DATA, selectedData);

		if ((configReg & LPMS_HEAVEMOTION_ENABLED) != 0) {
			configData->setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_ENABLED);
			cout << "[CONFIG] LPMS_HEAVEMOTION_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_HEAVEMOTION_ENABLED, SELECT_HEAVEMOTION_DISABLED);
			cout << "[CONFIG] LPMS_HEAVEMOTION_DISABLED\n";
		}

		if ((configReg & LPMS_GAIT_TRACKING_ENABLED) != 0) {
			configData->setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_ENABLED);
			cout << "[CONFIG] LPMS_GAIT_TRACKING_ENABLED\n";
		}
		else {
			configData->setParameter(PRM_GAIT_TRACKING_ENABLED, SELECT_GAIT_TRACKING_DISABLED);
			cout << "[CONFIG] LPMS_GAIT_TRACKING_DISABLED\n";
		}
		*/
	break;	
	
	case GET_SENSOR_DATA:
		parseSensorData();
		
		return true; // <- Careful, in case of GET_DATA don't reset anything to prevent overflow in stream mode.
		// if ((lpmsStatus & LPMS_STREAM_MODE) != 0) return true;
	break;

	case GET_IMU_ID:
		if (fromBuffer(oneTx, &l)) {
			configData->setParameter(PRM_OPENMAT_ID, (int) l);
			imuId = l;
		}
	break;

	case GET_STATUS:
		if (fromBuffer(oneTx, &l)) lpmsStatus = l;
	break;

	case GET_FIELD_ESTIMATE:
		fromBuffer(oneTx, 0, &(configData->fieldRadius));
	break;
	
	case GET_FIRMWARE_VERSION:
		fromBuffer(oneTx, &i0, &i1, &i2);
		configData->firmwareVersionDig0 = i0;
		configData->firmwareVersionDig1 = i1;
		configData->firmwareVersionDig2 = i2;
				
		configData->firmwareVersion = static_cast<ostringstream*>(&(ostringstream() << i2))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i1))->str() + std::string(".") + static_cast<ostringstream*>(&(ostringstream() << i0))->str();
	break;

	default:
		return false;
	break;
	}
	
	receiveReset();

	return true;
}

void LpemgIoInterface::clearRxBuffer(void)
{
    dataQueue = std::move(std::queue<unsigned char>{});
    oneTx.clear();
    receiveReset();
}

bool LpemgIoInterface::checkState(void)
{
	parseModbusByte();

	if (waitForAck == true && ackReceived == false) {	
		if (ackTimer.measure() > ACK_MAX_TIME) {
			if (resendI < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendI;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpemgIoInterface] ACK timeout error. Resending command: %d\n", resendI);
				
				return true;
			} else {
				currentState = IDLE_STATE;
				waitForAck = false;
				waitForData = false;				
				ackReceived = false;
				ackTimeout = 0;
				isOpen = false;
			
				LOGV("[LpemgIoInterface] ACK timeout error. Resetting send queue.\n");
			
				if (ifs.is_open() == true) ifs.close();
			
				return false;
			}
		}
	} 
	
	if (waitForData == true && dataReceived == false) {
		if (ackTimer.measure() > ACK_MAX_TIME) {
			/* if (resendI < MAX_COMMAND_RESEND) {
				ackTimer.reset();
				uploadTimer.reset();
				++resendI;
				sendModbusData(imuId, currentState, cLength, cBuffer);
				
				LOGV("[LpemgIoInterface] ACK timeout error. Resending command: %d\n", resendI);
				
				return true;
			} else { */		
				currentState = IDLE_STATE;
				waitForAck = false;
				waitForData = false;
				ackReceived = false;
				ackTimeout = 0;
				isOpen = false;
			
				LOGV("[LpemgIoInterface] ACK timeout error. Resetting send queue.\n");
			
				return false;
			// }
		}
	}

	if (waitForAck == true && ackReceived == true) {
		switch (currentState) {
			case UPDATE_FIRMWARE:
				handleFirmwareFrame();
			break;
			
			case UPDATE_IAP:
				handleIAPFrame();
			break;
			
			default:
				receiveReset();
			break;
		}
	}
	
	return true;
}

bool LpemgIoInterface::startUploadFirmware(std::string fn)
{
	bool f = false;
	long long l;
	unsigned long r;
	
	if (configData->deviceType == DEVICE_LPMS_BLE) {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH_LPMS_BLE;
	} else {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH;
	}
	
	if (ifs.is_open() == true) ifs.close();

	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpemgIoInterface] Firmware file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpemgIoInterface] Could not open firmware file %s\n", fn.c_str());
		f = false;
	
		return f;
	}
	
	ifs.seekg(0, ios::end);
	l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpemgIoInterface] Firmware filesize: %lld\n", l);
	
	firmwarePages = l / firmwarePageSize;
	r = l % firmwarePageSize;
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpemgIoInterface] Firmware pages: %lld\n", firmwarePages);
	LOGV("[LpemgIoInterface] Firmware remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpemgIoInterface] Firmware packets to be sent: %lld\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_FIRMWARE, 4, (unsigned char *)cBuffer);
	
	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;	
	uploadTimer.reset();
	
	return f;
}

bool LpemgIoInterface::checkUploadTimeout(void)
{
	if (currentState == UPDATE_FIRMWARE) {
		if (uploadTimer.measure() > MAX_UPLOAD_TIME) {	
			currentState = IDLE_STATE;
		
			waitForAck = false;
			ackReceived = false;	

			ifs.close();
			
			LOGV("[LpemgIoInterface] Firmware upload failed. Please reconnect sensor and retry.\n");

			return false;
		}
	}

	return true;
}

bool LpemgIoInterface::handleFirmwareFrame(void)
{
	uploadTimer.reset();
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
		
		ifs.close();		
		
		return false;
	}

	if (ifs.eof() == true || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;	

		ifs.close();
		
		LOGV("[LpemgIoInterface] Firmware upload finished. Now writing to flash. Please DO NOT detach the power from the device for 15s.\n");
		
		return true;
	}

	LOGV("[LpemgIoInterface] Firmware sending packet %d\n", pCount);
	++pCount;

	for (int i=0; i < firmwarePageSize; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, firmwarePageSize);
	cLength = firmwarePageSize;
	sendModbusData(imuId, UPDATE_FIRMWARE, firmwarePageSize, (unsigned char *)cBuffer);

	ackTimeout = 0;
	ackTimer.reset();
	dataTimeout = 0;
	resendI = 0;

	currentState = UPDATE_FIRMWARE;
	waitForAck = true;
	ackReceived = false;	
	
	return true;
}

bool LpemgIoInterface::startUploadIap(std::string fn)
{
	LOGV("[LpemgIoInterface] startUploadIap\n");
	bool f = false;

	if (configData->deviceType == DEVICE_LPMS_BLE) {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH_LPMS_BLE;
	} else {
		firmwarePageSize = FIRMWARE_PACKET_LENGTH;
	}	
	
	if (ifs.is_open() == true) ifs.close();
	
	ifs.clear();	
	ifs.open(fn.c_str(), std::ios::binary);
		
	if (ifs.is_open() == true) {
		f = true;
		LOGV("[LpemgIoInterface] IAP file %s opened.\n", fn.c_str());
	} else {
		LOGV("[LpemgIoInterface] Could not open IAP file %s\n", fn.c_str());	
		f = false;
	}
	
	ifs.seekg(0, ios::end);
	long long l = ifs.tellg();
	ifs.seekg(0, ios::beg);
	LOGV("[LpemgIoInterface] IAP filesize: %lld\n", l);
	
	firmwarePages = l / firmwarePageSize;
	unsigned long r = (long) (l % firmwarePageSize);
	
	if (r > 0) ++firmwarePages;
	
	LOGV("[LpemgIoInterface] IAP pages: %lld\n", firmwarePages);
	LOGV("[LpemgIoInterface] IAP remainder: %d\n", r);
	
	cBuffer[0] = firmwarePages & 0xff;
	cBuffer[1] = (firmwarePages >> 8) & 0xff;
	cBuffer[2] = (firmwarePages >> 16) & 0xff;
	cBuffer[3] = (firmwarePages >> 24) & 0xff;
	
	LOGV("[LpemgIoInterface] IAP packets to be sent: %lld\n", firmwarePages);
	
	cLength = 4;
	sendModbusData(imuId, UPDATE_IAP, 4, (unsigned char *)cBuffer);	
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;
	ackTimeout = 0;
	ackTimer.reset();

	pCount = 0;
	uploadTimer.reset();

	LOGV("[LpemgIoInterface] startUploadIap2\n");
	return f;
}	

bool LpemgIoInterface::handleIAPFrame(void)
{
//	LOGV("[LpemgIoInterface] handleIAPFrame\n");
	uploadTimer.reset();	
	
	if (ifs.is_open() == false) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;
			
		ifs.close();	
		
		return false;
	}

	if (ifs.eof() == true || firmwarePages == pCount) {
		currentState = IDLE_STATE;
		waitForAck = false;
		ackReceived = false;

		ifs.close();
		
		LOGV("[LpemgIoInterface] IAP upload finished\n");
		
		return true;		
	}
	
	LOGV("[LpemgIoInterface] Sending IAP packet %d\n", pCount);
	++pCount;		

	for (int i=0; i < firmwarePageSize; i++) cBuffer[i] = (char) 0xff;
	ifs.read((char *)cBuffer, firmwarePageSize);
	cLength = firmwarePageSize;
	sendModbusData(imuId, UPDATE_IAP, firmwarePageSize, (unsigned char *)cBuffer);
	
	ackTimeout = 0;
	dataTimeout = 0;
	resendI = 0;
	ackTimer.reset();
	
	currentState = UPDATE_IAP;
	waitForAck = true;
	ackReceived = false;		

	return true;
}

bool LpemgIoInterface::selectData(long p) 
{
	boost::uint32_t v = 0;
	
	if ((p & SELECT_LPMS_ACC_OUTPUT_ENABLED) != 0) {
		v |= LPMS_ACC_RAW_OUTPUT_ENABLED;
	} else {
		v &= ~LPMS_ACC_RAW_OUTPUT_ENABLED;
	}
	
    if ((p & SELECT_LPMS_GYRO_OUTPUT_ENABLED) != 0) {
        v |= LPMS_GYR_RAW_OUTPUT_ENABLED;
    }
    else {
        v &= ~LPMS_GYR_RAW_OUTPUT_ENABLED;
    }
	
    if ((p & SELECT_LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
        v |= LPMS_PRESSURE_OUTPUT_ENABLED;
    }
    else {
        v &= ~LPMS_PRESSURE_OUTPUT_ENABLED;
    }

    if ((p & SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
        v |= LPMS_ALTITUDE_OUTPUT_ENABLED;
    }
    else {
        v &= ~LPMS_ALTITUDE_OUTPUT_ENABLED;
    }	
	
	printf("select data\n");
	
	modbusSetInt32(SET_TRANSMIT_DATA, v);
	
	return true;
}

int LpemgIoInterface::getMode(void)
{
	if ((lpmsStatus & LPMS_COMMAND_MODE) != 0) {
			return SELECT_LPMS_MODE_COMMAND;
		} else if ((lpmsStatus & LPMS_STREAM_MODE) != 0) {
			return SELECT_LPMS_MODE_STREAM;
#if 0
		} else if ((lpmsStatus & LPMS_SLEEP_MODE) != 0) {
			return SELECT_LPMS_MODE_SLEEP;
#endif
		}

	return currentMode;
}

bool LpemgIoInterface::setCommandMode(void) 
{	
	bool r;

	lpmsStatus |= LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	//lpmsStatus &= ~LPMS_SLEEP_MODE;
	
	r = modbusSetNone(GOTO_COMMAND_MODE);
	
	return r;
}

bool LpemgIoInterface::setStreamMode(void) 
{
	bool r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus |= LPMS_STREAM_MODE;
	//lpmsStatus &= ~LPMS_SLEEP_MODE;	
	r = modbusSetNone(GOTO_STREAM_MODE);

	return r;
}
	
bool LpemgIoInterface::setSleepMode(void) 
{	
	bool r;

	lpmsStatus &= ~LPMS_COMMAND_MODE;
	lpmsStatus &= ~LPMS_STREAM_MODE;
	//lpmsStatus |= LPMS_SLEEP_MODE;	
	
	r = modbusSetNone(GOTO_SLEEP_MODE);

	return r;
}

bool LpemgIoInterface::restoreFactoryValue(void)
{
	return modbusSetNone(RESTORE_FACTORY_VALUE);
}

bool LpemgIoInterface::setSelfTest(long v)
{
	return modbusSetInt32(SELF_TEST, v);
}

bool LpemgIoInterface::setImuId(long v)
{
    return modbusSetInt32(SET_IMU_ID, v);
}

bool LpemgIoInterface::setStreamFrequency(long v)
{
    return modbusSetInt32(SET_STREAM_FREQ, v);
}

bool LpemgIoInterface::getConfig(void)
{
	latencyTimer.reset();

	return modbusGet(GET_CONFIG);
}

bool LpemgIoInterface::getImuId(void)
{
	return modbusGet(GET_IMU_ID);
}

bool LpemgIoInterface::getStatus(void)
{
	return modbusGet(GET_STATUS);
}

bool LpemgIoInterface::getSensorData(void)
{
	return modbusGet(GET_SENSOR_DATA);
}

bool LpemgIoInterface::isCalibrating(void)
{	
	return false;
}

bool LpemgIoInterface::isError(void)
{	
	return false;
}

bool LpemgIoInterface::writeRegisters(void)
{
	return modbusSetNone(WRITE_REGISTERS);
}

bool LpemgIoInterface::getFirmwareVersion(void)
{
	return modbusGet(GET_FIRMWARE_VERSION);
}

CalibrationData *LpemgIoInterface::getConfigData(void)
{
	return configData;
}

bool LpemgIoInterface::getUploadProgress(int *p)
{
	if (firmwarePages > 0) {
		*p = pCount * 100 / (int) firmwarePages;
	} else {
		*p = 0;
	}

	if (checkUploadTimeout() == false) return false;
	
	return true;
}

float LpemgIoInterface::getLatestLatency(void)
{
	return latestLatency;
}

bool LpemgIoInterface::setGyrAlignment(LpMatrix3x3f m)
{
	return modbusSetMatrix3x3f(SET_GYR_ALIGN_MATRIX, m);
}

bool LpemgIoInterface::setGyrAlignmentBias(LpVector3f v)
{
	return modbusSetVector3f(SET_GYR_ALIGN_BIAS, v);
}

bool LpemgIoInterface::getGyrAlignment(void)
{
	return modbusGet(GET_GYR_ALIGN_MATRIX);
}

bool LpemgIoInterface::getGyrAlignmentBias(void)
{
	return modbusGet(GET_GYR_ALIGN_BIAS);
}

long LpemgIoInterface::getConfigReg(void) {
	return configReg;
}

bool LpemgIoInterface::setTimestamp(float v)
{
	boost::uint32_t i;
	boost::uint32_t m = 0xff;
	unsigned char buffer[4];		

	i = conFtoI(v);	

	for (int j=0; j<4; j++) {
		buffer[j] = (unsigned char) (i & 0xff);
		i = i / 256;
	}

#if 0
	sendModbusData(imuId, RESET_TIMESTAMP, 4, buffer);
#endif

	return true;
}

bool LpemgIoInterface::setLpBusDataMode(int v)
{
	return modbusSetInt32(SET_LPBUS_DATA_MODE, v);
}

bool LpemgIoInterface::armTimestampReset(int v)
{
	printf("Arm timestamp reset\n");
	return modbusSetInt32(SET_ARM_HARDWARE_TIMESTAMP_RESET, v);
}
