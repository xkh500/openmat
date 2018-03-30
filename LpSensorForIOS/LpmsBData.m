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

#import "LpmsBData.h"

@implementation LpmsBData

-(instancetype)initWithLpmsBData {
    if (self = [super init]) {
        [self reset];
    }

    return self;
}

-(instancetype)initWithLpmsBData:(LpmsBData *)d {
    if (self = [super init]) {        
        _imuId = d ->_imuId;
        _timestamp = d->_timestamp;
        _frameNumber = d->_frameNumber;
        _pressure = d->_pressure;
        _altitude = d->_altitude;
        _batteryLevel = d->_batteryLevel;
        _batteryVoltage = d->_batteryVoltage;
        _temperature = d->_temperature;
        _chargingStatus = d->_chargingStatus;
        _heave = d->_heave;
                
        for (int i = 0; i<3; i++) _gyr[i] = d->_gyr[i];        
        for (int i = 0; i<3; i++) _acc[i] = d->_acc[i];        
        for (int i = 0; i<3; i++) _mag[i] = d->_mag[i];        
        for (int i = 0; i<3; i++) _angVel[i] = d->_angVel[i];        
        for (int i = 0; i<4; i++) _quat[i] = d->_quat[i];        
        for (int i = 0; i<3; i++) _euler[i] = d->_euler[i];       
        for (int i = 0; i<3; i++) _linAcc[i] = d->_linAcc[i];
    }

    return self;
}

-(void)reset {
    _imuId = 0;
    _timestamp = 0;
    _frameNumber = 0;
    _pressure = 0;
    _altitude = 0;
    _batteryLevel = 0;
    _batteryVoltage = 0;
    _temperature = 0;
    _chargingStatus = 0;
    _heave = 0;
    
    for (int i = 0; i<3; i++) {
        _gyr[i] = 0;
    }

    for (int i = 0; i<3; i++) {
        _acc[i] = 0;
    }
    
    for (int i = 0; i<3; i++) {
        _mag[i] = 0;
    }

    for (int i = 0; i<3; i++) {
        _angVel[i] = 0;
    }
    
    for (int i = 0; i<4; i++) {
        _quat[i] = 0;
    }

    for (int i = 0; i<3; i++) {
        _euler[i] = 0;
    }
    
    for (int i = 0; i<3; i++) {
        _linAcc[i] = 0;
    }
}

-(void)encodeWithCoder:(NSCoder  *)Coder {
    [Coder encodeInt:_imuId forKey:@"ID"];
    [Coder encodeFloat:_timestamp forKey:@"timestamp"];
    [Coder encodeInt:_frameNumber forKey:@"frameNumber"];
    [Coder encodeFloat:_acc[0] forKey:@"acc[0]"];
    [Coder encodeFloat:_acc[1] forKey:@"acc[1]"];
    [Coder encodeFloat:_acc[2] forKey:@"acc[2]"];
}

@end