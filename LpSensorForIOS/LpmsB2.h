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

#import <Foundation/Foundation.h>

// expected a type
@class CBPeripheral;
@class CBCentralManager;

typedef void (^sentValue)(NSString *str);

// 声明,类无法直接使用 NSThread
@interface LpmsB2 : NSObject {
    // 属性,放在这里会reset 
}

-(void)parse:(NSData *)data;
-(id)getLpmsBData;
-(void)settimeStamp:(sentValue)_sentBlock;
-(void)Reset:(float)m;
-(BOOL)connect:(CBCentralManager*)central Address:(CBPeripheral*)address;
-(NSString*)getSerialNumber;
-(NSString*)getDeviceName;
-(NSString*)getFirmwareInfo;
-(BOOL)startLogging;
-(BOOL)stopLogging;
-(BOOL)disconnect;
-(void)setCommandMode;
-(void)_getConfig;
-(void)_getFirmwareInfo;
-(void)setStreamingMode;
-(void)_getSerialNumber;
-(void)SetParameters;
-(void)_getDeviceName;
-(int)getConnectionStatus;
-(float)getBatteryLevel;
-(float)getBatteryVoltage;
-(int)getStreamFrequency;
-(void)lpbusSetNone:(int )command;
-(void)setStreamFrequency:(int )freq;
-(int)getGyroRange;
-(void)setGyroRange:(int)range;
-(int)getAccRange;
-(void)setAccRange:(int)range;
-(int)getMagRange;
-(void)setMagRange:(int)range;

@end
