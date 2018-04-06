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

#import "View.h"
#import "AppDelegate.h"
#import "LpmsB2.h"
#import "LpmsBData.h"

LpmsB2 *lpb;
LpmsBData *lpmsdata;

@interface View () {
}

@property(nonatomic,strong) UILabel *label,*status,*transmission,*range,*filter,*data;
@property (nonatomic,strong) UITextView *connection, *device, *firmware, *battery, *quaternion;
@property(nonatomic,strong) NSDecimalNumber *level, *voltage;
@property(nonatomic,strong) UISwitch *switcH;
@property(nonatomic,strong) NSTimer *timer;

@end

@implementation View

- (void)viewDidLoad {
    [super viewDidLoad];
    
    self.view.backgroundColor = [UIColor whiteColor];
    
    [self initConnection];
    [self initGui];
    
    _timer = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(updateData) userInfo:nil repeats:YES];
}

-(void)initGui {
    self.connection = [[UITextView alloc]initWithFrame:CGRectMake(20, 75, 500, 50)];
    self.connection.selectable = NO;
    self.connection.textColor = [UIColor blackColor];
    self.connection.font = [UIFont boldSystemFontOfSize:15];
    [self.view addSubview:self.connection];

    self.device = [[UITextView alloc] initWithFrame:CGRectMake(20, 100, 500, 50)];
    self.device.selectable = NO;
    self.device.font = [UIFont boldSystemFontOfSize:15];
    [self.view addSubview: self.device];
    self.firmware = [[UITextView alloc] initWithFrame:CGRectMake(20, 125, 500, 50)];
    self.firmware.selectable = NO;
    self.firmware.font = [UIFont boldSystemFontOfSize:15];
    [self.view addSubview: self.firmware];

    self.battery = [[UITextView alloc] initWithFrame:CGRectMake(20, 150, 500, 50)];
    self.battery.selectable = NO;
    self.battery.font = [UIFont boldSystemFontOfSize:15];
    [self.view addSubview: self.battery];
    
    self.quaternion = [[UITextView alloc] initWithFrame:CGRectMake(20, 175, 500, 50)];
    self.quaternion.selectable = NO;
    self.quaternion.font = [UIFont boldSystemFontOfSize:15];
    [self.view addSubview: self.quaternion];
}
    
-(void)initConnection {
    lpb = [[LpmsB2 alloc] init];
}

-(void)updateData {
    if ([lpb getConnectionStatus] == 1) {
        self.connection.text = [NSString stringWithFormat:@"%@",@"Connection: Connected"];
    } else if ([lpb getConnectionStatus] == 2){
        self.connection.text = [NSString stringWithFormat:@"%@",@"Connection: Connecting"];
    } else if([lpb getConnectionStatus] == 3){
        self.connection.text = [NSString stringWithFormat:@"%@",@"Connection: Disconnected"];
    }

    self.device.text = [NSString stringWithFormat:@"%@%@",@"Device ID: ", [lpb getDeviceName]];
    self.firmware.text = [NSString stringWithFormat:@"%@%@",@"Firmware version: ", [lpb getFirmwareInfo]];
    
    [self getBatteryData];
    self.battery.text = [NSString stringWithFormat:@"%@%@%@%@%@",@"Battery level: ", _level, @"%  ", _voltage, @"V"];

    if ([lpb hasNewData] > 0) {
        LpmsBData* lpd = [lpb getLpmsBData];
        self.quaternion.text = [NSString stringWithFormat:@"%@%f, %f, %f, %f",@"Quaternion: ", lpd->_quat[0], lpd->_quat[1], lpd->_quat[2], lpd->_quat[3]];
    }
}

-(void)getBatteryData {
    NSDecimalNumberHandler *Numerhandler = [NSDecimalNumberHandler decimalNumberHandlerWithRoundingMode:NSRoundBankers scale:1 raiseOnExactness:NO raiseOnOverflow:NO raiseOnUnderflow:NO raiseOnDivideByZero:YES];
    
    NSDecimalNumber *test = [NSDecimalNumber decimalNumberWithString:[NSString stringWithFormat:@"%f",[lpb getBatteryLevel]]];
    _level = [test decimalNumberByRoundingAccordingToBehavior:Numerhandler];
    
    test = [NSDecimalNumber decimalNumberWithString:[NSString stringWithFormat:@"%f",[lpb getBatteryVoltage]]];
    _voltage = [test decimalNumberByRoundingAccordingToBehavior:Numerhandler];
}

@end
