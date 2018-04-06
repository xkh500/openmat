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

#import "SecondViewController.h"
#import "ViewController.h"
#import "AppDelegate.h"
#import "LpmsB2.h"
#import "LpmsBData.h"
#import "DataLogger.h"

@interface SecondViewController ()<UITextViewDelegate>

@property(nonatomic,strong)UITextView *textview;
@property(nonatomic,strong)UITextView *textview1;
@property(nonatomic,strong)UITextView *textview2;

@end

LpmsB2 *lpmsb;
LpmsBData *lpdata;
float test;
BOOL testt = true;
DataLogger *datalog;
UIButton *saveB,*stopB,*maxB,*testb,*tb;

int Sensor10hz = 0.1;
int Sensor25hz = 0.04;
int Sensor50hz = 0.02;
int Sensor100hz = 0.01;

NSTimer *timer;

@implementation SecondViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    self.view.backgroundColor = [UIColor whiteColor];
    self.textview.delegate = self;
    
    self.textview = [[UITextView alloc]initWithFrame:CGRectMake(70, 60, 300, 50)];
    self.textview1 = [[UITextView alloc]initWithFrame:CGRectMake(170, 220, 130, 350)];
    self.textview2 = [[UITextView alloc]initWithFrame:CGRectMake(70, 180, 300, 50)];
    
    [self startButton];
    [self stopButton];
    [self Button];
    
    [self ButtonT];
    [self TB];
    [self Timer];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

-(void)TestView1 {
    self.textview.textAlignment = NSTextAlignmentCenter;
    self.textview.font = [UIFont systemFontOfSize:20.0f];
    self.textview.editable = NO;
    self.textview.selectable = NO;
    self.textview.backgroundColor = [UIColor whiteColor];
    [self.view addSubview:self.textview];
}

-(void)TestView {
    if (testt == true) {
        lpmsb = [[LpmsB2 alloc]init];
        lpdata = [[LpmsBData alloc]init];
        testt = false;
    }

    [lpmsb settimeStamp:^(NSString *testtime) {
        self.textview2.text = testtime;
    }];
    
    if ([lpmsb getConnectionStatus] == 1) {
        if (lpdata != nil) {
            self.textview.text = [NSString stringWithFormat:@"%@%f",@"TimeStamp:",lpdata->_timestamp];
            self.textview1.text = [NSString stringWithFormat:@"%@%@%f%@%f%@%f%@%@%@%f%@%f%@%f%@%@%@%f%@%f%@%f",@"Acc:",@"\n",lpdata->_acc[0],@"\n",lpdata->_acc[1],@"\n",lpdata ->_acc[2],@"\n",@"gyr:",@"\n",lpdata->_gyr[0],@"\n",lpdata->_gyr[1],@"\n",lpdata ->_gyr[2],@"\n",@"mag:",@"\n",lpdata->_mag[0],@"\n",lpdata->_mag[1],@"\n",lpdata ->_mag[2]];
        }
    }

    self.textview.textAlignment = NSTextAlignmentCenter;
    self.textview1.textAlignment = NSTextAlignmentCenter;
    self.textview2.textAlignment = NSTextAlignmentCenter;

    self.textview.font = [UIFont systemFontOfSize:20.0f];
    self.textview1.font = [UIFont systemFontOfSize:20.0f];
    self.textview2.font = [UIFont systemFontOfSize:20.0f];

    self.textview.editable = NO;
    self.textview.selectable = NO;
    self.textview.backgroundColor = [UIColor whiteColor];

    self.textview1.editable = NO;
    self.textview1.selectable = NO;
    self.textview1.backgroundColor = [UIColor whiteColor];


    self.textview2.editable = NO;
    self.textview2.selectable = NO;
    self.textview2.backgroundColor = [UIColor whiteColor];

    NSRange range = NSMakeRange([self.textview.text length]-1, 1);
    [self.textview scrollRangeToVisible:range];

    [self.view addSubview:self.textview];
    [self.view addSubview:self.textview1];
    [self.view addSubview:self.textview2];
}

-(void)startButton {
    saveB = [UIButton buttonWithType:UIButtonTypeCustom];
    saveB.backgroundColor = [UIColor whiteColor];

    saveB.frame = CGRectMake(30, 600, 80, 40);
    [saveB setTitle:@"Save" forState:UIControlStateNormal];
    [saveB setTitleColor:[UIColor blueColor] forState:UIControlStateNormal];
    [saveB addTarget:self action:@selector(SaveB) forControlEvents:UIControlEventTouchUpInside];

    NSLog(@"saveButton");
    [self.view addSubview:saveB];
}

-(void)ButtonT {
    testb = [UIButton buttonWithType:UIButtonTypeCustom];
    testb.backgroundColor = [UIColor blackColor];
    testb.frame = CGRectMake(30, 400, 80, 40);
    [testb setTitle:@"testb" forState:UIControlStateNormal];
    [testb setTitleColor:[UIColor blueColor] forState:UIControlStateNormal];
    [testb addTarget:self action:@selector(testb1) forControlEvents:UIControlEventTouchUpInside];
    
    NSLog(@"testb");
    [self.view addSubview:testb];
}

-(void)TB {
    tb = [UIButton buttonWithType:UIButtonTypeCustom];
    tb.backgroundColor = [UIColor blackColor];
    tb.frame = CGRectMake(30, 500, 80, 40);
    [tb setTitle:@"tb" forState:UIControlStateNormal];
    [tb setTitleColor:[UIColor blueColor] forState:UIControlStateNormal];
    [tb addTarget:self action:@selector(tb) forControlEvents:UIControlEventTouchUpInside];
    NSLog(@"tb");
    [self.view addSubview:tb];
}

-(void)stopButton {
    stopB = [UIButton buttonWithType:UIButtonTypeCustom];
    stopB.backgroundColor = [UIColor whiteColor];
    stopB.frame = CGRectMake(250, 600, 80, 40);
    [stopB setTitle:@"Stop" forState:UIControlStateNormal];
    [stopB setTitleColor:[UIColor blueColor] forState:UIControlStateNormal];
    [stopB addTarget:self action:@selector(StopB) forControlEvents:UIControlEventTouchUpInside];
    NSLog(@"stopButton");
    [self.view addSubview:stopB];
}

-(void)Button {
    maxB = [UIButton buttonWithType:UIButtonTypeCustom];
    maxB.backgroundColor = [UIColor whiteColor];
    maxB.frame = CGRectMake(120, 600, 80, 40);
    [maxB setTitle:@"Reset" forState:UIControlStateNormal];
    [maxB setTitleColor:[UIColor blueColor] forState:UIControlStateNormal];
    [maxB addTarget:self action:@selector(Reset) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:maxB];
}

-(void)Reset {
    NSLog(@"Reset");
    [lpmsb setStreamingMode];
}

-(void)SaveB {
    datalog = [[DataLogger alloc]init];
    [datalog startLogging];
    NSLog(@"Save");
}

-(void)testb1 {
    [lpmsb setCommandMode];
}

-(void)tb {
    [lpmsb lpbusSetNone:9];
}

-(void)StopB {
    datalog = [[DataLogger alloc]init];
    [datalog stopLogging];
    NSLog(@"Stop");
}

-(void)Timer {
    timer = [NSTimer scheduledTimerWithTimeInterval:Sensor100hz target:self selector:@selector(TestView) userInfo:nil repeats:YES];
}

-(void)Cancel {
    [timer invalidate];
}

-(id)initWithTimeData:(float)timedata {
    if (self =[super init]) {
        _timeData = [NSString stringWithFormat:@"%f",timedata];
    }

    return self;
}

@end
