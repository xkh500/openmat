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

#import "DataLogger.h"

int frameNumber = 0;
BOOL isLoggingStarted = false;
NSString *string;
NSString *outputFilename = @"";
NSString *statusMesg = @"Logging Stopped";
NSOutputStream *outputStream;
NSFileHandle *filehandle;
NSString *FilePath;
NSFileHandle *handle;
BOOL Test = true;

@implementation DataLogger

-(instancetype)initWithDataLogger {
    if (self = [super init]) {
        
    }
    return self;
}

- (Boolean) startLogging {
    Boolean status = false;
    @try {
        NSDate *date = [NSDate date];
        NSDateFormatter *sdf = [[NSDateFormatter alloc]init];
        [sdf setDateFormat:@"YYYY-MM-dd HH:mm:ss"];
        NSString *dataTime = [sdf stringFromDate:date];
       
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsDirectory = [paths objectAtIndex:0];
        
        NSString *path = [NSString stringWithFormat:@"%@%@",documentsDirectory, @"/LpLogger"];
        
        if ([[NSFileManager defaultManager]fileExistsAtPath:path]) {
            NSLog(@"文件夹已存在");
        } else {
            [[NSFileManager defaultManager]createDirectoryAtPath:path withIntermediateDirectories:YES attributes:nil error:nil];
            NSLog(@"文件夹创建成功");
        }
        
        NSString *testPath = [NSString stringWithFormat:@"%@%@%@",@"DataLog",dataTime,@".csv"];
        NSLog(@"Datalogger :%@",testPath);

        FilePath = [path stringByAppendingPathComponent:testPath];
        NSFileManager *fileManager = [NSFileManager defaultManager];
        [fileManager createFileAtPath:FilePath contents:[string dataUsingEncoding:NSUTF8StringEncoding] attributes:nil];
        outputStream = [[NSOutputStream alloc]initToFileAtPath:FilePath append:YES];
        
        frameNumber = 0;
        
        NSArray *array = [[NSArray alloc]initWithObjects:@"SensorId, TimeStamp (s), FrameNumber, AccX (g), AccY (g), AccZ (g),GyroX (deg/s), GyroY (deg/s), GyroZ (deg/s), MagX (uT), MagY (uT), MagZ (uT), EulerX(deg) , EulerY(deg) , EulerZ(deg) , QuatW, QuatX, QuatY, QuatZ, LinAccX(g) , LinAccY(g) , LinAccZ(g) ,Pressure(kPa), Altitude(m),Temperature(degC),BatteryLevel(%)",@"\n", nil];
        
        [array writeToFile:FilePath atomically:YES];
        
        [outputStream setDelegate:self];
        [outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        [outputStream open];
        NSLog(@"stream start");
        
        isLoggingStarted = true;
        outputFilename = FilePath;
        
        statusMesg = [NSString stringWithFormat:@"%@%@",@"Logging to ",outputFilename];
        
        status = true;
    } @catch (NSException *e) {
        NSLog(@"Exception %@",e);
    }
    return status;
}

-(Boolean)stopLogging{
    Boolean status = false;
    if (!isLoggingStarted) {
        statusMesg = @"Logging not started";
        return status;
    }
    @try {
        isLoggingStarted = false;
        @synchronized(outputStream) {
            [outputStream close];
            [outputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
            [handle closeFile];
            NSLog(@"stream stop");
        }
        
        statusMesg = @"Logging Stopped";
        status = true;
    } @catch (NSException *e) {
    }
  
    return status;
}

-(BOOL)isLogging{
    return isLoggingStarted;
}

-(Boolean)logLpmsData:(LpmsBData *)d{
    Boolean status = false;
    
    if (isLoggingStarted) {
        @try {
            @synchronized(handle) {
                if (Test == true) {
                    handle = [NSFileHandle fileHandleForWritingAtPath:FilePath];
                    Test = false;
                }
                
                [handle seekToEndOfFile];
                [handle writeData:[[NSString stringWithFormat:@"%d ,",d->_imuId] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,",d->_timestamp] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%d ,",frameNumber] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,",d->_acc[0],d->_acc[1],d->_acc[2]] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,",d->_gyr[0],d->_gyr[1],d->_gyr[2]] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,",d->_mag[0],d->_mag[1],d->_mag[2]] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,",d->_euler[0],d->_euler[1],d->_euler[2]] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,%f ,",d->_quat[0],d->_quat[1],d->_quat[2],d->_quat[3]] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,%f ,%f ,",d->_linAcc[0],d->_linAcc[1],d->_linAcc[2]] dataUsingEncoding:NSUTF8StringEncoding]];

                [handle writeData:[[NSString stringWithFormat:@"%f ,",d->_pressure] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,",d->_altitude] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,",d->_temperature] dataUsingEncoding:NSUTF8StringEncoding]];
                [handle writeData:[[NSString stringWithFormat:@"%f ,",d->_batteryLevel] dataUsingEncoding:NSUTF8StringEncoding]];
                
                [handle writeData:[[NSString stringWithFormat:@"\n"] dataUsingEncoding:NSUTF8StringEncoding]];
               
                frameNumber ++;
            }
            
            status = true;
        } @catch (NSException *e) {
        }
    }

    return status;
}

-(void)setStatusMesg:(NSString*)s {
    statusMesg = s;
}

@end
