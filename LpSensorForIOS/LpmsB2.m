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

#import "LpmsB2.h"
#import "LpmsBData.h"
#import "SecondViewController.h"
#import "ViewController.h"
#import "DataLogger.h"
#import "View.h"

#import <CoreBluetooth/CoreBluetooth.h>

#define kServiceUUID @"0000180F-0000-1000-8000-00805F9B34FB" //服务的UUID
#define kCharacteristicUUID @"00002A19-0000-1000-8000-00805F9B34FB" //特征的UUID

@interface LpmsB2 ()<CBCentralManagerDelegate,CBPeripheralDelegate>

@property (strong,nonatomic) CBCentralManager *centralManager;//中心设备管理器
@property (strong,nonatomic) NSMutableArray *peripherals;//连接的外围设备

@end

// Stream frequency enable bits
const static int LPMS_STREAM_FREQ_5HZ = 5;
const static int LPMS_STREAM_FREQ_10HZ = 10;
const static int LPMS_STREAM_FREQ_25HZ = 25;
const static int LPMS_STREAM_FREQ_50HZ = 50;
const static int LPMS_STREAM_FREQ_100HZ = 100;
const static int LPMS_STREAM_FREQ_200HZ = 200;
const static int LPMS_STREAM_FREQ_400HZ = 400;
const static int LPMS_FILTER_GYR = 0;
const static int LPMS_FILTER_GYR_ACC = 1;
const static int LPMS_FILTER_GYR_ACC_MAG = 2;
const static int LPMS_FILTER_MADGWICK_GYR_ACC = 3;
const static int LPMS_FILTER_MADGWICK_GYR_ACC_MAG = 4;

// Gyro Range
const static int LPMS_GYR_RANGE_125DPS  = 125;
const static int LPMS_GYR_RANGE_245DPS  = 245;
const static int LPMS_GYR_RANGE_500DPS  = 500;
const static int LPMS_GYR_RANGE_1000DPS  = 1000;
const static int LPMS_GYR_RANGE_2000DPS  = 2000;

// Acc Range
const static int LPMS_ACC_RANGE_2G = 2;
const static int LPMS_ACC_RANGE_4G = 4;
const static int LPMS_ACC_RANGE_8G = 8;
const static int LPMS_ACC_RANGE_16G = 16;

// Mag Range
const static int LPMS_MAG_RANGE_4GAUSS = 4;
const static int LPMS_MAG_RANGE_8GAUSS = 8;
const static int LPMS_MAG_RANGE_12GAUSS =12;
const static int LPMS_MAG_RANGE_16GAUSS =16;

// 10是以毫秒为单位，这里是秒
const static int PARAMETER_SET_DELAY =0.05f;

// Connection status
const static int SENSOR_STATUS_CONNECTED = 1;
const static int SENSOR_STATUS_CONNECTING = 2;
const static int SENSOR_STATUS_DISCONNECTED = 3;

// Offset mode
const static int  LPMS_OFFSET_MODE_OBJECT = 0;
const static int  LPMS_OFFSET_MODE_HEADING = 1;
const static int  LPMS_OFFSET_MODE_ALIGNMENT = 2;

NSString *TAG = @"LpmsB2";
NSUUID *MY_UUID_INSECURE ; // = UUID.fromString(@"00001101-0000-1000-8000-00805F9B34FB");

const int PACKET_ADDRESS0 	= 0;
const int PACKET_ADDRESS1 	= 1;
const int PACKET_FUNCTION0 	= 2;
const int PACKET_FUNCTION1 	= 3;
const int PACKET_LENGTH0 	= 4;
const int PACKET_LENGTH1 	= 5;
const int PACKET_RAW_DATA 	= 6;
const int PACKET_LRC_CHECK0 = 7;
const int PACKET_LRC_CHECK1 = 8;
const int PACKET_END 		= 9;
const int PACKET_TEST       = 10;
int rxState = PACKET_TEST;
// int rxState = PACKET_END;

// Command Registers
const int REPLY_ACK 			= 0;
const int REPLY_NACK 			= 1;
const int UPDATE_FIRMWARE 		= 2;
const int UPDATE_IAP 			= 3;
const int GET_CONFIG 			= 4;
const int GET_STATUS 			= 5;
const int GOTO_COMMAND_MODE 	= 6;
const int GOTO_STREAM_MODE 		= 7;
const int GET_SENSOR_DATA 		= 9;
const int SET_TRANSMIT_DATA 	= 10;
const int SET_STREAM_FREQ 		= 11;

// Register value save and reset
int WRITE_REGISTERS    	    = 15;
int RESTORE_FACTORY_VALUE   = 16;

// Reference setting and offset reset
int RESET_REFERENCE  	        = 17;
int SET_ORIENTATION_OFFSET      = 18;
int RESET_ORIENTATION_OFFSET    = 82;

// IMU ID setting
int SET_IMU_ID 		    	= 20;
const int GET_IMU_ID        = 21;

// Gyroscope settings
int START_GYR_CALIBRA 	    = 22;
int ENABLE_GYR_AUTOCAL   	= 23;
int ENABLE_GYR_THRES 		= 24;
int SET_GYR_RANGE    		= 25;
const int GET_GYR_RANGE     = 26;

// Accelerometer settings
int SET_ACC_BIAS     		= 27;
int GET_ACC_BIAS     		= 28;
int SET_ACC_ALIGN_MATRIX 	= 29;
int GET_ACC_ALIGN_MATRIX 	= 30;
int SET_ACC_RANGE    		= 31;
const int GET_ACC_RANGE     = 32;
int SET_GYR_ALIGN_BIAS   	= 48;
int GET_GYR_ALIGN_BIAS   	= 49;
int SET_GYR_ALIGN_MATRIX 	= 50;
int GET_GYR_ALIGN_MATRIX 	= 51;

// Magnetometer settings
int SET_MAG_RANGE    		    = 33;
const int GET_MAG_RANGE    		= 34;
int SET_HARD_IRON_OFFSET 	    = 35;
int GET_HARD_IRON_OFFSET 	    = 36;
int SET_SOFT_IRON_MATRIX 	    = 37;
int GET_SOFT_IRON_MATRIX 	    = 38;
int SET_FIELD_ESTIMATE   	    = 39;
int GET_FIELD_ESTIMATE   	    = 40;
int SET_MAG_ALIGNMENT_MATRIX    = 76;
int SET_MAG_ALIGNMENT_BIAS      = 77;
int SET_MAG_REFRENCE 		    = 78;
int GET_MAG_ALIGNMENT_MATRIX    = 79;
int GET_MAG_ALIGNMENT_BIAS      = 80;
int GET_MAG_REFERENCE		    = 81;

// Filter settings
int SET_FILTER_MODE  		    = 41;
const int GET_FILTER_MODE  		= 42;
int SET_FILTER_PRESET		    = 43;
int GET_FILTER_PRESET		    = 44;
int SET_LIN_ACC_COMP_MODE	    = 67;
int GET_LIN_ACC_COMP_MODE	    = 68;

// Status register contents
int SET_CENTRI_COMP_MODE 	    = 69;
int GET_CENTRI_COMP_MODE 	    = 70;
int SET_RAW_DATA_LP  		    = 60;
int GET_RAW_DATA_LP  		    = 61;
int SET_TIME_STAMP		        = 66;
int SET_LPBUS_DATA_MODE 	    = 75;
const int GET_FIRMWARE_VERSION 	= 47;
const int GET_BATTERY_LEVEL		= 87;
const int GET_BATTERY_VOLTAGE   = 88;
const int GET_CHARGING_STATUS   = 89;
const int GET_SERIAL_NUMBER     = 90;
const int GET_DEVICE_NAME       = 91;
const int GET_FIRMWARE_INFO     = 92;
const int START_SYNC            = 96;
const int STOP_SYNC             = 97;
const int GET_PING              = 98;
const int GET_TEMPERATURE       = 99;

// Configuration register contents
const static int LPMS_GYR_AUTOCAL_ENABLED               = 0x00000001 << 30;
const static int LPMS_LPBUS_DATA_MODE_16BIT_ENABLED     = 0x00000001 << 22;
const static int LPMS_LINACC_OUTPUT_ENABLED             = 0x00000001 << 21;
const static int LPMS_DYNAMIC_COVAR_ENABLED             = 0x00000001 << 20;
const static int LPMS_ALTITUDE_OUTPUT_ENABLED           = 0x00000001 << 19;
const static int LPMS_QUAT_OUTPUT_ENABLED               = 0x00000001 << 18;
const static int LPMS_EULER_OUTPUT_ENABLED              = 0x00000001 << 17;
const static int LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED   = 0x00000001 << 16;
const static int LPMS_GYR_CALIBRA_ENABLED               = 0x00000001 << 15;
const static int LPMS_HEAVEMOTION_OUTPUT_ENABLED        = 0x00000001 << 14;
const static int LPMS_TEMPERATURE_OUTPUT_ENABLED        = 0x00000001 << 13;
const static int LPMS_GYR_RAW_OUTPUT_ENABLED            = 0x00000001 << 12;
const static int LPMS_ACC_RAW_OUTPUT_ENABLED            = 0x00000001 << 11;
const static int LPMS_MAG_RAW_OUTPUT_ENABLED            = 0x00000001 << 10;
const static int LPMS_PRESSURE_OUTPUT_ENABLED           = 0x00000001 << 9;

int LPMS_STREAM_FREQ_5HZ_ENABLED      = 0x00000000;
int LPMS_STREAM_FREQ_10HZ_ENABLED     = 0x00000001;
int LPMS_STREAM_FREQ_25HZ_ENABLED     = 0x00000002;
int LPMS_STREAM_FREQ_50HZ_ENABLED     = 0x00000003;
int LPMS_STREAM_FREQ_100HZ_ENABLED    = 0x00000004;
int LPMS_STREAM_FREQ_200HZ_ENABLED    = 0x00000005;
int LPMS_STREAM_FREQ_400HZ_ENABLED    = 0x00000006;
int LPMS_STREAM_FREQ_MASK             = 0x00000007;

int MAX_BUFFER = 512;
int DATA_QUEUE_SIZE = 64;

// Status
int connectionStatus = SENSOR_STATUS_DISCONNECTED;

// Protocol parsing related
Byte rxBuffer[512];
Byte rawTxData[512];
Byte txBuffer[512];
Byte rawRxBuffer[512];
int currentAddress = 0;
int currentFunction = 0;
int currentLength = 0;
int rxIndex = 0;
Byte *b = 0;
int lrcCheck = 0;
int nBytes = 0;
BOOL waitForAck = false;
BOOL waitForData = false;
Byte inBytes[2];

// Settings related
int imuId = 0;
int gyrRange = 0;
int accRange = 0;
int magRange = 0;
int streamingFrequency = 0;
int filterMode = 0;
BOOL isStreamMode = false;

int configurationRegister = 0;
BOOL configurationRegisterReady = false;
NSString *serialNumber = @"";
BOOL serialNumberReady = false;
NSString *deviceName = @"";
BOOL deviceNameReady = false;
NSString *firmwareInfo= @"";
BOOL firmwareInfoReady = false;
NSString *firmwareVersion;
float batteryLevel = 0.0f;
float batteryVoltage = 0.0f;

BOOL accEnable = false;
BOOL gyrEnable = false;
BOOL magEnable = false;
BOOL angularVelEnable = false;
BOOL quaternionEnable = false;
BOOL eulerAngleEnable = false;
BOOL linAccEnable = false;
BOOL pressureEnable = false;
BOOL altitudeEnable = false;
BOOL temperatureEnable = false;
BOOL heaveEnable = false;
BOOL sixteenBitDataEnable = false;
BOOL resetTimestampFlag = false;
BOOL readCharacteristicReady = false;
BOOL newDataFlag = false;

int frameCounter = 0;

LpmsBData *mLpmsBData;
DataLogger *dl;
float mT;
int vmajor,vminor,vbuild;
NSString *mAddress;

// Test
NSMutableArray *tableArray;
BOOL Connect = false;
float data;
BOOL a = true;
BOOL a16 = true;

// 丢包
float lose=0;
float dt;
float MaxDt;
BOOL Check = false;

// 全局变量
CBPeripheral *p;
CBCharacteristic *chara;
dispatch_queue_t queue;

@implementation LpmsB2 {
    // 部分使用成员变量
}

// If true, the sensor will be at command mode. And after that, you should
// use setAcquisitionParameters() to set some sensor param.
-(BOOL)connect:(CBCentralManager*)central Address:(CBPeripheral*)address {
    _centralManager=[[CBCentralManager alloc]initWithDelegate:self queue:nil];
    _centralManager = central;
    _centralManager.delegate = self;

    if (connectionStatus != SENSOR_STATUS_DISCONNECTED) {
        return false;
    }
    
    if (_centralManager == nil) {
        connectionStatus = SENSOR_STATUS_DISCONNECTED;
        return false;
    }

    p = address;

    [_centralManager stopScan];
    [self.centralManager connectPeripheral:p options:@{CBConnectPeripheralOptionNotifyOnConnectionKey:@YES}];
    
    return true;
}

// 数据解析后再运行这里
-(void)SetParameters{
    // NSLog(@" SetParameters    %@", [NSThread mainThread]);
    [self setCommandMode];

    if (waitForAck == false) {
        // bug, 死循环
        while (waitForAck && readCharacteristicReady) {
            [self setCommandMode];
            @try {
                [NSThread sleepForTimeInterval:1L];
            }
            @catch (NSException *exception) {
                NSLog(@"SetParameters error: %@",exception);
            }
        }
    }

    [self _getBatteryPercentage];
    [self _getBatteryVoltage];
    [self _getSensorSettings];
    [self setStreamingMode];
    connectionStatus = SENSOR_STATUS_CONNECTED;
}


// 连接到外围设备
-(void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)
peripheral{
    // NSLog(@"B2 连接外围设备成功!");
    // 设置外围设备的代理为当前视图控制器
    peripheral.delegate=self;
    
    // 外围设备开始寻找服务
    [peripheral discoverServices:@[[CBUUID UUIDWithString:kServiceUUID]]];
    
    // NSLog(@"what :,%@",peripheral);
}

// 连接外围设备失败
-(void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)
peripheral error:(NSError *)error {
    NSLog(@"B2 连接外围设备失败!");
    connectionStatus = SENSOR_STATUS_DISCONNECTED;
}

#pragma mark - CBPeripheral 代理方法
// 外围设备寻找到服务后
-(void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error{
    // NSLog(@"B2 已发现可用服务...");
    // [self writeToLog:@"已发现可用服务..."];
    if (error) {
        connectionStatus = SENSOR_STATUS_DISCONNECTED;
        NSLog(@"外围设备寻找服务过程中发生错误，错误信息：%@",error.localizedDescription);
        
        return;
    }

    // 遍历查找到的服务
    CBUUID *serviceUUID=[CBUUID UUIDWithString:kServiceUUID];
    CBUUID *characteristicUUID=[CBUUID UUIDWithString:kCharacteristicUUID];

    // NSLog(@"peripheral %@",peripheral.services);
    // NSLog(@"CBservice %@",service);
    for (CBService *service in peripheral.services) {
        if ([service.UUID isEqual:serviceUUID]) {
            // 外围设备查找指定服务中的特征,监听
            [peripheral discoverCharacteristics:@[characteristicUUID] forService:service];
        }
    }
}

// 外围设备寻找到特征后
-(void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)
service error:(NSError *)error {
    // NSLog(@"B2 已发现可用特征...");
    // [self writeToLog:@"已发现可用特征..."];
    if (error) {
        connectionStatus = SENSOR_STATUS_DISCONNECTED;
        NSLog(@"外围设备寻找特征过程中发生错误，错误信息：%@",error.localizedDescription);
        return;
    }

    // 遍历服务中的特征
    CBUUID *serviceUUID=[CBUUID UUIDWithString:kServiceUUID];
    CBUUID *characteristicUUID=[CBUUID UUIDWithString:kCharacteristicUUID];
    if ([service.UUID isEqual:serviceUUID]) {
        // 遍历出所需要的特征
        for (CBCharacteristic *characteristic in service.characteristics) {
            if ([characteristic.UUID isEqual:characteristicUUID]) {
                // 特征值属性
                // NSLog(@"characteristic :%@",characteristic);
                // 订阅通知
                [peripheral setNotifyValue:YES forCharacteristic:characteristic];
                p = peripheral;
                chara = characteristic;
                connectionStatus = SENSOR_STATUS_CONNECTING;
                // NSLog(@" 主线程    %@",[NSThread mainThread]);
                // 线程优先级
                // queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_LOW, 0);
                // 异步
                queue = dispatch_queue_create("GCD", DISPATCH_QUEUE_SERIAL);
                dispatch_async(queue, ^{
                    [self SetParameters];
                });
                mLpmsBData = [[LpmsBData alloc]init];
                tableArray = [[NSMutableArray alloc]initWithCapacity:0];
                dl = [[DataLogger alloc]init];
            }
        }
    }
}

// 特征值被更新后
-(void)peripheral:(CBPeripheral *)peripheral didUpdateNotificationStateForCharacteristic:
(CBCharacteristic *)characteristic error:(NSError *)error{
    
    if (error) {
        connectionStatus = SENSOR_STATUS_DISCONNECTED;
        NSLog(@"更新通知状态时发生错误，错误信息：%@",error.localizedDescription);
    }
    // 给特征值设置新的值
    CBUUID *characteristicUUID=[CBUUID UUIDWithString:kCharacteristicUUID];
    if ([characteristic.UUID isEqual:characteristicUUID]) {
        if (characteristic.isNotifying) {
            if (characteristic.properties == CBCharacteristicPropertyNotify) {
                NSLog(@"已订阅特征通知.");
                return;
            } else if (characteristic.properties == CBCharacteristicPropertyRead) {
                // 从外围设备读取新值,调用此方法会触发代理方法：
                // -(void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
                [peripheral readValueForCharacteristic:characteristic];
            }
        } else {
            NSLog(@"停止已停止.");
            
            // 取消连接
            [self.centralManager cancelPeripheralConnection:peripheral];
        }
        // NSLog(@"why");
        readCharacteristicReady = true;
        NSLog(@"B2 收到特征更新通知...");
    }
}

// 更新特征值后（调用readValueForCharacteristic:方法或者外围设备在订阅后更新特征值都会调用此代理方法）
// 读取数据
-(void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic
                                                                              *)characteristic error:(NSError *)error{
    // NSLog(@"didUpdateValueForCharacteristic");
    if (error) {
        connectionStatus = SENSOR_STATUS_DISCONNECTED;
        NSLog(@"更新特征值时发生错误，错误信息：%@",error.localizedDescription);
        return;
    }

    // 数据显示
    if (characteristic.value) {
        [self parse:characteristic.value];
        
        // NSLog(@"characteristic : %@",characteristic.value);
    } else {
        NSLog(@"未发现特征值.");        
    }

}

-(BOOL)disconnect{
    BOOL res = false;

    if (connectionStatus != SENSOR_STATUS_DISCONNECTED)
        res = true;
    
    connectionStatus = SENSOR_STATUS_DISCONNECTED;

    return res;    
}

// 设置命令模式
-(void)setCommandMode {
    if ([self assertConnected]) {
        waitForAck = true;

        [self lpbusSetNone:GOTO_COMMAND_MODE];
        [self _waitForAckLoop];
    }
}

// change sensor to Streaming mode.
// it will send data too much ,and you have better to
// change sensor mode to set sensor or get some data you need.
-(void)setStreamingMode {
    if (![self assertConnected]) {
        return;
    }

    waitForAck = true;

    [self lpbusSetNone:GOTO_STREAM_MODE];
    [self _waitForAckLoop];

    isStreamMode = true;
}

-(void)setImuId:(int)id {
    if (![self assertConnected]) {
        return;
    }

    NSThread *nt = [[NSThread alloc] initWithTarget:self selector:@selector(setImuIdRun:) object:nil];

    [nt start];
}

-(void)setImuIdRun:(int )id {
    BOOL b = isStreamMode;
    [self setCommandMode];

    waitForAck = true;

    [self lpbusSetInt32:SET_IMU_ID and:id];
    [self _waitForAckLoop];
    [self _getSensorSettings];
    [self _saveParameters];
    
    if (b) {
        [self setStreamingMode];
    }
}

-(int)getImuId {    
    return imuId;
}

-(int)getGyroRange {     
    return gyrRange;
}

-(void)setGyroRange:(int)range {

    if (![self assertConnected]){
        return;
    }

    if (range == LPMS_GYR_RANGE_125DPS ||
        range == LPMS_GYR_RANGE_245DPS ||
        range == LPMS_GYR_RANGE_500DPS ||
        range == LPMS_GYR_RANGE_1000DPS||
        range == LPMS_GYR_RANGE_2000DPS) {
        
        dispatch_async(queue, ^{
            BOOL b = isStreamMode;

            [self setCommandMode];
            waitForAck = true;

            [self lpbusSetInt32:SET_GYR_RANGE and:range];
            [self _waitForAckLoop];
            [self _getSensorSettings];
            [self _saveParameters];

            if (b) {
                [self setStreamingMode];
            }
        });
    }
}

-(int)getAccRange {    
    return accRange;
}

-(void)setAccRange:(int)range
{
    if (![self assertConnected]) {
        return;
    }

    if (range == LPMS_ACC_RANGE_2G ||
        range == LPMS_ACC_RANGE_4G ||
        range == LPMS_ACC_RANGE_8G ||
        range == LPMS_ACC_RANGE_16G) {
        
        dispatch_async(queue, ^{
            BOOL b = isStreamMode;

            [self setCommandMode];
            waitForAck = true;

            [self lpbusSetInt32:SET_ACC_RANGE and:range];
            [self _waitForAckLoop];
            [self _getSensorSettings];
            [self _saveParameters];
            
            if (b) {
                [self setStreamingMode];
            }

        });
    }
}

-(int)getMagRange {
    return magRange;
}

-(void)setMagRange:(int)range {

    if (![self assertConnected]) {
        return;
    }
    if (range == LPMS_MAG_RANGE_4GAUSS ||
        range == LPMS_MAG_RANGE_8GAUSS ||
        range == LPMS_MAG_RANGE_12GAUSS ||
        range == LPMS_MAG_RANGE_16GAUSS) {
        
        dispatch_async(queue, ^{
            BOOL b = isStreamMode;

            [self setCommandMode];
            waitForAck = true;
            
            [self lpbusSetInt32:SET_ACC_RANGE and:range];
            [self _waitForAckLoop];
            [self _getSensorSettings];
            [self _saveParameters];
            
            if (b) {
                [self setStreamingMode];
            }
        });
    }
}

-(int)getFileterMode {    
    return filterMode;
}

-(void)setFilterMode:(int)mode {
    if (![self assertConnected]) {
        return;
    }

    if (mode == LPMS_FILTER_GYR ||
        mode == LPMS_FILTER_GYR_ACC ||
        mode == LPMS_FILTER_GYR_ACC_MAG ||
        mode == LPMS_FILTER_MADGWICK_GYR_ACC ||
        mode == LPMS_FILTER_MADGWICK_GYR_ACC_MAG) {
        
        dispatch_async(queue, ^{
            BOOL b = isStreamMode;
            [self setCommandMode];
            waitForAck = true;
            [self lpbusSetInt32:SET_FILTER_MODE and:mode];
            [self _waitForAckLoop];
            [self _getSensorSettings];
            [self _saveParameters];
            
            if (b) {
                [self setStreamingMode];
            }
        });
    }
}

-(int)getStreamFrequency {
    // NSLog(@"getStreamFrequency :%d",streamingFrequency);
    return streamingFrequency;
}

-(void)setStreamFrequency:(int )freq {

    if (![self assertConnected]) {
        return;
    }

    if (freq == LPMS_STREAM_FREQ_5HZ ||
        freq == LPMS_STREAM_FREQ_10HZ ||
        freq == LPMS_STREAM_FREQ_25HZ ||
        freq == LPMS_STREAM_FREQ_50HZ ||
        freq == LPMS_STREAM_FREQ_100HZ ||
        freq == LPMS_STREAM_FREQ_200HZ ||
        freq == LPMS_STREAM_FREQ_400HZ) {
        
        dispatch_async(queue, ^{
            //NSLog(@"12333");
            BOOL b = isStreamMode;
            [self setCommandMode];
            waitForAck = true;
            [self lpbusSetInt32:SET_STREAM_FREQ and:freq];
            [self _waitForAckLoop];
            [self _getSensorSettings];
            [self _saveParameters];
            
            if (b) {
                [self setStreamingMode];
            }
        });
    }
}

-(void)setTransmissionData:(int )v {
    if (![self assertConnected]) {
        return;
    }

    dispatch_async(queue, ^{
        BOOL b = isStreamMode;
        [self setCommandMode];
        waitForAck = true;
        [self lpbusSetInt32:SET_TRANSMIT_DATA and:v];
        [self _waitForAckLoop];
        [self _getSensorSettings];
        [self _saveParameters];
        
        if (b) {
            [self setStreamingMode];
        }
    });
}

-(void)enableGyroData:(BOOL)b {
    if (![self assertConnected])
        return;
    
    if (b)
        configurationRegister |= LPMS_GYR_RAW_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_GYR_RAW_OUTPUT_ENABLED;
    
    [self _setTransmissionData];
}

-(BOOL)isGyroDataEnabled {    
    return gyrEnable;
}

-(void)enableAccData:(BOOL)b {
    if (![self assertConnected])
        return;
    if (b)
        configurationRegister |= LPMS_ACC_RAW_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_ACC_RAW_OUTPUT_ENABLED;
       [self _setTransmissionData];
}

-(BOOL)isAccDataEnabled{    
    return accEnable;
}

-(void)enableMagData:(BOOL)b{
    if (![self assertConnected])
        return;

    if (b)
        configurationRegister |= LPMS_MAG_RAW_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_MAG_RAW_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)isMagDataEnabled {    
    return magEnable; 
}

-(void)enableAngularVelData:(BOOL)b {
    if (![self assertConnected])
        return;
    if (b)
        configurationRegister |= LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)isAngularVelDataEnable{    
    return angularVelEnable;
}

-(void)enableQuaternionData:(BOOL)b 
{
    if (![self assertConnected])
        return;

    if (b)
        configurationRegister |= LPMS_QUAT_OUTPUT_ENABLED;
    else
        configurationRegister &=~LPMS_QUAT_OUTPUT_ENABLED;
        
        // 传输数据
        [self _setTransmissionData];
}

-(BOOL)isQuaternionDataEnabled {    
    return quaternionEnable;
}

-(void)enableEulerData:(BOOL)b {
    if (![self assertConnected])
        return;

    if (b)
        configurationRegister |= LPMS_EULER_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_EULER_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)isEulerDataEnabledP {    
    return eulerAngleEnable;
}

-(void)enableLinAccData:(BOOL)b {
    if (![self assertConnected])
        return;

    if (b)
        configurationRegister |= LPMS_LINACC_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_LINACC_OUTPUT_ENABLED;
       [self _setTransmissionData];
}

-(BOOL)isLinAccDataEnabled {    
    return linAccEnable;
}

-(void)enablePressureData:(BOOL)b {
    if (![self assertConnected ])
        return;

    if (b)
        configurationRegister |= LPMS_PRESSURE_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_PRESSURE_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)isPressureDataEnabled {    
    return pressureEnable;
}

-(void)enableAltitudeData:(BOOL)b {
    if (![self assertConnected])
        return;
    
    if (b)
        configurationRegister |= LPMS_ALTITUDE_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_ALTITUDE_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)isAltitudeDataEnabled {    
    return altitudeEnable;
}


-(void)enableTemperatureData:(BOOL)b {
    if (![self assertConnected])
        return;

    if (b)
        configurationRegister |= LPMS_TEMPERATURE_OUTPUT_ENABLED;
    else
        configurationRegister &= ~LPMS_TEMPERATURE_OUTPUT_ENABLED;
        [self _setTransmissionData];
}

-(BOOL)enableTemperatureData {
    return temperatureEnable;
}

-(void)enable16BitData {
    if (![self assertConnected])
        return;

    configurationRegister &= ~LPMS_LPBUS_DATA_MODE_16BIT_ENABLED;
    [self _setTransmissionData];
}

-(void)enable32BitData {
    if (![self assertConnected])
        return;

    configurationRegister &= ~LPMS_LPBUS_DATA_MODE_16BIT_ENABLED;
    [self _setTransmissionData];
}

-(BOOL)is16BitDataEnabled {
    if (sixteenBitDataEnable)
        return true;
    else
        return false;
}

// Return the data length of the sensor.
-(int)hasNewData {
    int n;

    @synchronized(tableArray) {
        //[NSString stringWithFormat:@"%ld",tableArray.count]
        n = [[NSString stringWithFormat:@"%lu",(unsigned long)tableArray.count]intValue];
    }

    NSLog(@"Check hasNewData :%d",n);

    return n;
}

-(id)getLpmsBData {
    LpmsBData *d = nil;
    
    if (![self assertConnected])
        return d;
    
    if (!isStreamMode) {
        // 流模式
        @synchronized(tableArray) {
            while (tableArray.count > 0) {
                d =  tableArray.lastObject;
                // NSLog(@"DATA d %@",d);

                [tableArray removeLastObject];
                // NSLog(@"Test %lu",(unsigned long)tableArray.count);
            }
        }
        waitForData = true;

        // wrong
        [self lpbusSetNone:GET_SENSOR_DATA];
        [self _waitForDataLoop];
    }
    
    @synchronized(tableArray) {
        if (tableArray.count > 0) {
            d =  tableArray.lastObject;
            [tableArray removeLastObject];
        }
    }

    return d;
}

-(id)NSMutableArray {    
    return tableArray;
}

// not string
-(NSString*)getSerialNumber {   
    return serialNumber;
}

-(NSString*)getDeviceName {   
    return deviceName;
}

-(BOOL)isStreamingMode {   
    return isStreamMode;
}

-(int)getConnectionStatus {   
    return connectionStatus;
}

-(NSString*)getFirmwareInfo {   
    return firmwareInfo;
}

-(float)getBatteryLevel {
    return batteryLevel;
}

-(float)getBatteryVoltage {
    return batteryVoltage;
}

-(void)startSyncSensor {
    if (![self assertConnected])
        return;

    [self lpbusSetNone:START_SYNC];

    waitForAck = true;
    [self _waitForAckLoop];
}

-(void)stopSyncSensor {
    if (![self assertConnected]) {
        return;
    }

    [self lpbusSetNone:STOP_SYNC];

    waitForAck = true;
    [self _waitForAckLoop];
}

-(void)testPing {
    if (![self assertConnected ])
        return;

    [self lpbusSetNone:GET_PING];
}

-(void)resetFactorySettings {
    if (![self assertConnected])
        return;

    BOOL b = isStreamMode;

    [self setCommandMode];
    waitForAck = true;

    [self lpbusSetNone:RESTORE_FACTORY_VALUE];
    [self _waitForAckLoop];
    [self _getSensorSettings];
    [self _saveParameters];

    if (b)
        [self setStreamingMode];
}

// 设置方向偏移
-(void)setOrientationOffset:(int)offset {
    if (![self assertConnected])
        return;
    
    if (offset == LPMS_OFFSET_MODE_ALIGNMENT ||
        offset == LPMS_OFFSET_MODE_HEADING ||
        offset == LPMS_OFFSET_MODE_OBJECT) {
        [self lpbusSetInt32:SET_ORIENTATION_OFFSET and:offset];
    }
}

// 重置方向偏移
-(void)resetOrientationOffset {
    if (![self assertConnected])
        return;
 
   [self lpbusSetNone:RESET_ORIENTATION_OFFSET];
}

-(void)resetTimestamp {
    if (![self assertConnected])
        return;

    [self lpbusSetInt32:SET_TIME_STAMP and:0];
}

-(void)setTimestamp:(int)ts {
    if (![self assertConnected])
        return;

    [self lpbusSetInt32:SET_TIME_STAMP and:ts];
}

// 电池百分比
-(void)_getBatteryPercentage {
    if (![self assertConnected])
        return;

    [self lpbusSetNone:GET_BATTERY_LEVEL];
}

// 电池电压
-(void)_getBatteryVoltage {
    if (![self assertConnected])
        return;

    [self lpbusSetNone:GET_BATTERY_VOLTAGE];
}

-(void)getChargingStatus{
    if (![self assertConnected])
        return;
    
    [self lpbusSetNone:GET_CHARGING_STATUS];
}

// LP Bus Related
-(void)lpbusSetInt32:(int)command and:(int)v {
    for (int i = 0; i< 4 ; ++i) {
        // rawTxData[i] = [[NSString stringWithFormat:@"%d",(v & 0xff)]floatValue];
        rawTxData[i] = (Byte)(v & 0xff);
        v = v >> 8;
    }

    [self sendData:command Length:4];
}

-(void)lpbusSetNone:(int)command {    
    [self sendData:command Length:0];
}

-(void)lpbusSetData:(int)command and:(int)length and:(Byte[])dataBuffer
{    
    for (int i = 0; i < length; i++) {
        rawTxData[i] = dataBuffer[i];
    }
 
    [self sendData:command Length:length];
}

// 数据解析成功地方
-(void)parseSensorData {
    int o = 0;
    float r2d = 57.2958f;
    
    mLpmsBData -> _imuId = imuId;
    
    mLpmsBData ->_timestamp = (float)convertRxbytesToInt(o, rxBuffer)*0.0025f;
    o +=4;
        
    dt = mLpmsBData ->_timestamp - lose;
    lose = mLpmsBData ->_timestamp;
    if (dt > MaxDt) {
        MaxDt = dt;
        data = MaxDt;
    }

    if (gyrEnable) {        
        mLpmsBData -> _gyr[0] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _gyr[1] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _gyr[2] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        // NSLog(@"gyr[0]: %f",mLpmsBData ->_gyr[0]);
        // NSLog(@"gyr[1]: %f",mLpmsBData ->_gyr[1]);
        // NSLog(@"gyr[2]: %f",mLpmsBData ->_gyr[2]);
    }

    // accEnable
    if (accEnable) {
        
        // NSLog(@"%x",rxBuffer);
        mLpmsBData -> _acc[0] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _acc[1] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _acc[2] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        // NSLog(@"acc[0]: %f",mLpmsBData ->_acc[0]);
        // NSLog(@"acc[1]: %f",mLpmsBData ->_acc[1]);
        // NSLog(@"acc[2]: %f",mLpmsBData ->_acc[2]);
    }

    // magEnable
    if(magEnable) { 
        mLpmsBData -> _mag[0] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        mLpmsBData -> _mag[1] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        mLpmsBData -> _mag[2] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        // NSLog(@"mag[0]: %f",mLpmsBData ->_mag[0]);
        // NSLog(@"mag[0]: %f",mLpmsBData ->_mag[1]);
        // NSLog(@"mag[0]: %f",mLpmsBData ->_mag[2]);
    }
    
    if(angularVelEnable){
        
        mLpmsBData -> _angVel[0] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _angVel[1] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _angVel[2] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        // NSLog(@"angVel[0]: %f",mLpmsBData ->_angVel[0]);
        // NSLog(@"angVel[1]: %f",mLpmsBData ->_angVel[1]);
        // NSLog(@"angVel[2]: %f",mLpmsBData ->_angVel[2]);
    }

    // 开关
    if (quaternionEnable) {
        mLpmsBData -> _quat[0] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _quat[1] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _quat[2] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _quat[3] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        // NSLog(@"quat[0]: %f",mLpmsBData ->_quat[0]);
        // NSLog(@"quat[1]: %f",mLpmsBData ->_quat[1]);
        // NSLog(@"quat[2]: %f",mLpmsBData ->_quat[2]);
        // NSLog(@"quat[3]: %f",mLpmsBData ->_quat[3]);
    }
    
    if (eulerAngleEnable) {
        mLpmsBData -> _euler[0] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _euler[1] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        mLpmsBData -> _euler[2] = convertRxbytesToFloat(o, rxBuffer) * r2d;
        o+= 4;
        
        // NSLog(@"euler[0]: %f",mLpmsBData ->_euler[0]);
        // NSLog(@"euler[1]: %f",mLpmsBData ->_euler[1]);
        // NSLog(@"euler[2]: %f",mLpmsBData ->_euler[2]);
    }
    
    if (linAccEnable) {
        mLpmsBData -> _linAcc[0] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _linAcc[1] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        mLpmsBData -> _linAcc[2] = convertRxbytesToFloat(o, rxBuffer);
        o+= 4;
        
        // NSLog(@"linAcc[0]: %f",mLpmsBData ->_linAcc[0]);
        // NSLog(@"linAcc[1]: %f",mLpmsBData ->_linAcc[1]);
        // NSLog(@"linAcc[2]: %f",mLpmsBData ->_linAcc[2]);
    }
    
    if (pressureEnable) {
        mLpmsBData -> _pressure = convertRxbytesToFloat(o, rxBuffer);
        o+=4;
    }
    
    if (altitudeEnable) {
        // 高度计
        mLpmsBData -> _altitude = convertRxbytesToFloat(o, rxBuffer);
        o += 4;
    }
    
    if (temperatureEnable) {
        // 温度计
        mLpmsBData -> _temperature = convertRxbytesToFloat(o, rxBuffer);
        o += 4;
    }
    
    if (heaveEnable) {
        mLpmsBData -> _heave = convertRxbytesToFloat(o, rxBuffer);
        o += 4;
    }
    
    @synchronized(tableArray) {
        if (tableArray.count < DATA_QUEUE_SIZE ) {
            [tableArray addObject:[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]];
            [dl logLpmsData:[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]];
            
            // NSLog(@"Count %lu",(unsigned long)[tableArray count]);
            // NSLog(@"B2 %@",[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]);
        } else {  
            [tableArray removeLastObject];
            [tableArray addObject:[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]];
            [dl logLpmsData:[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]];
            // NSLog(@"Test");
        }
    }
    
    newDataFlag = true;
}

-(void) parseSensorData16Bit {
    int o = 0;
    float r2d = 57.2958f;
    
    if (a16 == true) {
        // resettimestamp
        mLpmsBData = [[LpmsBData alloc] initWithLpmsBData];
    }

    mLpmsBData->_imuId = imuId;
    mLpmsBData->_timestamp = (float) convertRxbytesToInt(0, rxBuffer)*0.0025f;
    
    o += 4;
 
    mLpmsBData->_frameNumber = frameCounter;
    frameCounter++;

    if ( gyrEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_gyr[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f * r2d;
            o += 2;
        }
        // Log.d(TAG, mLpmsBData.gyr[0]+" "+mLpmsBData.gyr[1]+" "+mLpmsBData.gyr[2]);
    }

    if ( accEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_acc[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f;
            o += 2;

        }
    }

    if ( magEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_mag[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 100.0f;
            o += 2;
        }
    }

    if ( angularVelEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_angVel[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f* r2d;
            o += 2;
        }
    }

    if ( quaternionEnable )
    {
        for (int i = 0; i < 4; ++i) {
            mLpmsBData->_quat[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f;
            o += 2;
        }
    }

    if ( eulerAngleEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_euler[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f * r2d;
            o += 2;
        }
    }

    if ( linAccEnable )
    {
        for (int i = 0; i < 3; ++i) {
            mLpmsBData->_linAcc[i] = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 1000.0f;
            o += 2;
        }
    }

    if ( pressureEnable )
    {
        mLpmsBData->_pressure = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 100.0f;
        o += 2;
    }

    if ( altitudeEnable )
    {
        mLpmsBData->_altitude = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 10.0f;
        o += 2;
    }

    if ( temperatureEnable )
    {
        mLpmsBData->_temperature = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 100.0f;
        o += 2;
    }

    if ( heaveEnable )
    {
        mLpmsBData->_heave = (float) ((short) (((rxBuffer[o + 1]) << 8) | (rxBuffer[o + 0] & 0xff))) / 100.0f;
        o += 2;
    }

    @synchronized (tableArray) {
        if (tableArray.count < DATA_QUEUE_SIZE) {
            [tableArray addObject:[[LpmsBData alloc] initWithLpmsBData:mLpmsBData]];
        }else{
            [tableArray removeLastObject];
            [tableArray addObject:[[LpmsBData alloc]initWithLpmsBData:mLpmsBData]];
        }
    }

    newDataFlag = true;
}


-(void)parseFunction{
    // 第一次进来 ＝ 0； ＝ 91
    // NSLog(@"currentFunction :%d", currentFunction);

    switch (currentFunction) {
        case REPLY_ACK:
            NSLog(@"parseFunction: Reply Ack~");
            waitForAck = false;
        break;
            
        case REPLY_NACK:
            NSLog(@"parseFunction: NAck~");
            waitForAck = false;
        break;
            
        case GET_CONFIG:
            // 配置登记册
            configurationRegister = convertRxbytesToInt(0, rxBuffer);
            
            // NSLog(@"ParseFunction configurationRegister :%d",configurationRegister);
            // 配置注册好了
        
            configurationRegisterReady = true;
            waitForData = false;
        break;
            
        case GET_STATUS:
            waitForData = false;
        break;
            
        case GET_SENSOR_DATA:
            // NSLog(@"parseFunction :get sensor data come in");
            if ((configurationRegister & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
                [self parseSensorData16Bit];
                // NSLog(@"parseSensorData16Bit");
            } else {
                // 默认，先做
                [self parseSensorData];

                // NSLog(@"parseSensorData");
                
            }
            waitForData = false;
        break;
            
        case GET_IMU_ID:
            NSLog(@"parseFunction: get IMU id ~");
            imuId = convertRxbytesToInt(0, rxBuffer);
            waitForData = false;
        break;
            
        case GET_GYR_RANGE:
            NSLog(@"parseFunction: get GYR_range  ~");
            gyrRange = convertRxbytesToInt(0, rxBuffer);
            waitForData = false;
        break;
            
        case GET_ACC_RANGE: // 加速度数据,
            NSLog(@"parseFunction: get ACC_range  ~");
            accRange = convertRxbytesToInt(0, rxBuffer);
            waitForData = false;
        break;
            
        case GET_MAG_RANGE:
            NSLog(@"parseFunction: get MAG_range  ~");
            magRange = convertRxbytesToInt(0, rxBuffer);
            waitForData = false;
        break;
            
        case GET_FILTER_MODE:
            filterMode = convertRxbytesToInt(0, rxBuffer);
            waitForData = false;
        break;
            
        case GET_BATTERY_LEVEL:
            NSLog(@"parseFunction: get Battery Level ~");
            mLpmsBData -> _batteryLevel = convertRxbytesToFloat(0, rxBuffer);
            batteryLevel = mLpmsBData -> _batteryLevel;
            // Log.d(TAG, "GET_BATTERY_LEVEL " +  mLpmsBData.batteryLevel);
            waitForData = false;
        break;
            
        case GET_CHARGING_STATUS://
            mLpmsBData -> _chargingStatus = convertRxbytesToInt(0, rxBuffer);
            // Log.d(TAG, "GET_CHARGING_STATUS ");
            waitForData = false;
        break;
            
        case GET_BATTERY_VOLTAGE://
            NSLog(@"parseFunction: get Battery Voltage ~");
            mLpmsBData -> _batteryVoltage = convertRxbytesToFloat(0, rxBuffer);
            batteryVoltage = mLpmsBData -> _batteryVoltage;
            // Log.d(TAG, "GET_BATTERY_VOLTAGE " + mLpmsBData.batteryVoltage);
            waitForData = false;
        break;
            
        case GET_SERIAL_NUMBER:
            NSLog(@"parseFunction :get serial number");
            serialNumber = convertRxbytesToString(24, rxBuffer);
            // Log.d(TAG, serialNumber);
            serialNumberReady = true;
            waitForData = false;
        break;
            
        case GET_DEVICE_NAME://
            deviceName = convertRxbytesToString(16, rxBuffer);
            // Log.d(TAG, deviceName);
            NSLog(@"parseFunction :get device name");
            deviceNameReady = true;
            waitForData = false;
        break;
            
        case GET_FIRMWARE_INFO://
            NSLog(@"parseFunction :get firmware info");
            firmwareInfo = convertRxbytesToString(16, rxBuffer);
            // NSLog(@"parseFunction firmwareInfo :%@",firmwareInfo);
            // Log.d(TAG, firmwareInfo);
            firmwareInfoReady = true;
            waitForData = false;
        break;
            
        case GET_FIRMWARE_VERSION://
            NSLog(@"parseFunction :get firmware version");
            vmajor = convertRxbytesToInt(8, rxBuffer);
            vminor = convertRxbytesToInt(4, rxBuffer);
            vbuild = convertRxbytesToInt(0, rxBuffer);
            firmwareVersion = [NSString stringWithFormat:@"%d%@%d%@%d",vmajor,@".",vminor,@".",vbuild];
            // NSLog(@"%@",firmwareVersion);
            waitForData = false;
        break;
            
        case GET_PING:
            mT = (float) convertRxbytesToInt(0, rxBuffer)*0.0025f;
            // Log.d(TAG, "GET_PING: " +  mT );
            waitForData = false;
        break;
            
        case START_SYNC:
            // Log.d(TAG, "START_SYNC");
            waitForAck = false;
        break;
            
        case STOP_SYNC:
            // Log.d(TAG, "STOP_SYNC");
            waitForAck = false;
        break;
            
        case SET_TRANSMIT_DATA:
            // NSLog(@"parseFunction: set transmit data");
            waitForData = false;
        break;
            
        case GET_TEMPERATURE://
            // mLpmsBData.temperature = convertRxbytesToFloat(0, rxBuffer);
            mLpmsBData -> _temperature = convertRxbytesToFloat(0, rxBuffer);
            // Log.d(TAG, "GET_TEMPERATURE");
            waitForData = false;
        break;
    }
    
    waitForAck = false;
    waitForData = false;
}



BOOL printData = true;

-(void) parse:(NSData *)data {
    int lrcReceived = 0;
    b = (Byte*)[data bytes];    
    
    for (int i = 0; i < [data length]; i++) {
        // NSLog(@"rxState :%d %x",rxState, b[i]);
        // NSLog(@"length %lu",(unsigned long)[data length]);

        switch (rxState) {
            case PACKET_TEST:
                
                if (b[i] == 0x0a) {
                    rxState = PACKET_END;
                }
                break;
                
            case PACKET_END:                
                if (b[i] == 0x3a) {
                    rxState = PACKET_ADDRESS0;
                }
                break;
                
            case PACKET_ADDRESS0:
                inBytes[0] = b[i];
                
                rxState = PACKET_ADDRESS1;
                break;
                
            case PACKET_ADDRESS1:
                inBytes[1] = b[i];
                currentAddress = convertRxbytesToInt16(0, inBytes);
                // Log.d("address",""+currentAddress);
                // NSLog(@"address :%d",currentAddress);
                imuId = currentAddress;
                rxState = PACKET_FUNCTION0;
                break;
                
            case PACKET_FUNCTION0:
                inBytes[0] = b[i];                
                rxState = PACKET_FUNCTION1;
                break;
                
            case PACKET_FUNCTION1:
                inBytes[1] = b[i];
                currentFunction = convertRxbytesToInt16(0, inBytes);
                rxState = PACKET_LENGTH0;
                break;
                
            case PACKET_LENGTH0:
                inBytes[0] = b[i];                
                rxState = PACKET_LENGTH1;
                break;
                
            case PACKET_LENGTH1:
                inBytes[1] = b[i];
                currentLength = convertRxbytesToInt16(0, inBytes);
                // NSLog(@"currentLength:%d",currentLength);//maybe
                rxState = PACKET_RAW_DATA;
                rxIndex = 0;
                break;
                
            case PACKET_RAW_DATA:
                if (rxIndex == currentLength) {
                    lrcCheck = (currentAddress & 0xffff) + (currentFunction & 0xffff) + (currentLength & 0xffff);//&是位运算符
                    for (int j = 0; j < currentLength; j++) {
                        if (j < MAX_BUFFER) {
                            lrcCheck += (int) rxBuffer[j] & 0xff;
                        } else
                            break;
                    }
                    inBytes[0] = b[i];
                    rxState = PACKET_LRC_CHECK1;
                } else {
                    if (rxIndex < MAX_BUFFER) {                        
                        rxBuffer[rxIndex] = b[i];
                        rxIndex++;
                    } else {
                        rxState = PACKET_LRC_CHECK1;
                    }
                    break;
                }
                break;
                
            case PACKET_LRC_CHECK1: //校检
                inBytes[1] = b[i];
                
                lrcReceived = convertRxbytesToInt16(0, inBytes);
                lrcCheck = lrcCheck & 0xffff;
                
                // 第一个丢包问题
                if (lrcReceived == lrcCheck) {
                    [self parseFunction];
                } else {
                    printData = false;
                }
                
                rxState = PACKET_TEST;
                // rxState = PACKET_END;
                break;
                
            default:
                rxState = PACKET_TEST;
                // rxState = PACKET_END;
                break;
        }
    }
}

-(void)sendData:(int )function Length:(int )length {
    int txLrcCheck;
    // NSLog(@"sendData");

    txBuffer[0] = 0x3a;

    [self convertInt16ToTxbytes:imuId and:1 and:txBuffer];
    [self convertInt16ToTxbytes:function and:3 and:txBuffer];
    [self convertInt16ToTxbytes:length and:5 and:txBuffer];
    
    for (int i = 0; i < length; ++i) {
        txBuffer[7+i] = rawTxData[i];
    }

    txLrcCheck = (imuId & 0xffff) + (function & 0xffff) + (length & 0xffff);
    
    // NSLog(@"sendData txLrcCheck %d", txLrcCheck);
    for (int j = 0; j <length ; j++) {
        txLrcCheck += ceilf(rawTxData[j] & 0xff);
        // NSLog(@"txLrcCheck :%d",txLrcCheck);
    }
    
    [self convertInt16ToTxbytes:txLrcCheck and:7+length and:txBuffer];
    
    txBuffer[9+length] = 0x0d;
    txBuffer[10+length] = 0x0a;
    
    NSString *s = @"";
    NSString *news = @"";
    
    for (int i = 0; i < 11+length; i++) {
        news = [NSString stringWithFormat:@"%x", txBuffer[i] & 0xff];
        // NSLog(@"sendData :%@", news);
        
        s = [NSString stringWithFormat:@"%@%@", s, news];
    }
    NSLog(@"sendData s:%@",s);
    
    @try {
        [self wirte:txBuffer length:length+11];
        // NSLog(@"Test: %hhu",txBuffer[14]);
    }
    @catch (NSException *exception) {
        NSLog(@"mOutStream exception: %@",exception);
    }
}

-(void)wirte:(Byte*)b length:(int)l {
    NSData *data = [[NSData alloc] initWithBytes:b length:l];
    // CBCharacteristicWriteWithResponse / CBCharacteristicWriteWithoutResponse
    [p writeValue:data forCharacteristic:chara type:CBCharacteristicWriteWithResponse];
}

// 回调
-(void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error{
    // NSLog(@"didWriteValueForCharacteristic");
    
    [p readValueForCharacteristic:chara];
    Check = true;

    if (error) {
        NSLog(@"didWriteValueForCharacteristic error :%@",error.localizedDescription);
    }
}

-(void)sendAck{ [self sendData:REPLY_ACK Length:0]; }
-(void)sendNack{ [self sendData:REPLY_NACK Length:0]; }

typedef union bytesToFloat {    
    float f;
    Byte buffer[4];
    int i;
} bytesToFloat;

// Test
float convertRxbytesToFloat(int offset,Byte buffer[]) {
    bytesToFloat val;
    int l;

    l = buffer[offset + 0];
    l &= 0xff;
    l |=((long)buffer[offset + 1] << 8);
    l &= 0xffff;
    l |= ((long) buffer[offset + 2] << 16);
    l &= 0xffffff;
    l |= ((long) buffer[offset + 3] << 24);
    
    val.i = l;
    // NSLog(@"val int : %f",val.f);

    return val.f;
}

int convertRxbytesToInt(int offset, Byte buffer[]) {
    int v;

    v = (int) ((buffer[offset] & 0xFF)
               | ((buffer[offset+1] & 0xFF)<<8)
               | ((buffer[offset+2] & 0xFF)<<16)
               | ((buffer[offset+3] & 0xFF)<<24));

    return v;    
}

int convertRxbytesToInt16(int offset, Byte buffer[]) {
    int v;
    
    v= (int) ((buffer[offset]&0xFF) | ((buffer[offset+1]<<8) & 0xFF00));
    // NSLog(@"int16 :%d",v);

    return v;
}

NSString* convertRxbytesToString(int length, Byte buffer[]) {
    // Byte[] t = new byte[length];
    Byte t[length];

    for (int i = 0; i < length; i++) {
        t[i] = buffer[i];
    }

    NSString *decodedString =[[NSString stringWithFormat:@"%s",t] stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
    // NSLog(@"convertRxbytesToString :%@",decodedString);

    return decodedString;
}

// BYTE转32位int
-(void)convertIntToTxbytes:(int)v and:(int)offset and:(Byte[])buffer {
    Byte t[4];

    t[3] = (Byte)(0xff & v);

    for (int i = 0; i < 4; i++) {
        buffer[3 - i + offset] = t[i];
    }
}

// BYTE转16位int
-(void)convertInt16ToTxbytes:(int)v and:(int)offset and:(Byte[])buffer {
    Byte t[2];

    t[1] = (Byte)(v & 0xff);

    // 不能注释
    for (int i = 0; i < 2; i++) {
        buffer[1 - i + offset] = t[i];
    }
}

-(void)convertFloatToTxbytes:(float)f and:(int)offset and:(Byte[])buffer{
    int v = ceilf(f);
    Byte t[4];

    t[3] = (Byte)(v & 0xff);

    for (int i = 0; i < 4; i++) {
        buffer[3 - i + offset] = t[i];
    }
}

-(BOOL)startLogging {
    if (connectionStatus == SENSOR_STATUS_DISCONNECTED) {
        [dl setStatusMesg:@"NO sensor connected"];
        NSLog(@"NO sensor connected");

        return false;
    }

    if ([dl startLogging]) {
        [self resetTimestamp];

        return true;
    }

    return false;
}

-(BOOL)stopLogging {
    return [dl stopLogging];
}

-(BOOL)isLogging{   
    return [dl isLogging];
}

//　-(NSString)getLoggerStatusMesg　{   
    return [dl getStatusMesg];
}

//　-(NSString)getOutputFilename　{     
    return [dl getOutputFilename];
}

-(void)_waitForAckLoop　{
    int timeout = 0;

    while (timeout++ < 60 && waitForAck) {
        @try {
            //　NSLog(@"ack timeout :%i",timeout);

            //　单位为秒
            [NSThread sleepForTimeInterval:PARAMETER_SET_DELAY];
        }

        @catch (NSException *exception) {
            NSLog(@"_waitForAckLoop exception :%@",exception);
        }
    }
}


//　等待数据循环
-(void)_waitForDataLoop{
    int  timeout = 0;

    while (timeout++ < 60 && waitForData) {
        @try {
            [NSThread sleepForTimeInterval:PARAMETER_SET_DELAY];
        }
        @catch (NSException *exception) {
            NSLog(@"_waitForDataLoop exception :%@",exception);
        }
    }
}

//　获取传感器设定
-(void)_getSensorSettings　{
    [self _getSerialNumber];

    //　3a105b0005c0da
    [self _getDeviceName];

    //　3a105c0005d0da
    [self _getFirmwareInfo];

    //　3a10400050da
    [self _getConfig];

    [self _getGyroRange];
    [self _getAccRange];
    [self _getMagRange];
    [self _getFilterMode];
    [self printConfig];
}

//　获取配置
-(void)_getConfig　{
    //　NSLog(@"_getConfig %d",configurationRegisterReady);
    configurationRegisterReady = false;
    [self lpbusSetNone:GET_CONFIG];
    //　configurationRegisterReady  = true

    while (!configurationRegisterReady) {
        @try {
            [NSThread sleepForTimeInterval:PARAMETER_SET_DELAY];
        }
        @catch (NSException *exception) {
            NSLog(@"_getConfig exception :%@",exception);
        }
    }

    //　1110842396
    //　NSLog(@"_getconfig configurationRegister :%d",configurationRegister);
    
    [self parseConfig:configurationRegister];
}

-(void)parseConfig:(int)config　{
    //　Stream frequency
    if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_5HZ_ENABLED) {
        streamingFrequency = LPMS_STREAM_FREQ_5HZ;
    }　else if((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_10HZ_ENABLED){
        streamingFrequency = LPMS_STREAM_FREQ_10HZ;
    }　else if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_25HZ_ENABLED)　{
        streamingFrequency = LPMS_STREAM_FREQ_25HZ;
    }　else if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_50HZ_ENABLED)　{
        streamingFrequency = LPMS_STREAM_FREQ_50HZ;
    }　else if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_100HZ_ENABLED)　{
        streamingFrequency = LPMS_STREAM_FREQ_100HZ;
    }　else if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_200HZ_ENABLED)　{
        streamingFrequency = LPMS_STREAM_FREQ_200HZ;
    }　else if ((configurationRegister & LPMS_STREAM_FREQ_MASK) == LPMS_STREAM_FREQ_400HZ_ENABLED)　{
        streamingFrequency = LPMS_STREAM_FREQ_400HZ;
    }
    
    if ((config & LPMS_GYR_RAW_OUTPUT_ENABLED) != 0) {
        gyrEnable = true;
    }　else　{
        gyrEnable = false;
    }
    
    if ((config & LPMS_ACC_RAW_OUTPUT_ENABLED) != 0) {
        accEnable = true;
    }　else　{
        accEnable = false;
    }
    
    if ((config & LPMS_MAG_RAW_OUTPUT_ENABLED) != 0) {
        magEnable = true;
    }　else　{
        magEnable = false;
    }
    
    if ((config & LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED) != 0) {
        angularVelEnable = true;
    }　else　{
        angularVelEnable = false;
    }
    
    if ((config & LPMS_QUAT_OUTPUT_ENABLED) != 0) {
        quaternionEnable = true;
    }　else　{
        quaternionEnable = false;
    }
    
    if ((config & LPMS_EULER_OUTPUT_ENABLED) != 0) {
        eulerAngleEnable = true;
    }　else　{
        eulerAngleEnable = false;
    }
    
    if ((config & LPMS_LINACC_OUTPUT_ENABLED) != 0) {
        linAccEnable = true;
    }　else　{
        linAccEnable = false;
    }
    
    if ((config & LPMS_PRESSURE_OUTPUT_ENABLED) != 0) {
        pressureEnable = true;
    }　else　{
        pressureEnable = false;
    }
    
    if ((config & LPMS_TEMPERATURE_OUTPUT_ENABLED) != 0) {
        temperatureEnable = true;
    }　else　{
        temperatureEnable = false;
    }
    
    if ((config & LPMS_ALTITUDE_OUTPUT_ENABLED) != 0) {
        altitudeEnable = true;
    }　else　{
        altitudeEnable = false;
    }
    
    if ((config & LPMS_HEAVEMOTION_OUTPUT_ENABLED) != 0) {
        heaveEnable = true;
    }　else　{
        heaveEnable = false;
    }
    
    if ((config & LPMS_LPBUS_DATA_MODE_16BIT_ENABLED) != 0) {
        sixteenBitDataEnable = true;
    }　else　{
        sixteenBitDataEnable = false;
    }
}

//　获取gyro范围
-(void)_getGyroRange　{ 
    waitForData = true;
    [self lpbusSetNone:GET_GYR_RANGE];
    [self _waitForDataLoop];
}

//　获取acc范围
-(void)_getAccRange　{
    waitForData = true;
    [self lpbusSetNone:GET_ACC_RANGE];
    [self _waitForDataLoop];
}

//　获取mag范围
-(void)_getMagRange　{
    waitForData = true;
    [self lpbusSetNone:GET_MAG_RANGE];
    [self _waitForDataLoop];
}

//　获取过滤模式
-(void)_getFilterMode　{
    waitForData = true;
    [self lpbusSetNone:GET_FILTER_MODE];
    [self _waitForDataLoop];
}

//　获取序列号
-(void)_getSerialNumber　{
    serialNumberReady = false;
    serialNumber = @"";
    [self lpbusSetNone:GET_SERIAL_NUMBER];

    while (!serialNumberReady) {
        @try {
            [NSThread sleepForTimeInterval:PARAMETER_SET_DELAY];
            //　[NSTimer scheduledTimerWithTimeInterval:PARAMETER_SET_DELAY target:self selector:@selector(_waitForAckLoop) userInfo:nil repeats:NO];
        }
        @catch (NSException *exception) {
            NSLog(@"_getSerialNumber exception:%@",exception);
        }
    }
}

//　获取name
-(void)_getDeviceName　{
    deviceNameReady = false;
    deviceName = @"";

    [self lpbusSetNone:GET_DEVICE_NAME];

    while (!deviceNameReady) {
        @try {
            [NSThread sleepForTimeInterval:PARAMETER_SET_DELAY];
        }
        @catch (NSException *exception) {
            NSLog(@"_getDeviceName exception :%@",exception);
        }
    }
}

//　获取固件信息
-(void)_getFirmwareInfo　{
    firmwareInfoReady = false;
    firmwareInfo = @"";

    [self lpbusSetNone:GET_FIRMWARE_INFO];

    while (!firmwareInfoReady) {
        @try {
            [NSTimer scheduledTimerWithTimeInterval:PARAMETER_SET_DELAY target:self selector:@selector(_waitForAckLoop) userInfo:nil repeats:NO];
        }
        @catch (NSException *exception) {
            NSLog(@"_getFirmwareInfo exception :%@",exception);
        }
    }
}

-(void)_saveParameters　{
    waitForAck = true;

    [self lpbusSetNone:WRITE_REGISTERS];
    [self _waitForAckLoop];
}

-(void)_setTransmissionData　{
    BOOL b = isStreamMode;

    [self setCommandMode];
    waitForAck = true;

    [self lpbusSetInt32:SET_TRANSMIT_DATA and:configurationRegister];
    [self _waitForAckLoop];
    [self _getSensorSettings];
    [self _saveParameters];

    if (b) {
        [self setStreamingMode];
    }    
}

-(BOOL) assertConnected { 
    if　(connectionStatus != SENSOR_STATUS_DISCONNECTED)　{
        return true;
    }

    return false;
}

-(void)printConfig {
    NSLog(@"LpmsB2 SN: %@",serialNumber);
    NSLog(@"LpmsB2 FW: %@",firmwareInfo);
    NSLog(@"LpmsB2 DN: %@",deviceName);
    NSLog(@"LpmsB2 imuId: %d",imuId);

    // 流的频率
    NSLog(@"LpmsB2 StreamFreq: %d",streamingFrequency);
    NSLog(@"LpmsB2 Gyro: %d",gyrRange);
    NSLog(@"LpmsB2 Acc: %d",accRange);
    NSLog(@"LpmsB2 Mag: %d",magRange);
    
    if (gyrEnable) {
        NSLog(@"LpmsB2 GYRO ENABLED");
    } else {
        NSLog(@"LpmsB2 GYRO DISABLED");
    }
    
    if (accEnable) {
        NSLog(@"LpmsB2 ACC ENABLED");
    } else {
        NSLog(@"LpmsB2 ACC DISABLED");
    }
    
    if (magEnable) {
        NSLog(@"LpmsB2 MAG ENABLED");
    } else {
        NSLog(@"LpmsB2 MAG DISABLED");
    }
    
    if (angularVelEnable) {
        NSLog(@"LpmsB2 AngVel ENABLED");
    } else {
        NSLog(@"LpmsB2 AngVel DISABLED");
    }
    
    if (quaternionEnable) {
        NSLog(@"LpmsB2 QUAT ENABLED");
    } else {
        NSLog(@"LpmsB2 QUAT DISABLED");
    }
    
    if (eulerAngleEnable) {
        NSLog(@"LpmsB2 EULER ENABLED");
    } else {
        NSLog(@"LpmsB2 EULER DISABLED");
    }
    
    if (linAccEnable) {
        NSLog(@"LpmsB2 LINACC ENABLED");
    } else {
        NSLog(@"LpmsB2 LINACC DISABLED");
    }
    
    if (pressureEnable) {
        NSLog(@"LpmsB2 PRESSURE ENABLED");
    } else {
        NSLog(@"LpmsB2 PRESSURE DISABLED");
    }
    
    if (altitudeEnable) {
        NSLog(@"LpmsB2 ALTITUDE ENABLED");
    } else {
        NSLog(@"LpmsB2 ALTITUDE DISABLED");
    }
    
    if (temperatureEnable) {
        NSLog(@"LpmsB2 TEMPERATURE ENABLED");
    } else {
        NSLog(@"LpmsB2 TEMPERATURE DISABLED");
    }
    
    if (heaveEnable) {
        NSLog(@"LpmsB2 heave ENABLED");
    } else {
        NSLog(@"LpmsB2 heave DISABLED");
    }
    
    if (sixteenBitDataEnable) {
        NSLog(@"LpmsB2 16 bit ENABLED");
    } else {
        NSLog(@"LpmsB2 16 bit DISABLED");
    }
}

// 传值方法
-(void)settimeStamp:(sentValue)_sentBlock {
    _sentBlock([NSString stringWithFormat:@"%@%f",@"MaxDt :",data]);
}

-(void)Reset:(float)m {
    // NSLog(@"reset %f",MaxDt);
    MaxDt = m;
}

@end