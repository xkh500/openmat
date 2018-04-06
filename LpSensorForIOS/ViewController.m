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

#import "ViewController.h"

#import <CoreBluetooth/CoreBluetooth.h>

#import "LpmsB2.h"
#import "LpmsBData.h"
#import "SecondViewController.h"
#import "View.h"

#define kServiceUUID @"0000180F-0000-1000-8000-00805F9B34FB"
#define kCharacteristicUUID @"00002A19-0000-1000-8000-00805F9B34FB"

@interface ViewController ()<CBCentralManagerDelegate,CBPeripheralDelegate,UITabBarDelegate,UITableViewDataSource,UITableViewDelegate>

@property (strong,nonatomic) CBCentralManager *centralManager;
@property (strong,nonatomic) NSMutableArray *peripherals;

- (IBAction)startClick:(id)sender;
- (IBAction)End:(id)sender;

@end

NSMutableArray *Device;
UITableView *tableview;
LpmsBData *mlpmsdata;
CBPeripheral *Peripheral;
LpmsB2 *lpmsb2;
bool Test11 = true;
NSData *data1;

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    _centralManager = [[CBCentralManager alloc]initWithDelegate:self queue:nil];
    
    self.view.backgroundColor = [UIColor whiteColor];
}

- (IBAction)startClick:(UIBarButtonItem *)sender {
    [self centralManagerDidUpdateState:_centralManager];

    if (_centralManager.state == CBCentralManagerStatePoweredOff) {
        _centralManager=[[CBCentralManager alloc]initWithDelegate:self queue:nil];
        [self.centralManager scanForPeripheralsWithServices:nil options:
         @{CBCentralManagerScanOptionAllowDuplicatesKey:@YES}];
    }
    
    if (_centralManager.state == CBCentralManagerStatePoweredOn) {
        [self.centralManager scanForPeripheralsWithServices:nil options:nil];
    }
}

- (IBAction)End:(id)sender {
    if (_peripherals == nil) {
    } else {
        [self disconnect];
        
        SecondViewController *scv = [[SecondViewController alloc]init];
        [scv Cancel];
    }
}

- (void)disconnect {
    if(self.centralManager && self.peripherals) {
        [self.centralManager cancelPeripheralConnection:Peripheral];
        NSLog(@"Peripherial status:%@", Peripheral);
        isConnected = false;
    }
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
    
    switch (central.state) {
        case 0:
            NSLog(@"CBCentralManagerStateUnknown");
            break;
            
        case 1:
            NSLog(@"CBCentralManagerStateResetting");
            break;
            
        case 2:
            NSLog(@"CBCentralManagerStateUnsupported");
            break;
            
        case 3:
            NSLog(@"CBCentralManagerStateUnauthorized");
            break;
            
        case 4:
            NSLog(@"CBCentralManagerStatePoweredOff");
            break;
            
        case 5:
            NSLog(@"CBCentralManagerStatePoweredOn");
            break;
            
        default:
            break;
    }
}

-(void)foundTableView {
    tableview = [[UITableView alloc]initWithFrame:CGRectMake(20, 125, 500, 500) style:UITableViewStyleGrouped];
    [tableview setRowHeight:40];
    [tableview setSeparatorColor:[UIColor grayColor]];
    [tableview setSeparatorStyle:UITableViewCellSeparatorStyleSingleLineEtched];
    [tableview setBackgroundColor:[UIColor whiteColor]];
    
    tableview.delegate = self;
    tableview.dataSource = self;

    [self.view addSubview: tableview];
}

-(void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI {
    if (Test11 == true) {
        Device = [[NSMutableArray alloc]init];
        Test11 = false;
    }

    if (peripheral) {
        if (peripheral.name != (NULL)) {
            if (![Device containsObject:peripheral]  ) {
                [Device addObject:peripheral];
                NSLog(@"peripheral.name: %@",peripheral.name);
            } else {
                [self.peripherals addObject:peripheral];
            };
        }
        if (self.peripherals != nil) {
            [self foundTableView];
        }
    }
}

-(void)delay {
    [self performSelector:@selector(StopScan) withObject:nil afterDelay:3.0];
}

-(void)StopScan{
    [self.centralManager stopScan];
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    return 1;
}

-(NSString *)tableView:(UITableView *)tableView titleForHeaderInSection:(NSInteger)section {
    return @"Dicovered Bluetooth Devices:";
}

-(NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
    return [Device count];
}

static NSString *cell_Id = @"cell_Id";

-(UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:cell_Id];

    if (cell == nil) {
        cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleValue1 reuseIdentifier:cell_Id];
    }

    cell.layer.cornerRadius = 12;
    cell.layer.masksToBounds = YES;
    
    CBPeripheral *per = (CBPeripheral *)Device[indexPath.row];
    cell.textLabel.text = per.name;

    cell.accessoryType = UITableViewCellAccessoryDisclosureIndicator;
    
    return cell;
}

bool isConnected = false;

-(void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath {
    Peripheral = (CBPeripheral *)Device[indexPath.row];
    Peripheral.delegate = self;
    
    if (isConnected == false) {
        lpmsb2 = [[LpmsB2 alloc] init];
        [lpmsb2 connect:_centralManager Address:Peripheral];
        isConnected = true;
    }
    
    [self delayMethod];
}

-(void)delayMethod {
    [self performSelector:@selector(StartView) withObject:nil afterDelay:2.0];
}

-(void)StartView {
    View *view = [[View alloc] init];
    [self.navigationController pushViewController:view animated:YES];
}

-(NSMutableArray *)peripherals {
    if (!_peripherals) {
        _peripherals=[NSMutableArray array];
    }
    
    return _peripherals;
}

@end
