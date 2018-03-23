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

#ifndef IMU_DATA
#define IMU_DATA

// Structure for data exchange inside the OpenMAT network
typedef struct _ImuData {
    // The OpenMAT ID of the sensor that created this data structure.
    int openMatId;

    // Calibrated accelerometer sensor data (unit: g)
    float a[3];

    // Calibrated gyroscope sensor data (unit: deg/s)
    float g[3];

    // Calibrated magnetometer sensor data (unit: uT)
    float b[3];

    // Angular velocity data (unit: radians/s)
    float w[3];

    // Euler angle data (unit: degree)
    float r[3];

    // Quaternion orientation data (no unit)
    float q[4];

    // Orientation data as rotation matrix without offset.
    float rotationM[9];

    // Orientation data as rotation matrix after zeroing.
    float rotOffsetM[9];

    // Raw accelerometer (without misalignment correction) sensor data (unit: g)
    float aRaw[3];

    // Raw gyroscope (without misalignment correction) sensor data (unit: deg/s)
    float gRaw[3];

    // Raw magnetometer (without magnetic field correction) sensor data (unit: uT)
    float bRaw[3];

    // Barometric pressure (unit: kPa)
    float pressure;

    // Index of the data frame 
    int frameCount;

    // Linear acceleration x, y and z (unit: g)
    float linAcc[3];

    // Gyroscope temperature (unit: degree Celcius)
    float gTemp;

    // Altitude (unit: m)
    float altitude;

    // Temperature (unit: degree Celcius)
    float temperature;

    // Sampling time of the data (unit: seconds)
    double timeStamp;

    // Heave motion (unit: m)
    float heave;
} ImuData;

#endif