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

#ifndef CALIBRATION_DATA
#define CALIBRATION_DATA

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;

#include "LpMatrix.h"
#include "LpmsDefinitions.h"
#include "util.h"
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include "pugixml.hpp"

#define ABSMAXPITCH 3
#define ABSMAXROLL 6
#define ABSMAXYAW 6

class CalibrationData
{
    const std::string TAG;
public:
    std::string name;
    std::string deviceId;
    std::string firmwareInfo;
    std::string serialNumber;
    int openMatId;
    int deviceType;
    int parameterSet;
    int filterMode;
    int gyrThresEnable;
    float accCompGain;
    float accCovariance;
    float magCompGain;
    float magCovariance;
    int quaternionCalcLocal;
    int samplingRate;
    int gyrRange;
    int magRange;
    int accRange;
    int magAutocalibration;
    int canStreamFormat;
    int canBaudrate;
    int selfTestOn;
    float fieldRadius;
    int magThreshold;
    int magOutOfRange;
    int gyrAutocalibration;
    LpVector3f fieldMap[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW];
    LpVector3f hardIronOffset;
    LpMatrix3x3f softIronMatrix;
    LpMatrix3x3f misalignMatrix;
    LpVector3f accBias;
    LpMatrix3x3f gyrMisalignMatrix;
    LpVector3f gyrAlignmentBias;
    LpVector3f gyrStaticBias;
    int selectedData;
    std::string firmwareVersion;
    int lowPassFilter;
    int canMapping[32];
    int canHeartbeat;
    int heavemotionEnabled;
    int gaitTrackingEnabled;
    int linAccCompMode;
    int centriCompMode;
    int canPointMode;
    int canChannelMode;
    int canStartId;
    int lpBusDataMode;
    LpVector3f magReference;
    int firmwareVersionDig0;
    LpVector3f magMAlignmentBias;
    int firmwareVersionDig1;
    LpMatrix3x3f magMAlignmentMatrix;
    int firmwareVersionDig2;
    int uartBaudrate;
    int uartFormat;

private:
    std::mutex calibrationMutex;

public:
    CalibrationData(void);
    bool setDefaultParameters(std::string name, std::string deviceId, int deviceType);
    bool setParameter(int parameterIndex, std::string parameter);
    bool setParameter(int parameterIndex, int parameter);
    bool setParameter(int parameterIndex, int *parameter);
    bool setParameter(int parameterIndex, float parameter);
    bool getParameter(int parameterIndex, std::string *parameter);
    bool getParameter(int parameterIndex, int *parameter);
    bool getParameter(int parameterIndex, float *parameter);

    void writeXML(std::string tag, pugi::xml_node node, LpVector3f v);
    void writeXML(std::string tag, pugi::xml_node node, LpVector4f v);
    void writeXML(std::string tag, pugi::xml_node node, LpMatrix3x3f m);
    void writeXML(std::string tag, pugi::xml_node node, bool b);
    void writeXML(std::string tag, pugi::xml_node node, std::string s);
    void writeXML(std::string tag, pugi::xml_node node, int i);
    void writeXML(std::string tag, pugi::xml_node node, float v);

    bool readXML(std::string tag, pugi::xml_node node, LpVector3f *v);
    bool readXML(std::string tag, pugi::xml_node node, LpVector4f *v);
    bool readXML(std::string tag, pugi::xml_node node, LpMatrix3x3f *m);
    bool readXML(std::string tag, pugi::xml_node node, int *i);
    bool readXML(std::string tag, pugi::xml_node node, bool *b);
    bool readXML(std::string tag, pugi::xml_node node, std::string *s);
    bool readXML(std::string tag, pugi::xml_node node, float *v);

    bool load(std::string fn);
    bool save(std::string fn);

    void print(void);
};

#endif
