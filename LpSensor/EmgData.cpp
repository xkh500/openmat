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

#include "EmgData.h"

#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

void EmgData::writeCSVHeader(std::ofstream& of)
{
    of << "SensorId, TimeStamp(s), FrameNumber, ";
    /* for (int i = 0; i < 20; ++i) {
        of << "Voltage#" << i << "(V),"
		<< "AudioL(dB),Audio R,"
        << "AccX(g),AccY(g),AccZ(g),"
        << "GyroX(deg/s),GyroY (deg/s),GyroZ (deg/s)"
        << std::endl;
	} */

	of << "Voltage(V), "
		<< "AudioL(dB), AudioR(dB),"
		<< "AccX(g), AccY(g), AccZ(g),"
		<< "GyroX(deg/s), GyroY(deg/s), GyroZ(deg/s)"
		<< std::endl;
}

void EmgData::writeCSVData(std::ofstream& of, int frameCounterOffset)
{
    of << openMatId << "," << timeStamp
        << ", " << (frameCount - frameCounterOffset)
        << ", ";
    
	/* for (int i = 0; i < 20; ++i) {
        of << voltage[i] << ",";
	} */

	of << singleVoltage << ", ";
	
    of << audio[0] << ", " << audio[1]
        << ", " << acc[0] << ", " << acc[1] << ", " << acc[2]
        << ", " << gyro[0] << ", " << gyro[1] << ", " << gyro[2]
        << std::endl;
}

bool EmgData::readCSVData(std::string& line)
{
    boost::char_separator<char> sep(", ");
    boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator ti;

    if (tokens.begin() == tokens.end())
        return false;

    ti = tokens.begin();
    openMatId = (int) boost::lexical_cast<double>(*ti); ++ti;
    timeStamp = boost::lexical_cast<double>(*ti); ++ti;
    frameCount = (int) boost::lexical_cast<double>(*ti); ++ti;

    /* for (int i = 0; i < 20; ++i)
        voltage[i] = boost::lexical_cast<double>(*ti); ++ti; */

	singleVoltage = (int) boost::lexical_cast<double>(*ti); ++ti;

    for (int i = 0; i < 3; ++i) {
        acc[i] = (int) boost::lexical_cast<double>(*ti); ++ti;
    }
    for (int i = 0; i < 3; ++i) {
        gyro[i] = (int) boost::lexical_cast<double>(*ti); ++ti;
    }

    return ti == tokens.end();
}
