/***********************************************************************
** (c) LP-RESEARCH Inc.
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
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

#include "GraphWindow.h"

#include <iostream>
#include <string>

#include <QGridLayout>

using namespace std;

GraphWindow::GraphWindow(QWidget* parent) : QWidget(parent)
{
    const int nSamples = 1000;
    mPlots["voltage"] = new Plot("", "Samples", "Voltage [V]",
        "V", "", "", "",
        0, nSamples * 20, 1, 0, 1.5f);
    // TODO check L/R order
    mPlots["audio"] = new Plot("Audio", "Samples", "Amplitude",
        "L", "R", "", "",
        0, nSamples, 2, 0, 1.0f);
    mPlots["gyro"] = new Plot("Angular Velocity", "Samples", "Angular Velocity [deg/s]", "X", "Y", "Z", "",
        0, nSamples, 3, -2000.f, 2000.f);
    mPlots["acc"] = new Plot("Acceleration", "Samples", "Acceleration [g]", "X", "Y", "Z", "",
        0, nSamples, 3, -5.f, 5.f);

    QGridLayout *vLayout = new QGridLayout();

    vLayout->addWidget(mPlots["voltage"], 0, 0);
    vLayout->addWidget(mPlots["audio"], 1, 0);
    vLayout->addWidget(mPlots["acc"], 2, 0);
    vLayout->addWidget(mPlots["gyro"], 3, 0);
    //vLayout->setColumnStretch(0, 2);

    this->setLayout(vLayout);

    setPalette(QPalette(QColor(255, 255, 255)));

    setMode(Mode::Emmg);
}

void GraphWindow::setMode(const Mode& mode)
{
    this->mode = mode;

    clearGraphs();

    switch (mode) {
	case Mode::Emmg:
        mPlots["voltage"]->show();
        mPlots["audio"]->show();
        mPlots["acc"]->hide();
        mPlots["gyro"]->hide();
		break;
		
    case Mode::Motion:
        mPlots["acc"]->show();
        mPlots["gyro"]->show();
        mPlots["voltage"]->hide();
        mPlots["audio"]->hide();
    break;
    }
}

void GraphWindow::plotDataSet(const EmgData& ld)
{
    // mPlots["voltage"]->addData(ld.voltage, 1); // FIXME: this will only display the first sample in each block of voltages
	mPlots["voltage"]->addData(ld.singleVoltage, 0);
    mPlots["audio"]->addData(ld.audio, 2);
    mPlots["acc"]->addData(ld.acc, 3);
    mPlots["gyro"]->addData(ld.gyro, 3);
}

void GraphWindow::clearGraphs(void)
{
    for (auto& p : mPlots)
        p.second->clearData();
}

void GraphWindow::setActiveLpms(int openMatId)
{
    activeOpenMatId = openMatId;
}


void GraphWindow::setXAxisAutoScrolling(bool autoscrolling)
{
    for (auto& p : mPlots)
        p.second->setXAxisAutoScrolling(autoscrolling);
}

void GraphWindow::setYAxisAutoScaling(bool autoscaling)
{
    for (auto& p : mPlots)
        p.second->setYAxisAutoScaling(autoscaling);
}
