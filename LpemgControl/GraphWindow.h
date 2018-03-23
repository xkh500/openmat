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

#ifndef GRAPH_WINDOW
#define GRAPH_WINDOW

#include <map>
#include <string>

#include <QWidget>
#include "qwt_legend.h"

#include "Plot.h"
#include "EmgData.h"


/* Contains the sensor data plots. */
class GraphWindow : public QWidget
{
    Q_OBJECT

public:
    enum class Mode {
        Emmg,
		Motion
    };

    GraphWindow(QWidget *parent = 0);
    void clearGraphs(void);
    void setActiveLpms(int openMatId);
    void setMode(const Mode& m);
    void setXAxisAutoScrolling(bool autoscrolling);
    void setYAxisAutoScaling(bool autoscaling);

    public slots:
    void plotDataSet(const EmgData& ld);

private:
    std::map<std::string, Plot *> mPlots;

    int activeOpenMatId;
    Mode mode;
};

#endif
