/***********************************************************************
** Copyright (C) 2013 LP-Research
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

#ifndef SINUS_PLOT
#define SINUS_PLOT

#include <vector>

#include <QWidget>
#include <QSize>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_legend.h>

#define MAX_DATA 512

#define STD_SPACING 5

/* Contains all information for one data graph curve. */
class Curve
{
public:
    QwtPlotCurve* qwtCurve;
    int nData;
    double xData[MAX_DATA];
    double yData[MAX_DATA];
    int xPos;
    bool first;
};

/* QwtPlot window that contains a data graph. */
class Plot : public QwtPlot
{
    Q_OBJECT

public:
    Plot(std::string title, std::string xAxis, std::string yAxis,
        std::string aName, std::string bName, std::string cName, std::string dName,
        int color, int maxData, int nCurves,
        float yMin, float yMax,
        QwtLegend* legend = 0,
        QWidget* parent = 0);
    void addData(float y, int i);
    void addData(const float* y, int n);
    void clearData(void);
    void setData(int i);
    void setXMarker(std::string text);
    void clearXMarker(void);
    void setXAxisAutoScrolling(bool autoScrolling);
    void setYAxisAutoScaling(bool autoscaling);
    QSize minimumSizeHint() const;
    QwtLegend* getLegend() { return legend; }
public slots:
    void show() { legend->show(); QWidget::show(); }
    void hide() { QWidget::hide(); legend->hide(); }

private:
    std::vector<Curve> curves;
    int maxData;
    int nCurves;
    std::vector<QwtPlotMarker*> markerX;
    std::string curveName[4];
    int textRedraw[4];
    float yMax;
    float yMin;
    float defaultYMax;
    float defaultYMin;
    int rescaleCount;
    bool xAxisAutoScrolling;
    bool yAxisAutoScaling;
    QwtLegend* legend;
};

#endif
