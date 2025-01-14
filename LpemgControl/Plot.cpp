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

#include "Plot.h"

#include <iostream>

#include <QGridLayout>

#include <qwt_dyngrid_layout.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_magnifier.h>
#include <qwt_scale_widget.h>

using namespace std;

Plot::Plot(string title, string xAxis, string yAxis,
    string aName, string bName, string cName, string dName,
    int color, int maxData, int nCurves,
    float yMin, float yMax,
    QwtLegend *legend_,
    QWidget *parent)
    : QwtPlot(parent),
    defaultYMin(yMin),
    defaultYMax(yMax),
    yMax(yMax),
    yMin(yMin),
    xAxisAutoScrolling(false),
    yAxisAutoScaling(false),
    legend(legend)
{
    rescaleCount = 0;
    this->nCurves = nCurves;

    if (maxData < MAX_DATA) {
        this->maxData = maxData;
    }
    else {
        // The code below uses maxData where it should be using this->maxData.
        this->maxData = maxData = MAX_DATA;
    }

    setAutoFillBackground(true);
    setPalette(QPalette(QColor(255, 255, 255)));

    this->legend = legend = new QwtLegend();
    QwtDynGridLayout *dgl = qobject_cast<QwtDynGridLayout *>(legend->contentsWidget()->layout());
    dgl->setMaxColumns(1);
    legend->setMinimumWidth(500);
    legend->setFixedWidth(500);
    legend->contentsWidget()->layout()->setAlignment(Qt::AlignLeft);
    legend->contentsWidget()->layout()->setContentsMargins(50, 11, 75, 11);
    insertLegend(legend, QwtPlot::RightLegend, 1.0);

    QwtText qwtTitle(title.c_str());

    QFont f;
    f.setUnderline(true);
    f.setPointSize(9);
    f.setBold(false);

    qwtTitle.setFont(f);

    QwtText t0(QString(xAxis.c_str()));
    QwtText t1(QString(yAxis.c_str()));

    f.setUnderline(false);
    t0.setFont(f);
    t1.setFont(f);

    setAxisFont(xBottom, f);
    setAxisFont(yLeft, f);

    setAxisTitle(xBottom, t0);
    setAxisScale(xBottom, 0.0, (float)maxData);
    setAxisTitle(yLeft, t1);

    setAxisScale(yLeft, defaultYMin, defaultYMax);
    setAxisMaxMajor(yLeft, 5);

    QwtPlot::setAutoReplot();

    (void) new QwtPlotPanner(canvas());
    (void) new QwtPlotMagnifier(canvas());

    ((QwtPlotCanvas*)canvas())->setLineWidth(1);
    ((QwtPlotCanvas*)canvas())->setFrameStyle(QFrame::NoFrame);
    ((QwtPlotCanvas*)canvas())->setFrameStyle(QFrame::Box | QFrame::Plain);

    QPalette canvasPalette(Qt::white);
    canvasPalette.setColor(QPalette::Foreground, QColor(0, 0, 0));
    canvas()->setPalette(canvasPalette);

    QwtScaleWidget *scaleWidget = axisWidget(QwtPlot::yLeft);
    scaleWidget->scaleDraw()->setMinimumExtent(50);

    scaleWidget = axisWidget(QwtPlot::yRight);
    scaleWidget->scaleDraw()->setMinimumExtent(200);

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableX(true);
    grid->enableY(true);
    grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->setMinorPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->attach(this);

    curveName[0] = aName;
    curveName[1] = bName;
    curveName[2] = cName;
    curveName[3] = dName;

    QPen redPen(Qt::red);
    redPen.setWidth(0);
    QPen greenPen(Qt::green);
    greenPen.setWidth(1);
    QPen bluePen(Qt::blue);
    bluePen.setWidth(1);
    QPen magentaPen(Qt::magenta);
    magentaPen.setWidth(1);

    for (int i = 0; i < nCurves; i++) {
        Curve c;

        c.qwtCurve = new QwtPlotCurve(curveName[i].c_str());
        c.qwtCurve->setTitle(QString(curveName[i].c_str()) + QString(" = +00.00"));
        c.nData = 0;
        c.xPos = 0;
        c.first = true;

        for (int j = 0; j < maxData; j++)
            c.xData[j] = (float)j;
        switch (i) {
        case 2:
            c.qwtCurve->setPen(redPen);
            break;

        case 1:
            c.qwtCurve->setPen(greenPen);
            break;

        case 0:
            c.qwtCurve->setPen(bluePen);
            break;

        default:
            c.qwtCurve->setPen(magentaPen);
            break;
        }

        c.qwtCurve->attach(this);
        c.qwtCurve->setRawSamples(c.xData, c.yData, c.nData);

        QString text;
        float y = 0.0f;
        text.sprintf("%+08.3f\t", y);
        c.qwtCurve->setTitle(QString(curveName[i].c_str()) + QString(" = ") + text);

        curves.push_back(c);
    }
    setXMarker("");

    clearData();

    textRedraw[0] = textRedraw[1] = textRedraw[2] = textRedraw[3] = 0;
}

QSize Plot::minimumSizeHint() const
{
    return QSize(150, 100);
}

void Plot::addData(float y, int i)
{
    if (i < nCurves) {
        if (curves[i].xPos > maxData - 1) {
            curves[i].xPos = 0;
            curves[i].nData = maxData;
            curves[i].first = false;
        }

        if (curves[i].first == true) curves[i].nData = curves[i].xPos + 1;

        curves[i].xData[curves[i].xPos] = (float)curves[i].xPos;
        curves[i].yData[curves[i].xPos] = (float)y;

        setData(i);

        if (markerX.size() > 0) markerX[0]->setXValue((float)curves[i].xPos);

        ++curves[i].xPos;

        if (textRedraw[i] > 1) {
            textRedraw[i] = 0;

            QString text;
            text.sprintf("%+08.3f\t", y);
            curves[i].qwtCurve->setTitle(QString(curveName[i].c_str()) + QString(" = ") + text);
        }
        else {
            ++textRedraw[i];
        }
    }
}

void Plot::addData(const float *y, int n)
{
    for (int i = 0; i < n; ++i)
    {
        if (xAxisAutoScrolling)
        {
            memcpy(curves[i].yData, curves[i].yData + 1, (maxData - 1)*sizeof(double));
            curves[i].yData[maxData - 1] = (double)y[i];
            // Scrolling
            //setAxisScale(QwtPlot::xBottom, curves[i].xData[0], curves[i].xData);
            curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, maxData);
            markerX[0]->setXValue(0);
        }
        else
        {
            if (curves[i].xPos > maxData - 1) {
                curves[i].xPos = 0;
                curves[i].nData = maxData;
                curves[i].first = false;
            }

            if (curves[i].first == true) curves[i].nData = curves[i].xPos + 1;

            curves[i].xData[curves[i].xPos] = (float)curves[i].xPos;
            curves[i].yData[curves[i].xPos] = (float)y[i];

            setData(i);

            if (markerX.size() > 0) markerX[0]->setXValue((float)curves[i].xPos);

            ++curves[i].xPos;
        }


        if (textRedraw[i] > 5) {
            textRedraw[i] = 0;
            QString text;
            text.sprintf("%+08.3f\t", y[i]);
            curves[i].qwtCurve->setTitle(QString(curveName[i].c_str()) + QString(" = ") + text);
        }
        else {
            ++textRedraw[i];
        }




        if (yAxisAutoScaling)
        {
            if (y[i] < yMin)
            {
                yMin = y[i];
                setAxisScale(yLeft, yMin, yMax);
                rescaleCount = 0;
            }
            else if (y[i] > yMax)
            {
                yMax = y[i];
                setAxisScale(yLeft, yMin, yMax);
                rescaleCount = 0;
            }
        }
    }
    if (yAxisAutoScaling)
    {
        if (rescaleCount++ > 800)
        {
            setAxisScale(yLeft, yMin, yMax);
            yMin = 0;
            yMax = 0;
            rescaleCount = 0;
        }
    }
}

void Plot::setXMarker(string text)
{
    QwtPlotMarker* m;

    m = new QwtPlotMarker();
    m->setLabelAlignment(Qt::AlignRight | Qt::AlignTop);
    m->setLineStyle(QwtPlotMarker::VLine);
    m->setLinePen(QPen(Qt::gray, 0, Qt::SolidLine));
    m->setXValue(0);

    m->attach(this);

    markerX.push_back(m);
}

void Plot::clearXMarker(void)
{
    for (unsigned int i = 0; (int)i < markerX.size(); i++) {
        markerX[i]->detach();
        delete markerX[i];
    }

    markerX.clear();
}

void Plot::setData(int i)
{
    if (i < nCurves) {
        curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData);
    }
}

void Plot::clearData(void)
{
    for (int i = 0; i < nCurves; i++) {
        curves[i].nData = 0;
        curves[i].first = true;

        for (int j = 0; j < maxData; j++) {
            //curves[i].xData[j] = 0.0f;
            curves[i].yData[j] = 0.0f;
        }

        curves[i].nData = 0;
        curves[i].xPos = 0;

        curves[i].qwtCurve->setRawSamples(curves[i].xData, curves[i].yData, curves[i].nData);
    }

    markerX[0]->setXValue(0);

    setAxisScale(xBottom, 0.0, (float)maxData);
    setAxisScale(yLeft, defaultYMin, defaultYMax);
}

void Plot::setXAxisAutoScrolling(bool autoScrolling)
{
    xAxisAutoScrolling = autoScrolling;
}

void Plot::setYAxisAutoScaling(bool autoscaling)
{
    yAxisAutoScaling = autoscaling;
    if (!yAxisAutoScaling)
    {
        rescaleCount = 0;
        yMin = 99999;
        yMax = -99999;
        setAxisScale(yLeft, defaultYMin, defaultYMax);
    }
}