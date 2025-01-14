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

#ifndef FIELD_MAP_CONTAINER
#define FIELD_MAP_CONTAINER

#include <QtGui>
#include <QGLWidget>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "FieldMapDisplay.h"
#include "FieldModuleIndicator.h"

class FieldMapContainer : public QWidget {
    Q_OBJECT

public:
    FieldMapContainer(QWidget* parent = 0);
    void updateFieldMap(float magField[ABSMAXPITCH][ABSMAXROLL][ABSMAXYAW][3],
        float hardIronOffset[3], float softIronMatrix[3][3],
        float fieldStrength, float currentField[3]);
    void updateCurrentField(float fieldRadius, float currentField[3]);

    FieldMapDisplay* fmd;
    FieldModuleIndicator* fmi;
    QCheckBox* originalFieldCheck;
    QCheckBox* correctedFieldCheck;
    QCheckBox* ellipsoidCheck;

    QLabel* fieldRadiusLabel;
    QLabel* fieldOffsetLabel[3];
    QLabel* fieldDistortionLabel[3];

    public slots:
    void updateGraphs(int v);
};

#endif