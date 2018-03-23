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

#ifndef DISCOVERY_ITEM
#define DISCOVERY_ITEM

#include <QLabel>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QWidget>

#include <string>

#include "LpmsDefinitions.h"

class DiscoveryItem : public QWidget
{
    Q_OBJECT

public:
    QTreeWidgetItem* treeItem;
    QLabel* deviceIdItem;
    QLabel* deviceTypeItem;
    QLabel* interfaceTypeItem;
    int deviceType;
    std::string deviceId;

    DiscoveryItem(QTreeWidget* tree, std::string deviceId, int deviceType, int interfaceType);
    ~DiscoveryItem(void);
    int getDeviceType(void);
    std::string getDeviceId(void);
};

#endif