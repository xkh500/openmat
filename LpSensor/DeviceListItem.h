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

#ifndef DEVICE_LIST_ITEM
#define DEVICE_LIST_ITEM

#include <string>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

// Item for automatic device discovery
class DeviceListItem {
public:
    // Constructor
    DeviceListItem(const char *deviceId, int deviceType)
        : deviceId(deviceId),
          deviceType(deviceType)
    {
    }

    // Constructor
    DeviceListItem(void)
    {
    }

    // Device ID
    std::string deviceId;

    // Device type
    int deviceType;
};

// Contains all discovered devices
class LpmsDeviceList {
public:
    // Constructor
    LpmsDeviceList(void)
    {
    }

    // Adds a device
    void push_back(DeviceListItem i)
    {
        devices.push_back(i);
    }

    void push_back(const char *deviceId, int deviceType)
    {
        push_back(DeviceListItem(deviceId, deviceType));
    }

    // Clears device list
    void clear(void)
    {
        devices.clear();
    }

    // Gets number of known devices
    size_t getNDevices() const {
        return devices.size();
    }

    // Retrieves ID of device
    const char *getDeviceId(int i) const {
        return devices[i].deviceId.c_str();
    }

    // Retrieves device type
    int getDeviceType(int i) const {
        return devices[i].deviceType;
    }

    // Writes devices to file
    void writeToFile(const char *fn)
    {
        FILE *f;
#ifdef _WIN32
        fopen_s(&f, fn, "w+");
#else
        f = fopen(fn, "w+");
#endif

        if (f == NULL) return;

        for (unsigned i = 0; i < getNDevices(); ++i) {
            fprintf(f, "%s %d\n", devices[i].deviceId.c_str(), devices[i].deviceType);
        }

        fclose(f);
    }

    // Returns index of device with the same id if already contained in list, -1 otherwise
    int findDevice(const std::string& id) const {
        for (unsigned i = 0; i < getNDevices(); ++i) {
            if (devices[i].deviceId == id)
                return i;
        }
        return -1;
    }

private:
    std::vector<DeviceListItem> devices;
};

#endif
