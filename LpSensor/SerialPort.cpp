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

#include "SerialPort.h"
using namespace std;

Serial::Serial() :
portNo(0),
connected(false)
{
}

Serial::~Serial()
{
    // Check if we are connected before trying to disconnect
    close();
}

bool Serial::open(int portno)
{
    if (IsConnected())
        CloseHandle(this->hSerial);
    portNo = portno;
    stringstream ss;
    ss << "\\\\.\\COM" << portNo;
    string portName = ss.str();
    // We're not yet connected
    this->connected = false;

    // Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0, //FILE_ATTRIBUTE_NORMAL,
        NULL);

    // Check if the connection was successfull
    if (this->hSerial == INVALID_HANDLE_VALUE)
    {
        printf("[Serial] ERROR: Invalide handle value\n");
        CloseHandle(this->hSerial);
        return false;
    }
    else
    {
        char mode_str[128];
        strcpy(mode_str, "baud=921600");
        strcat(mode_str, " data=8");
        strcat(mode_str, " parity=n");
        strcat(mode_str, " stop=1");
        strcat(mode_str, " dtr=on rts=on");
        DCB port_settings;
        memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
        port_settings.DCBlength = sizeof(port_settings);

        if (!BuildCommDCBA(mode_str, &port_settings))
        {
            printf("unable to set comport dcb settings\n");
            CloseHandle(this->hSerial);
            return false;
        }
        if (!BuildCommDCBA(mode_str, &port_settings))
        {
            printf("unable to set comport dcb settings\n");
            CloseHandle(this->hSerial);
            return false;
        }

        if (!SetCommState(this->hSerial, &port_settings))
        {
            printf("unable to set comport cfg settings\n");
            CloseHandle(this->hSerial);
            return false;
        }

        COMMTIMEOUTS Cptimeouts;

        Cptimeouts.ReadIntervalTimeout = MAXDWORD;
        Cptimeouts.ReadTotalTimeoutMultiplier = 0;
        Cptimeouts.ReadTotalTimeoutConstant = 0;
        Cptimeouts.WriteTotalTimeoutMultiplier = 0;
        Cptimeouts.WriteTotalTimeoutConstant = 0;

        if (!SetCommTimeouts(this->hSerial, &Cptimeouts))
        {
            printf("unable to set comport time-out settings\n");
            CloseHandle(this->hSerial);
            return false;
        }
    }

    this->connected = true;

    return true;
}

bool Serial::close()
{
    if (this->connected)
    {
        // We're no longer connected
        this->connected = false;
        // Close the serial handler
        return CloseHandle(this->hSerial);
    }

    return true;
}

int Serial::ReadData(unsigned char *buffer, unsigned int nbChar)
{
    if (!IsConnected())
        return 0;
    // Number of bytes we'll have read
    DWORD bytesRead;
    // Number of bytes we'll really ask to read
    unsigned int toRead;

    int n;
    ReadFile(this->hSerial, buffer, nbChar, (LPDWORD)((void *)&n), NULL);
    return n;
    // Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    // Check if there is something to read
    if (this->status.cbInQue > 0)
    {
        // If there is we check if there is enough data to read the required number
        // of characters, if not we'll read only the available characters to prevent
        // locking of the application.
        if (this->status.cbInQue > nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        // Try to read the require number of chars, and return the number of read bytes on success
        if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
        {
            return bytesRead;
        }

    }

    // If nothing has been read, or that an error was detected return -1
    return -1;

}


bool Serial::WriteData(unsigned char *buffer, unsigned int nbChar)
{
    if (!IsConnected())
    {
        std::cout << "dongle not connected\n";
        return false;
    }
    DWORD bytesSend;

    // Try to write the buffer on the Serial port
    if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        std::cout << "Write error\n";
        // In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);

        return false;
    }
    else
        return true;
}

bool Serial::IsConnected()
{
    // Simply return the connection status
    return this->connected;
}