#***********************************************************************
#* Copyright (C) 2018 LP-RESEARCH Inc.
#* All rights reserved.
#* Contact: info@lp-research.com
#*
#* This file is part of the Open Motion Analysis Toolkit (OpenMAT).
#*
#* Redistribution and use in source and binary forms, with 
#* or without modification, are permitted provided that the 
#* following conditions are met:
#*
#* Redistributions of source code must retain the above copyright 
#* notice, this list of conditions and the following disclaimer.
#* Redistributions in binary form must reproduce the above copyright 
#* notice, this list of conditions and the following disclaimer in 
#* the documentation and/or other materials provided with the 
#* distribution.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
#* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#**********************************************************************/

cmake_minimum_required(VERSION 2.8.3)

project (LpSensor)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 3)
set(PATCH_VERSION 5)
set(SO_VERSION    0)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

OPTION(LPSENSOR_BUILD_LPMS_U "Whether to build LPMS-U support if FTDI library is detected." ON)
OPTION(LPSENSOR_BUILD_LPMS_B "Whether to build LPMS-B, LPMS-B2 and LPMS-BLE support if supported by platform." ON)

OPTION(LPSENSOR_BUILD_STATIC "Whether to build as static library, only useful if included in another project." OFF)

find_path(CLOSED_SOURCE_PATH LpMagnetometerCalibration.c PATHS "../../openmat-closedsource")

set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${BoostLIBRARIES})
include_directories(${PUGIXML_PATH}/src)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Windows SDK
    set(WINDOWS_SDK_PATH "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A" CACHE STRING "")
    
    # FTDI
    set(WINDOWS_FTDI_PATH "C:/Opt/Ftdi" CACHE STRING "")
    
    # PCAN-BasicAPI
    set(WINDOWS_PCAN_PATH "C:/Opt/PcanBasicApi" CACHE STRING "")
    include_directories("${WINDOWS_PCAN_PATH}/Include")
    
    # Silicon labs
    set(WINDOWS_SL_PATH "c:/Opt/SiliconLabs/MCU/USBXpress_SDK/Library/Host/Windows" CACHE STRING "")
    include_directories("${WINDOWS_SL_PATH}")
	
    if (BUILD_ARCHITECTURE STREQUAL "32-bit")
        find_library(FTDI_LIB ftd2xx PATHS "${WINDOWS_FTDI_PATH}/i386")
        find_library(PCAN_BASIC_LIB PCANBasic PATHS "${WINDOWS_PCAN_PATH}/Win32/VC_LIB")
        find_library(SILABS_LIB SiUSBXp PATHS "${WINDOWS_SL_PATH}/x86")
    endif() 

    if (BUILD_ARCHITECTURE STREQUAL "64-bit")
        find_library(FTDI_LIB ftd2xx PATHS "${WINDOWS_FTDI_PATH}/amd64")
        find_library(PCAN_BASIC_LIB PCANBasic PATHS "${WINDOWS_PCAN_PATH}/x64/VC_LIB")
        find_library(SILABS_LIB SiUSBXp PATHS "${WINDOWS_SL_PATH}/x64")
    endif()

    set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
            Ws2_32
            ${PCAN_BASIC_LIB}
            Setupapi
            ${SILABS_LIB}
    )

    if (LPSENSOR_BUILD_LPMS_B)
        set(BUILD_BLUETOOTH TRUE)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
            Bthprops
        )
    endif()

    if (FTDI_LIB AND LPSENSOR_BUILD_LPMS_U)
        SET(BUILD_LPMS_U TRUE)
        include_directories("${WINDOWS_FTDI_PATH}")
        set (EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
            ${FTDI_LIB})
    endif()

    ADD_DEFINITIONS(-DUSE_EIGEN)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
    ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
    ADD_DEFINITIONS(-DPLATFORM_X86)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
    endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_library(BLUETOOTH_LIB bluetooth)
    if (LPSENSOR_BUILD_LPMS_B AND BLUETOOTH_LIB)
        set(BUILD_BLUETOOTH TRUE)
        set(EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
            #libftd2xx.a
            ${BLUETOOTH_LIB}
        )
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11")
endif()

set(sources

    CalibrationData.cpp

    EmgData.cpp
    LpemgIoInterface.cpp
    LpIoBase.cpp
    LpemgSensor.cpp
    LpemgSensorManager.cpp

    LpmsSensorManager.cpp
    LpmsIoInterface.cpp
    LpmsSensor.cpp
    LpMatrix.c
    util.cpp
    ${PUGIXML_PATH}/src/pugixml.cpp
)

if (BUILD_OPENSOURCE)
# if (CLOSED_SOURCE_PATH)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    set(sources
        ${sources}

        ${CLOSED_SOURCE_PATH}/LpMagnetometerCalibration.c
        ${CLOSED_SOURCE_PATH}/LpMagnetometerMAlignment.c
        ${CLOSED_SOURCE_PATH}/CalcMisalignment.c
    )
else()
    set(sources
        ${sources}

        LpMagnetometerCalibrationOS.c
        LpMagnetometerMAlignmentOS.c
        CalcMisalignmentOS.c
    )
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(sources
        ${sources}

        CanEngine.cpp
        LpmsCanIo.cpp
        LpmsRS232.cpp
        LpmsTcp.cpp

        LpmsU2.cpp
        PCANBasicClass.cpp
        SerialPort.cpp
        version.rc
    )
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(sources
        ${sources}

        CanEngineLinux.cpp
        LpmsCanIoLinux.cpp
        LpmsRS232Linux.cpp
    )
endif()

set(headers
    DeviceListItem.h
    ImuData.h
    LpmsDefinitions.h
    LpmsSensorI.h
    LpmsSensorManagerI.h
    MicroMeasure.h
    CanEngine.h
    CalibrationData.h
    LpmsSensorManager.h

    EmgData.h
    LpemgIoInterface.h
    LpemgSensor.h
    LpemgSensorI.h
    LpemgSensorManager.h
    LpemgSensorManagerI.h
    LpIoBase.h
    LpSensorBaseI.h
    SensorData.h

    LpmsCanIo.h
    LpmsIoInterface.h
    LpmsU2.h
    LpmsRS232.h
    LpmsSensor.h
    LpMatrix.h
    RingBuffer.h
    LpmsRegisterDefinitions.h
    PCANBasicClass.h

    SerialPort.h
)

if(DEFINED BUILD_LPMS_U)
    ADD_DEFINITIONS(-DBUILD_LPMS_U)
    set(sources ${sources}
        LpmsU.cpp)
    set(headers ${headers}
        LpmsU.h)
endif()

if(DEFINED BUILD_BLUETOOTH)
    ADD_DEFINITIONS(-DBUILD_BLUETOOTH)
    set(sources ${sources}
        LpemgBluetooth.cpp
    )
    if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(sources ${sources}
        LpmsBBluetooth.cpp
        BleEngine.cpp
        BleUart.c
        BleCmdDef.c
        BleCommands.c
        LpmsBle2.cpp
        )
    elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      set(sources ${sources}
        LpmsBBluetoothLinux.cpp)
    endif()
    set(headers ${headers}
        LpemgBluetooth.h
        LpmsBBluetooth.h
        LpmsBle2.h
        BleEngine.h
        BleUart.h
        BleCmdDef.h
    )
endif()

set (libheaders
    DeviceListItem.h
    ImuData.h
    LpmsDefinitions.h
    LpmsSensorI.h
    LpmsSensorManagerI.h
)

if (LPSENSOR_BUILD_STATIC)
    add_library(LpSensor STATIC ${sources} ${headers})
    add_definitions(-DLPSENSOR_BUILD_STATIC)
else ()
    add_library(LpSensor SHARED ${sources} ${headers})
endif ()
target_link_libraries(LpSensor ${EXTERNAL_LIBRARIES})

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET_TARGET_PROPERTIES(LpSensor PROPERTIES OUTPUT_NAME LpSensorD)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET_TARGET_PROPERTIES (LpSensor PROPERTIES DEFINE_SYMBOL "DLL_EXPORT")
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    include(InstallRequiredSystemLibraries)
    SET(CPACK_GENERATOR "DEB")
    SET(CPACK_PACKAGE_NAME "liblpsensor")
    set(CPACK_PACKAGE_VENDOR "LP-Research Inc. <www.lp-research.com>")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library for communicating and interfacing with LP-Research sensors. (Build20160608)")
    set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/SUMMARY.txt")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
    set(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
    set(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
    set(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
    SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Huei Ee YAP <yap@lp-research.com>") #required
     
    include(CPack)

    install(FILES ${libheaders} DESTINATION "include/lpsensor")

    install(TARGETS ${LIBRARY_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if (BUILD_ARCHITECTURE STREQUAL "64-bit")
        add_custom_command(TARGET LpSensor POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${WINDOWS_SL_PATH}/x64/SiUSBXp.dll $<TARGET_FILE_DIR:LpSensor>)
        add_custom_command(TARGET LpSensor POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${WINDOWS_FTDI_PATH}/amd64/ftd2xx64.dll $<TARGET_FILE_DIR:LpSensor>)
    endif()
    if (BUILD_ARCHITECTURE STREQUAL "32-bit")
        add_custom_command(TARGET LpSensor POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${WINDOWS_SL_PATH}/x86/SiUSBXp.dll $<TARGET_FILE_DIR:LpSensor>)
        add_custom_command(TARGET LpSensor POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${WINDOWS_FTDI_PATH}/i386/ftd2xx.dll $<TARGET_FILE_DIR:LpSensor>)
    endif()
endif()
