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

project(OpenMAT)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

option(LPSENSOR_BUILD_GUI "If this is set LpmsControl stuff will be built, provided Qt is found." ON)

option(LPSENSOR_SET_TARGET_DIRS "If this option is set, this will override the output directories, writing everything in a single dir" ON)
set (TARGET_DIR "${CMAKE_BINARY_DIR}/OpenMAT" CACHE STRING
    "Directory where executables and libraries are written to.")

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_PREFIX_PATH "C:/Qt/5.9.1/msvc2017_64")
	# set(BOOST_ROOT "C:/Opt/boost_1_65_1" CACHE STRING "Top directory of Boost installation")
    set(WINDOWS_BOOST_ROOT "C:/Opt/boost_1_65_1" CACHE STRING "")
    set(BOOST_ROOT "${WINDOWS_BOOST_ROOT}" CACHE STRING "")
endif()
find_package(Boost 1.65.1 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARYDIR})

if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(BUILD_ARCHITECTURE "64-bit" CACHE STRING "")
else()
    set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
endif()
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

if (CMAKE_BUILD_TYPE STREQUAL "Debug"
    OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"
    OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set (SHOW_CONSOLE TRUE CACHE BOOL "Whether to show the console window")
else()
    set (SHOW_CONSOLE FALSE CACHE BOOL "Whether to show the console window")
endif()

set(BUILD_OPENSOURCE CACHE BOOL "")

# Build everything into a single directory
if (LPSENSOR_SET_TARGET_DIRS)
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TARGET_DIR})
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TARGET_DIR})
endif()

# The subdirectories use these to point to the libraries.
set(PUGIXML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Pugixml-1.8")
set(EIGEN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Eigen-3")
set(QWT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Qwt-6.1.3")

# Don't build GUI stuff if Qt not found

add_subdirectory(LpSensor)
if (LPSENSOR_BUILD_GUI)
    find_package(Qt5 COMPONENTS
                 # Needed explicitly for LpmsControl:
                 Widgets Core OpenGL
                 # Needed for qwt:
                 PrintSupport Concurrent Svg)

    if(${Qt5Widgets_FOUND})
        try_compile(QT_WORKS ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/qt_compile_test.cpp
            LINK_LIBRARIES Qt5::Widgets)

        if (NOT ${QT_WORKS})
            message(STATUS "Incompatible Qt version found, not building GUI parts.")
        else ()
            add_subdirectory(Qwt-6.1.3)
            add_subdirectory(LpmsControl LpmsControl)
            add_subdirectory(LpemgControl LpemgControl)		
            add_subdirectory(LpSensorCWrapper LpSensorCWrapper) 
        endif()
    endif()
endif()
