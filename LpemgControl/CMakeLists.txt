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

project (LpemgControl)

if (COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0020 NEW)
endif(COMMAND cmake_policy)

include_directories(${Boost_INCLUDE_DIR})
include_directories("${PUGIXML_PATH}/src")
include_directories("${EIGEN_PATH}")
include_directories("../LpSensor")
include_directories("${QWT_PATH}/src")

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS(-DUSE_EIGEN)
	ADD_DEFINITIONS(-DQWT_DLL)
	ADD_DEFINITIONS(-DNOMINMAX)
	ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN_STATICALLY)
	ADD_DEFINITIONS(-D_WIN32_WINNT=0x05010200)
	ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
	ADD_DEFINITIONS(-DPLATFORM_X86)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

	if (SHOW_CONSOLE)
		ADD_DEFINITIONS(-DSHOW_CONSOLE)
	endif()

	if (CMAKE_BUILD_TYPE STREQUAL "Release")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET(SHOW_CONSOLE TRUE)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fPIC")
endif()

set(sources
	MainWindow.cpp
	Plot.cpp
	GraphWindow.cpp
	main.cpp
	StdRedirector.cpp
	SensorGuiContainer.cpp
	DiscoveryItem.cpp
	DiscoveryTree.cpp
	RescanDialog.cpp
	PlayControl.cpp

	../LpSensor/LpMatrix.c

	LpmsControl.rc

	${PUGIXML_PATH}/src/pugixml.cpp
)

set(headers
    MainWindow.h
    Plot.h
    GraphWindow.h
    StdRedirector.h
    SensorGuiContainer.h
    DiscoveryItem.h
    DiscoveryTree.h
    RescanDialog.h
    VerticalBarGraph.h
    PlayControl.h
    ../LpSensor/LpMatrix.h
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core OpenGL)
find_package(OpenGL REQUIRED)

if (CMAKE_SYSTEM_NAME STREQUAL "Windows" AND NOT SHOW_CONSOLE)
	add_executable(LpemgControl WIN32 ${sources} ${headers} ${moc_sources})
else()
	add_executable(LpemgControl ${sources} ${headers} ${moc_sources})
endif()

target_link_libraries(LpemgControl ${OPENGL_LIBRARIES})
target_link_libraries(LpemgControl Qt5::Widgets Qt5::Core Qt5::OpenGL)
target_link_libraries(LpemgControl qwt)
target_link_libraries(LpemgControl LpSensor)

# This appears to be a problem in the combination of Ubuntu's (and maybe
# other vendors') linker settings and cmake, anyway, the fix is simple.
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(LpemgControl -pthread)
endif()

add_custom_command(TARGET LpemgControl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/icons $<TARGET_FILE_DIR:LpemgControl>/icons)
add_custom_command(TARGET LpemgControl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LpmsStyles.qss $<TARGET_FILE_DIR:LpemgControl>)
