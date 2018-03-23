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

project (LpSensorCWrapper)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0020 NEW)
endif(COMMAND cmake_policy)

set(BUILD_ARCHITECTURE "32-bit" CACHE STRING "")
set_property(CACHE BUILD_ARCHITECTURE PROPERTY STRINGS "32-bit" "64-bit")

SET(CMAKE_BUILD_TYPE "Release")

include_directories("../LpSensor")

ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ADD_DEFINITIONS(-DDLL_EXPORT)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET(SHOW_CONSOLE TRUE)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fPIC")
endif() 

if (${WIN32} MATCHES TRUE)
	if (CMAKE_BUILD_TYPE STREQUAL "Release")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /O2")
	endif()
endif()

add_library(LpSensorCWrapper SHARED LpSensorCWrapper.cpp)
target_link_libraries(LpSensorCWrapper LpSensor)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_custom_command(TARGET LpSensorCWrapper POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/LpSensorCWrapper.lib $<TARGET_FILE_DIR:LpSensorCWrapper>)
endif()