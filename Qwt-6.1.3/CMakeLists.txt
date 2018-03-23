cmake_minimum_required(VERSION 2.8.12)

project (qwt)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

FILE(GLOB sources
    src/*.cpp
)

FILE(GLOB headers
    src/*.h
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS
				Widgets
				OpenGL
				Core
				PrintSupport
				Concurrent
				Svg)

add_library(qwt SHARED ${sources} ${headers})
target_link_libraries(qwt
    Qt5::Widgets Qt5::OpenGL Qt5::Core Qt5::PrintSupport Qt5::Concurrent Qt5::Svg)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS(-DQWT_DLL)
    SET_TARGET_PROPERTIES (qwt PROPERTIES DEFINE_SYMBOL "QWT_MAKEDLL")
endif()
