# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(gazeEvaluator)

file(GLOB source *.cpp)
file(GLOB header *.h)

find_package(OpenCV REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(gazeEvaluator ${source} ${header} )

target_link_libraries(gazeEvaluator ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS gazeEvaluator DESTINATION bin)

