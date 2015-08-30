# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(armFollower)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

set(sources armFollower.cpp)
source_group("Source Files" FILES ${sources})

add_executable(armFollower ${sources})
target_link_libraries(armFollower ${YARP_LIBRARIES} ctrlLib)
install(TARGETS armFollower DESTINATION bin)

