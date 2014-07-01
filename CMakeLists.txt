# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(gazeStabilizer)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

find_package(OpenCV)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include(YarpInstallationHelpers)   # pick up yarp's cmake scripts

file(GLOB source src/*.cpp)
file(GLOB header src/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(gazeStabilizer ${source} ${header} )

target_link_libraries(gazeStabilizer ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS gazeStabilizer DESTINATION bin)
icubcontrib_add_uninstall_target()

### create a single target that installs all applications at once
file(GLOB conf     app/conf/*.ini)
file(GLOB scripts  app/scripts/*.template)
yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/gazeStabilizer)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
