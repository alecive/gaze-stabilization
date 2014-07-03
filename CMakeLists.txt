# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(gazeStabilization)

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

file(GLOB source_gs modules/gazeStabilizer/*.cpp)
file(GLOB header_gs modules/gazeStabilizer/*.h)
file(GLOB source_tc modules/torsoController/*.cpp)
file(GLOB header_tc modules/torsoController/*.h)
file(GLOB source_ge modules/gazeEvaluator/*.cpp)
file(GLOB header_ge modules/gazeEvaluator/*.h)

source_group("Source Files" FILES ${source_gs} ${source_tc} ${source_ge})
source_group("Header Files" FILES ${header_gs} ${header_tc} ${header_ge})

add_executable(gazeStabilizer  ${source_gs} ${header_gs})
add_executable(torsoController ${source_tc} ${header_tc})
add_executable(gazeEvaluator   ${source_ge} ${header_ge})

target_link_libraries(gazeStabilizer  ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
target_link_libraries(torsoController ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
target_link_libraries(gazeEvaluator   ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS gazeStabilizer  DESTINATION bin)
install(TARGETS torsoController DESTINATION bin)
install(TARGETS gazeEvaluator   DESTINATION bin)
icubcontrib_add_uninstall_target()

### create a single target that installs all applications at once
file(GLOB conf     app/conf/*.ini)
file(GLOB scripts  app/scripts/*.template)
yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/gazeStabilization)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
