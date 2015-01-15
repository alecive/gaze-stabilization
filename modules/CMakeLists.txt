# Copyright: (C) 2014 RobotCub Consortium
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(OpenCV REQUIRED)

add_subdirectory(gazeEvaluator)
add_subdirectory(gazeStabilizer)
add_subdirectory(torsoController)
add_subdirectory(imuIdentifier)
add_subdirectory(iKinGazeCtrlMOD)
