/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <yarp/os/BufferedPort.h>

#include <yarp/math/SVD.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinHlp.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/ctrl/math.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <string>
#include <stdarg.h>
#include <deque>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class gazeStabilizerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    int verbosity;      // Flag that manages verbosity
    string name;        // Name of the module (to change port names accordingly)  
    string robot;       // Name of the robot (to address both icub and icubSim)
    string if_mode;     // Interface mode to use (either vel1 or vel2)
    string src_mode;    // Source to use (either torso velocities through a port or the intetial sensor)

    // Classical interfaces - HEAD
    PolyDriver         *ddH;    // head device driver
    IEncoders          *iencsH;
    IPositionControl   *iposH;
    IVelocityControl   *ivelH1;
    IVelocityControl2  *ivelH2;
    IControlLimits     *ilimH;
    Vector             *encsH;
    int jntsH;

    // Classical interfaces - TORSO
    PolyDriver         *ddT;     // torso device driver
    IEncoders          *iencsT;
    IControlLimits     *ilimT;
    Vector             *encsT;
    int jntsT;

    // Eyes' and inertial's kinematics
    iCubEye             *eyeR;
    iCubEye             *eyeL;
    iCubHeadCenter      *neck;
    iCubInertialSensor  *IMU;
    iKinChain           *chainNeck;
    iKinChain           *chainEyeL;
    iKinChain           *chainEyeR;
    iKinChain           *chainIMU;

    // Internal matrices and variables
    Vector xFP_R;
    Matrix J_E;
    bool isRunning;         // Flag to manage the status of the thread

    // Input from the torsoController
    BufferedPort<Bottle>  *inTorsoPort;   // port for reading from the torsoController
    Bottle                *inTorsoBottle; // bottle used for the port

    BufferedPort<Bottle>  *inIMUPort;    // port for reading from the inertial sensor
    Bottle                *inIMUBottle;  // bottle used for the port

    /**
    * Updates a kinematic chain belonging to an eye. It can be either an iCubEye, 
    * or an iCubHeadCenter.
    **/
    void updateEyeChain(iKinChain &_eye, const string _eyeType);

    /**
    * Updates the iCubInertialSensor kinematic chain. Not that useful, though
    * it has been implemented only to keep the syntax used for updateEyeChain.
    **/
    void updateIMUChain(iKinChain &_imu);

    void run_torsoMode();
    void run_inertialMode();

    bool moveEyes(const Vector &_dq_E);

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
     * Closes properly a given port
    **/
    void closePort(yarp::os::Contactable *_port);

public:
    // CONSTRUCTOR
    gazeStabilizerThread(int _rate, string _name, string _robot, int _v, string _if_mode, string _src_mode);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    // START AND STOP THE STABILIZATION
    bool startStabilization();
    bool  stopStabilization();
};
