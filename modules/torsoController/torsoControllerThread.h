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
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <iostream>
#include <string>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;

using namespace std;

class torsoControllerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    int verbosity;  // Flag that manages verbosity
    string name;    // Name of the module (to change port names accordingly)  
    string robot;   // Name of the robot (to address both icub and icubSim):

    // Driver for classical interfaces
    PolyDriver       ddH; // right arm device driver
    PolyDriver       ddT; // left arm  device driver

    // Classical interfaces - TORSO
    IEncoders          *iencsT;
    IPositionControl   *iposT;
    IVelocityControl2  *ivelT;
    IControlLimits     *ilimT;
    Vector             *encsT;
    int jntsT;

    // Classical interfaces - HEAD
    IEncoders          *iencsH;
    IPositionControl   *iposH;
    IVelocityControl2  *ivelH;
    IControlLimits     *ilimH;
    Vector             *encsH;
    int jntsH;

    // 
    iCubEye *eyeR;
    iCubEye *eyeL;

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
    torsoControllerThread(int _rate, string _name, string _robot, int _v);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
};
