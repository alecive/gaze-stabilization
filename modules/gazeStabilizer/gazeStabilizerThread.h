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

using namespace yarp;
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
    string robot;       // Name of the robot (to address both icub and icubSim):

    // Classical interfaces - HEAD
    PolyDriver         *ddH;    // head device driver
    IEncoders          *iencsH;
    IPositionControl   *iposH;
    IVelocityControl2  *ivelH;
    Vector             *encsH;
    int jntsH;

    // Classical interfaces - TORSO
    PolyDriver         *ddT;     // torso device driver
    IEncoders          *iencsT;
    Vector             *encsT;
    int jntsT;

    // Eyes' kinematics
    iCubEye        *eyeR;
    iCubEye        *eyeL;
    iCubHeadCenter *neck;
    iKinChain      *chainNeck;
    iKinChain      *chainEyeL;
    iKinChain      *chainEyeR;

    // Cartesian Helper
    CartesianHelper cartHlp;

    // Input from the torsoController
    BufferedPort<Bottle>  *inTorsoPort;   // port for reading from the torsoController
    Bottle                *inTorsoBottle; // bottle used for the port

    void updateEyeChain(iKinChain &_eye, const string _eyeType);

    /**
    * Aligns head joints bounds with current onboard bounds.
    * Returns a matrix containing the actual limits
    */
    Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                             const double eyeTiltMin, const double eyeTiltMax);

    /**
    * Copies joints bounds from first chain to second chain
    */
    void copyJointsBounds(iKinChain *ch1, iKinChain *ch2);

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
    gazeStabilizerThread(int _rate, string _name, string _robot, int _v);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
};
