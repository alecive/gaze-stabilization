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

#include <iostream>
#include <string>
#include <stdarg.h>
#include <deque>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class headFollowerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    int verbosity;      // Flag that manages verbosity
    string name;        // Name of the module (to change port names accordingly)  
    string robot;       // Name of the robot (to address both icub and icubSim)
    string if_mode;     // Interface mode to use (either vel1 or vel2)

    // Classical interfaces - HEAD
    PolyDriver         *ddH;    // head device driver
    IEncoders          *iencsH;
    IPositionControl   *iposH;
    IVelocityControl   *ivelH1;
    IVelocityControl2  *ivelH2;
    IControlMode2      *imodH;
    Vector             *encsH;
    int jntsH;

    bool isRunning;         // Flag to manage the status of the thread
    
    // Input from the torsoController
    BufferedPort<Bottle>  inGlassPort;   // port for reading from the torsoController
    Bottle               *inGlassBottle; // bottle used for the port

    /**
    * Moves the neck joint in velocity mode
    **/
    bool moveNeck(const Vector &_dq_H);
    
    /**
     * Check the state of each joint to be controlled
     * @param  jointsToSet vector of integers that defines the joints to be set
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return             true/false if success/failure
     */
    bool areJointsHealthyAndSet(VectorOf<int> &jointsToSet,const string &_s);

    /**
     * Changes the control modes of the torso to either position or velocity
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return    true/false if success/failure
     */
    bool setHeadCtrlModes(const VectorOf<int> &jointsToSet,const string &_s);

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
     * Closes properly a given port
    **/
    void closePort(yarp::os::Contactable &_port);

public:
    // CONSTRUCTOR
    headFollowerThread(int _rate, string &_name, string &_robot, int _v, string &_if_mode, bool _isRunning);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    // START AND STOP THE STABILIZATION
    bool startFollowing();
    bool  stopFollowing();
    // SET IF_MODE ON THE FLY
    bool set_if_mode        (const string &_ifm);
    // GET IF_MODE
    string get_if_mode()    { return if_mode; };
    // GO HOME
    bool goHome();
};
