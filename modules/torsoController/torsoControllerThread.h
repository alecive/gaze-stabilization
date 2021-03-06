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

#include <iostream>
#include <string>
#include <stdarg.h>
#include <sstream>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace std;

string int_to_string( const int a );

struct wayPoint
{
    string name;
    Vector jntlims;    // torso joints
    Vector vels;       // torso velocities

    wayPoint();
    wayPoint(string _name);
    wayPoint(string _name, Vector _jntlims, Vector _vels);

    /**
    * Copy Operator
    **/
    wayPoint &operator=(const wayPoint &jv);

    /**
    * Print functions
    **/
    void print();
    void printCompact();
};

class torsoControllerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    int verbosity;      // Flag that manages verbosity
    string name;        // Name of the module (to change port names accordingly)  
    string robot;       // Name of the robot (to address both icub and icubSim)
    int iterations;

    // Classical interfaces - TORSO
    PolyDriver          ddT;   // torso device driver
    IPositionControl   *iposT;
    IVelocityControl2  *ivelT;
    IEncoders          *iencsT;
    IControlMode2      *imodT;
    Vector             *encsT;
    int jntsT;

    double timeNow;
    std::vector <yarp::sig::Vector> ctrlCommands;

    std::vector<wayPoint> wayPoints;
    int currentWaypoint;
    int    numWaypoints;
    int            step;    // Flag to know in which step the thread is

    BufferedPort<Bottle> *outPort;
    RpcClient             GSrpcPort;

    bool processWayPoint();

    /**
     * Deals the rpc interaction with the gazeStabilization. It manages start/stop/home states.
     * @param _g string to be passed through the rpc ports
     */
    void gateStabilization(const string _g);

    /**
     * Sends the vels of the waypoint under current evaluation to the port AS-THEY-ARE.
     * No check will be done. The format is a simple bottle of double (v0, v1, v2), in [deg]
     */
    void sendCommand();

    /**
     * Changes the control modes of the torso to either position or velocity
     * @param  _s mode to set. It can be either "position" or "velocity"
     * @return    true/false if success/failure
     */
    bool setTorsoCtrlModes(const string _s);

    /**
     * goes into home configuration (i.e. 0 0 0)
     * @return    true/false if success/failure
     */
    bool goHome();

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
    torsoControllerThread(int _rate, string _name, string _robot, int _v, int _nW, const ResourceFinder &_rf);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    // RESTART THE CYCLE
    bool redoCycle();
};
