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

class gazeStabilizerThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    int verbosity;      // Flag that manages verbosity
    string name;        // Name of the module (to change port names accordingly)  
    string robot;       // Name of the robot (to address both icub and icubSim)
    string if_mode;     // Interface mode to use (either vel1 or vel2)
    string src_mode;    // Source to use (either torso velocities through a port or the inertial sensor)
    string ctrl_mode;   // Control to use (either only "eyes" or "headEyes")
    bool   calib_IMU;   // Flag to know if the calibration on the imu has to be done or not.

    // Classical interfaces - HEAD
    PolyDriver         *ddH;    // head device driver
    IEncoders          *iencsH;
    IPositionControl   *iposH;
    IVelocityControl   *ivelH1;
    IVelocityControl2  *ivelH2;
    IControlLimits     *ilimH;
    IControlMode2      *imodH;
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

    // LowPass filter for IMU signal
    iCub::ctrl::Filter *filt;
    Vector num;
    Vector den;
    Vector y0;

    // LowPass filter for IMU signal
    iCub::ctrl::Integrator *integrator;

    // Internal matrices and variables
    Vector xFP_R;
    Matrix J_E;
    bool isRunning;         // Flag to manage the status of the thread
    bool isIMUCalibrated;   // Flag to manage if the IMU has been calibrated or not
    std::vector<Vector> IMUCalib;
    Vector IMUCalibratedAvg;
    
    Vector dq_T;            // Velocity commands at the torso joints (if we are in torso mode)
    Vector dx_FP;           // 6D speed of the fixation point (src_mode dependent)
    Vector dx_FP_filt;      // 6D speed filtered to reduce the resonant behavior at the head with the inertial
    Vector dx_FP_ego;       // 6D speed of the fixation point due to our own head movements

    // Input from the torsoController
    BufferedPort<Bottle>  inTorsoPort;   // port for reading from the torsoController
    Bottle               *inTorsoBottle; // bottle used for the port

    BufferedPort<Bottle>  inIMUPort;     // port for reading from the inertial sensor
    Bottle               *inIMUBottle;   // bottle used for the port

    BufferedPort<Bottle>  inWBPort;      // port for reading neck velocities from the whole body
    Bottle               *inWBBottle;    // bottle used for the port

    /**
    * Updates a kinematic chain belonging to an eye.
    **/
    void updateEyeChain(iKinChain &_eye, const string &_eyeType);

    /**
    * Updates a kinematic chain belonging to the neck.
    **/
    void updateNeckChain(iKinChain &_neck);

    /**
    * Updates the iCubInertialSensor kinematic chain. Not that useful, though
    * it has been implemented only to keep the syntax used for updateEyeChain.
    **/
    void updateIMUChain(iKinChain &_imu);

    /**
    * Three sources of information lead to three different ways of computing
    * the velocity of the fixation point (listed below)
    **/
    bool compute_dxFP_torsoMode(Vector &_dx_FP);
    bool compute_dxFP_inertialMode(Vector &_dx_FP,Vector &_dx_FP_filt);
    bool compute_dxFP_wholeBodyMode(Vector &_dx_FP);

    /**
    * Computes the velocity of the fixation point given a vector of joint velocities dq
    **/
    Vector compute_dxFP_kinematics(Vector &_dq);

    /**
     * computes the velocity of the fixation point given the gyro measurements
     * @param  _gyro [description]
     * @return       [description]
     */
    Vector compute_dxFP_inertial(Vector &_gyro);

    /**
    * Two different gaze stabilization techniques: either eyes, or eyes + head
    **/
    Vector stabilizeHead(const Vector &_dx_FP);
    Vector stabilizeEyes(const Vector &_dx_FP);
    Vector stabilizeHeadEyes(const Vector &_dx_FP, const Vector &_dx_FP_filt);

    bool computeEgoMotion(const Vector &_dq_N);

    /**
    *
    **/
    bool moveHead(const Vector &_dq_H);
    bool moveEyes(const Vector &_dq_E);
    bool moveHeadEyes(const Vector &_dq_HE);

    /**
     * 
    **/
    bool calibrateIMUMeasurements();

    /**
     * Converts a vector expressed into root reference frame to eyes' RF
    **/
    Vector root2Eyes(const Vector &_v);

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
    gazeStabilizerThread(int _rate, string &_name, string &_robot, int _v, string &_if_mode,
                         string &_src_mode, string &_ctrl_mode, bool _calib_IMU);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    // START AND STOP THE STABILIZATION
    bool startStabilization();
    bool  stopStabilization();
    // SET IF_MODE, SRC_MODE AND CTLR_MODE ON THE FLY
    bool set_if_mode  (const string &_ifm);
    bool set_src_mode (const string &_srcm);
    bool set_ctrl_mode(const string &_ctrlm);
    bool set_calib_IMU(      bool    _cIMU);
    // CALIBRATE IMU MEASUREMENTS IN ORDER TO REMOVE THE BASELINE
    bool calibrateIMU();
    // GET IF_MODE, SRC_MODE AND CTRL_MODE
    string get_if_mode()   { return if_mode;   };
    string get_src_mode()  { return src_mode;  };
    string get_ctrl_mode() { return ctrl_mode; };
    bool   get_calib_IMU() { return calib_IMU; };
    // GO HOME
    bool goHome();
};
