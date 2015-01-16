#include "gazeStabilizerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define FF_STATE_IDLE 0
#define FF_STATE_INIT 1
#define FF_STATE_RX   2
#define FF_NOTX_THRES 4

gazeStabilizerThread::gazeStabilizerThread(int _rate, string &_name, string &_robot, int _v, string &_if_mode,
                                           string &_src_mode, string &_ctrl_mode, bool _calib_IMU, double _int_gain) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v), if_mode(_if_mode),
                                           src_mode(_src_mode), ctrl_mode(_ctrl_mode), calib_IMU(_calib_IMU), integrator_gain(_int_gain)
{
    eyeL = new iCubEye("left_v2");
    eyeR = new iCubEye("right_v2");
    neck = new iCubHeadCenter("right_v2");
    IMU  = new iCubInertialSensor("v2");

    eyeL -> setAllConstraints(false);
    eyeR -> setAllConstraints(false);
    neck -> setAllConstraints(false);
    IMU  -> setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

    // Release torso links
    for (int i = 0; i < 3; i++)
    {
        neck -> releaseLink(i);
        IMU  -> releaseLink(i);
    }

    // Get the chain objects
    chainNeck = neck -> asChain();
    chainEyeL = eyeL -> asChain();
    chainEyeR = eyeR -> asChain();
    chainIMU  = IMU  -> asChain();

    xFP_R.resize(3,0.0);
    J_E.resize(3,3);
    J_E.zero();
    dq_T.resize(3,0.0);
    dx_FP.resize(6,0.0);

    // Create the integrator
    integrator = new Integrator(_rate/1000.0,Vector(3,0.0));

    isRunning = false;
    isIMUCalibrated = false;
    IMUCalibratedAvg.resize(3,0.0);

    FFstate = FF_STATE_IDLE;
    FF_init_cnt = 0;
    FF_Ts = 0;
    dq_NE_FF.resize(6,0.0);

    dq_NE_REF.resize(6,0.0);
}

bool gazeStabilizerThread::threadInit()
{
    inTorsoPort.open(("/"+name+"/torsoController:i").c_str());
    inIMUPort  .open(("/"+name+"/inertial:i").c_str());
    inFFPort   .open(("/"+name+"/wholeBody:i").c_str());
    inREFPort  .open(("/"+name+"/dqRef:i").c_str());

    // Network::connect("/torsoController/torsoVels:o",("/"+name+"/torsoController:i").c_str());
    Network::connect("/torsoController/neckVel:o",("/"+name+"/wholeBody:i").c_str());
    Network::connect("/torsoController/rpc:o",("/"+name+"/rpc:i").c_str());
    Network::connect("/iKinGazeCtrlMOD/dq:o",("/"+name+"/dqRef:i").c_str());

    if (!Network::connect("/imuFilter/inertial:o",("/"+name+"/inertial:i").c_str()))
    {
        Network::connect(("/"+robot+"/inertial").c_str(),("/"+name+"/inertial:i").c_str());
    }
    else
    {
        isIMUCalibrated = true;
    }

    bool ok = 1;
    Property OptH;
    OptH.put("robot",  robot.c_str());
    OptH.put("part",   "head");
    OptH.put("device", "remote_controlboard");
    OptH.put("remote",("/"+robot+"/head").c_str());
    OptH.put("local", ("/"+name +"/head").c_str());

    ddH = new PolyDriver();
    if (!ddH->open(OptH))
    {
        yError("could not open head PolyDriver!\n");
        return false;
    }

    // open the view
    if (ddH->isValid())
    {
        ok = ok && ddH->view(iencsH);
        ok = ok && ddH->view(iposH);
        ok = ok && ddH->view(ivelH1);
        ok = ok && ddH->view(ivelH2);
        ok = ok && ddH->view(ilimH);
        ok = ok && ddH->view(imodH);
    }

    if (!ok)
    {
        yError(" Problems acquiring head interfaces!!!!");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    Vector headAcc(jntsH,1e9);
    ivelH1 -> setRefAccelerations(headAcc.data());
    ivelH2 -> setRefAccelerations(headAcc.data());

    Vector headVel(jntsH,10.0);
    iposH->setRefSpeeds(headVel.data());

    Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());

    ddT = new PolyDriver();
    if (!ddT->open(OptT))
    {
        yError("could not open torso PolyDriver!\n");
        return false;
    }

    if (ddT->isValid())
    {
        ok = ok && ddT->view(iencsT);
        ok = ok && ddT->view(ilimT);
    }

    if (!ok)
    {
        yError(" Problems acquiring torso interfaces!!!!");
        return false;
    }

    iencsT -> getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

    // joints bounds alignment
    deque<IControlLimits*> lim;
    lim.push_back(ilimT);
    lim.push_back(ilimH);

    neck -> alignJointsBounds(lim);
    eyeL -> alignJointsBounds(lim);
    eyeR -> alignJointsBounds(lim);
    IMU  -> alignJointsBounds(lim);

    return true;
}

void gazeStabilizerThread::run()
{
    if (calib_IMU && src_mode == "inertial")
    {
        if (!isIMUCalibrated)
        {
            isIMUCalibrated = calibrateIMUMeasurements();
        }
    }
    else
    {
        isIMUCalibrated=true;
    }

    /*
    * Conceptual recap:
    * 1 - Read encoders for torso and head
    * 2 - Update the iCubHeadCenter, eyeR and eyeL with those values
    * 3 - Compute Fixation Point Data (x_FP and J_E)
    */
    if (isRunning && isIMUCalibrated)
    {
        printf("\n\n");
        // 1 - Read the encoders for torso and head
        iencsT->getEncoders(encsT->data());
        iencsH->getEncoders(encsH->data());

        // 2 - Update the iCubHeadCenter, eyeR and eyeL with those values
        updateEyeChain (*chainEyeL,"left");
        updateEyeChain (*chainEyeR,"right");
        updateNeckChain(*chainNeck);
        updateIMUChain (*chainIMU);
        printMessage(4,"EyeL: %s\n",(CTRL_RAD2DEG*(eyeL->getAng())).toString(3,3).c_str());
        printMessage(4,"EyeR: %s\n",(CTRL_RAD2DEG*(eyeR->getAng())).toString(3,3).c_str());
        printMessage(4,"Neck: %s\n",(CTRL_RAD2DEG*(neck->getAng())).toString(3,3).c_str());
        printMessage(4,"IMU:  %s\n",(CTRL_RAD2DEG*(IMU ->getAng())).toString(3,3).c_str());

        handleFFPort();

        // 3 - Compute Fixation Point Data (x_FP and J_E) for later use
        //          x_FP = position of the fixation point
        //          J_E  = Jacobian that relates the eyes' joints to the motion of the FP
        if (CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,xFP_R,J_E))
        {
            yDebug(" xFP_R:\t%s\n", xFP_R.toString(3,3).c_str());
            printMessage(4,"J_E:\n%s\n",   J_E.toString(3,3).c_str());

            // 3A - Compute the velocity of the fixation point. It is src_mode dependent
            if (src_mode == "torso")
            {
                compute_dxFP_torsoMode(dx_FP);
            }
            else if (src_mode == "inertial" || src_mode == "wholeBody")
            {
                // When in whole body mode, we would like to use the inertial measurement as well
                compute_dxFP_inertialMode(dx_FP);
            }
            yDebug(" dx_FP:  %s", dx_FP.toString(3,3).c_str());

            // 3B - Compute the stabilization command and send it to the robot.
            //      It is ctrl_mode dependent
            if (ctrl_mode == "head")
            {
                Vector dq_N = -1.0*computeNeckVels(dx_FP);
                dq_N        = filterNeckVels(dq_N);
                yInfo("  dq_N:\t%s", dq_N.toString(3,3).c_str());
                moveHead(dq_N);
            }
            else if (ctrl_mode == "eyes")
            {
                Vector dq_E = -1.0*computeEyesVels(dx_FP);
                yInfo("  dq_E:\t%s", dq_E.toString(3,3).c_str());
                moveEyes(dq_E);
            }
            else if (ctrl_mode == "headEyes")
            {
                readVelsFromGazeCtrl();
                // dq_NE_REF.resize(6,0.0);
                Vector dq_N      = +1.0*dq_NE_REF.subVector(0,2)-1.0*computeNeckVels(dx_FP);
                Vector dq_E      = +1.0*dq_NE_REF.subVector(3,5)-1.0*computeEyesVels(dx_FP);
                // printf("%s %s\n",dq_N.toString().c_str(),dq_E.toString().c_str() );
                dq_N = filterNeckVels(dq_N);
                // printf("%s %s\n",dq_N.toString().c_str(),dq_E.toString().c_str() );
                
                Vector dq_NE(6,0.0);
                dq_NE.setSubvector(0,dq_N);
                dq_NE.setSubvector(3,dq_E);
                // dq_NE.resize(6,0.0);
                yInfo("  dq_NE:\t%s", dq_NE.toString(3,3).c_str());

                if (src_mode=="wholeBody")
                {
                    // Add the feedforward input (if there is one)
                    dq_NE = dq_NE -1.0*dq_NE_FF;

                    yInfo("  dq_NE_FF: %s", dq_NE_FF.toString(3,3).c_str());
                    yInfo("  dq_NE_TOT:%s", dq_NE.toString(3,3).c_str());
                }
                
                moveHeadEyes(dq_NE);
            }
        }
        else
        {
            yWarning("  computeFixationPointData() returned false!\n");
        }
    }
}

bool gazeStabilizerThread::readVelsFromGazeCtrl()
{
    if (inREFVector = inREFPort.read(false))
    {
        dq_NE_REF = *inREFVector;
        yInfo("  dq_NE_REF:%s", dq_NE_REF.toString(3,3).c_str());
    }
    else
        return false;

    return true;
}

bool gazeStabilizerThread::handleFFPort()
{
    yTrace("FFstate: %i",FFstate);
    if (FFstate == FF_STATE_IDLE)
    {
        // Reset the feedforward signal
        dq_NE_FF.resize(6,0.0);
        FF_Ts = 0.0;

        // if there is something on the port, change state to init
        if (inFFProperty = inFFPort.read(false))
        {
            FFstate = FF_STATE_RX;
            timeNow = yarp::os::Time::now();
        }
    }
    else if (FFstate == FF_STATE_RX)
    {
        if (inFFProperty = inFFPort.read(false))
        {
            if (inFFProperty->check("Ts"))
            {
                yDebug(" [FFPort] Reading %s",inFFProperty->toString().c_str());
                FF_init_cnt = 0;
                Vector dx_FP_FF(6,0.0);

                FF_Ts = inFFProperty->find("Ts").asDouble();
                compute_dxFP_wholeBodyMode(dx_FP_FF);
                yDebug(" dx_FP_FF: %s FF_Ts: %g",dx_FP_FF.toString(3,3).c_str(),FF_Ts);
                dq_NE_FF.setSubvector(0,computeNeckVels(dx_FP_FF));
                dq_NE_FF.setSubvector(3,computeEyesVels(dx_FP_FF));
            }
            else
            {
                FF_init_cnt++;
            }
        }
        else if (FF_init_cnt > int(FF_NOTX_THRES * FF_Ts / ((*this).getRate()/1000.0)))
        {
            yInfo("  [FFPort][INIT] FF_NOTX_THRES triggered. Going back to idle state.");
            FFstate = FF_STATE_IDLE;
        }
        else
        {
            FF_init_cnt++;
        }
        yTrace(" [FFPort][RX] FF_init_cnt: %i\tthreshold: %g",FF_init_cnt,FF_NOTX_THRES * FF_Ts / (*this).getRate());
    }

    return true;
}

Vector gazeStabilizerThread::computeNeckVels(const Vector &_dx_FP)
{
    // 0  - Take only the rotational part of the velocity of the
    //      fixation point, because we will only compensate that with the head
    Vector dx_FP = _dx_FP.subVector(3,5);
    
    // 1  - Convert x_FP from root to RF_E
    Vector xFP_E = root2Eyes(xFP_R);
    yTrace("xFP_E:\t%s", xFP_E.toString(3,3).c_str());

    // 2  - Compute J_H, that is the jacobian of the head joints alone
    // 2A - attach the fixation point to the neck chain
    Matrix HN = eye(4);
    HN(0,3)   = xFP_E(0);
    HN(1,3)   = xFP_E(1);
    HN(2,3)   = xFP_E(2);
    chainNeck -> setHN(HN);

    // 2B - compute J_N
    Matrix J_N  = chainNeck -> GeoJacobian();

    // 2C - take only the last three rows belonging to the head joints
    Matrix J_Np = J_N.submatrix(3,5,3,5);
    printMessage(3,"J_Np:\n%s\n",J_Np.toString(3,3).c_str());

    // 2D - remove the fixation point from the neck chain
    chainNeck -> setHN(eye(4,4));

    // 3 - Return J_N# * dx_FP
    Matrix J_N_pinv = pinv(J_Np);
    printMessage(3,"J_N_pinv:\n%s\n",J_N_pinv.toString(3,3).c_str());
    return CTRL_RAD2DEG * (J_N_pinv * dx_FP);
}

Vector gazeStabilizerThread::computeEyesVels(const Vector &_dx_FP)
{
    // 0  - Take only the translational part of the velocity of the
    //      fixation point, because we will only compensate that with the eyes
    Vector dx_FP = _dx_FP.subVector(0,2);

    // 1 - Compute dq_E  = J_E# * dx_FP;
    Matrix J_E_pinv = pinv(J_E);
    Vector dq_E = CTRL_RAD2DEG * (J_E_pinv * dx_FP);

    return dq_E;
}

Vector gazeStabilizerThread::filterNeckVels(const Vector &_dq_N)
{
    return integrator_gain*integrator->integrate(_dq_N);
}

bool gazeStabilizerThread::compute_dxFP_wholeBodyMode(Vector &_dx_FP)
{
    /*
    * Conceptual recap:
    * 4  - Read data from the wholeBody port 
           (if there is no data, nothing is commanded)
    * 5  - Compute the lever arm between the fixation point and the Neck Base
    * 6A - Do the same magic as if we were in inertial mode..
    * 6B - ..and also here
    * 7  - Send dq_H
    * 8  - Compute the dq_E as if we were in torso mode..
    */
    // 4  - Read data from the wholeBody port 
    Vector v(3,0.0);
    Vector w(3,0.0);

    if (inFFProperty->check("neck_velocity"))
    {
        Bottle *b = inFFProperty->find("neck_velocity").asList();

        // Translational part
        v[0] = b->get(0).asDouble();
        v[1] = b->get(1).asDouble();
        v[2] = b->get(2).asDouble();

        // Rotational part
        w[0] = b->get(3).asDouble();
        w[1] = b->get(4).asDouble();
        w[2] = b->get(5).asDouble();

        yDebug(" vNeck %s  %s",v.toString(3,3).c_str(),w.toString(3,3).c_str());
    }

    // 5  - Compute the lever arm between the neck base and the fixation point
    // Vector xFP_E = root2Eyes(xFP_R);
    // Matrix HN = eye(4,4);
    // HN(0,3)   = xFP_E(0);
    // HN(1,3)   = xFP_E(1);
    // HN(2,3)   = xFP_E(2);

    // chainNeck -> setHN(HN);
    // Matrix Htot  = neck -> getH();
    // Matrix Hneck = neck -> getH(2);     // get the H from ROOT to Neck Base
    // Matrix H = SE3inv(Hneck) * Htot;
    // chainNeck -> setHN(eye(4,4));

    Matrix H = eye(4,4);
    H(0,3)   = xFP_R[0];
    H(1,3)   = xFP_R[1];
    H(2,3)   = xFP_R[2];

    // 6 - Do the magic dx_FP = v+w^r
    Vector dx_FP_pos(3,0.0);
    dx_FP_pos = v+(w[0]*cross(H,0,H,3)+w[1]*cross(H,1,H,3)+w[2]*cross(H,2,H,3));

    _dx_FP.setSubvector(0, dx_FP_pos);
    _dx_FP.setSubvector(3, w);

    return true;
}

bool gazeStabilizerThread::compute_dxFP_inertialMode(Vector &_dx_FP)
{
    /*
    * Conceptual recap:
    * 4  - Read data from the inertial sensor
    *      (if there is no data from the IMU, keep the old value)
    * 5  - Compute the lever arm between the fixation point and the IMU
    * 6A - PUT THE MAGIC HERE ...
    * 6B - ... AND HERE!
    */

    if (inIMUBottle = inIMUPort.read(false))
    {
        // 4  - Read data from the inertial sensor
        Vector gyr(3,0.0);
        gyr[0] = inIMUBottle -> get(6).asDouble()-IMUCalibratedAvg[0];
        gyr[1] = inIMUBottle -> get(7).asDouble()-IMUCalibratedAvg[1];
        gyr[2] = inIMUBottle -> get(8).asDouble()-IMUCalibratedAvg[2];
        yDebug(" Gyro: \t%s",gyr.toString(3,3).c_str());

        // 5  - Compute the lever arm between the fixation point and the IMU
        Matrix H = IMU -> getH();
        H(0,3)   = xFP_R[0]-H(0,3);
        H(1,3)   = xFP_R[1]-H(1,3);
        H(2,3)   = xFP_R[2]-H(2,3);

        // 6A - Compute the positional component of the speed of the fixation point
        //      thanks to the rotational component measure obtained from the IMU
        Vector dx_FP_pos(3,0.0);
        dx_FP_pos=CTRL_DEG2RAD*(gyr[0]*cross(H,0,H,3)+gyr[1]*cross(H,1,H,3)+gyr[2]*cross(H,2,H,3));
        _dx_FP.setSubvector(0, dx_FP_pos);

        // 6B - Project IMU measure on the the rotational component
        //      of the speed of the fixation point
        H(0,3) = 0;        H(1,3) = 0;        H(2,3) = 0;

        gyr.push_back(1.0);
        Vector dx_FP_rot = CTRL_DEG2RAD * H * gyr;
        dx_FP_rot.pop_back();

        _dx_FP.setSubvector(3, dx_FP_rot);
    }
    else
    {
        // 4 - (if there is no data from the IMU, keep the old value)
        yWarning("No signal from the IMU!\n");
        return false;
    }

    return true;
}

bool gazeStabilizerThread::compute_dxFP_torsoMode(Vector &_dx_FP)
{
    /*
    * Conceptual recap:
    * 4  - Read dq_H and dq_T (for now only dq_T;
    *      if there is no dq_T, nothing is commanded)
    * 4B - Compute dx_FP with compute_dxFP_kinematics
    */
    dq_T.resize(3,0.0);

    if (inTorsoBottle = inTorsoPort.read(false))
    {
        // 4 - Read dq_H and dq_T (for now only dq_T;
        //     if there is no dq_T, nothing is commanded)
        Vector dq(6,0.0);
        dq_T[2] = inTorsoBottle->get(0).asDouble();
        dq_T[1] = inTorsoBottle->get(1).asDouble();
        dq_T[0] = inTorsoBottle->get(2).asDouble();
        dq.setSubvector(0,CTRL_DEG2RAD * dq_T);

        // 4B - Compute dx_FP with compute_dxFP_kinematics
        _dx_FP = compute_dxFP_kinematics(dq);
        return true;
    }
    else
    {
        printMessage(1,"No signal from the torso port!\n");
        return false;
    }
}

Vector gazeStabilizerThread::compute_dxFP_kinematics(Vector &_dq)
{
    if (_dq.size() != 6)
    {
        yError("compute_dxFP_kinematics got a wrong dq vector! Requested 6, got %i\n",_dq.size());
        return Vector(3,0.0);
    }

    // 5 - Convert x_FP from root to RF_
    Vector xFP_E = root2Eyes(xFP_R);
    printMessage(2,"xFP_E:\t%s\n", xFP_E.toString(3,3).c_str());

    // 6 - SetHN() with xFP_E
    Matrix HN = eye(4,4);
    HN(0,3)   = xFP_E(0);
    HN(1,3)   = xFP_E(1);
    HN(2,3)   = xFP_E(2);
    chainNeck -> setHN(HN);
    Matrix J_TN = chainNeck -> GeoJacobian();
    chainNeck -> setHN(eye(4,4));

    // 7 - Compute dx_FP = J_TN * [dq_T ; dq_H]
    printMessage(3,"J_TN:\n%s\n",J_TN.toString(3,3).c_str());
    return J_TN * _dq;
}

bool gazeStabilizerThread::moveHead(const Vector &_dq_N)
{
    VectorOf<int> jointsToSet;
    if (!areJointsHealthyAndSet(jointsToSet,"velocity"))
    {
        stopStabilization();
        return false;
    }
    else
    {
        setHeadCtrlModes(jointsToSet,"velocity");
    }

    // Move the head
    printMessage(3,"Moving neck to: %s\n",_dq_N.toString(3,3).c_str());
    std::vector<int> Ejoints;  // indexes of the joints to control
    Ejoints.push_back(0);
    Ejoints.push_back(1);
    Ejoints.push_back(2);
    printMessage(4,"Head joints to be controlled: %i %i %i\n",Ejoints[0],Ejoints[1],Ejoints[2]);

    if (if_mode == "vel2")
    {
        int nJnts = 3;
        ivelH2 -> velocityMove(nJnts,Ejoints.data(),_dq_N.data());
    }
    else if (if_mode == "vel1")
    {
        ivelH1 -> velocityMove(Ejoints[0],_dq_N(0));
        ivelH1 -> velocityMove(Ejoints[1],_dq_N(1));
        ivelH1 -> velocityMove(Ejoints[2],_dq_N(2));
    }
    else
    {
        yWarning("if_mode is neither vel1 or vel2. No velocity will be sent.");
        return false;
    }

    return true;
}

bool gazeStabilizerThread::moveEyes(const Vector &_dq_E)
{
    VectorOf<int> jointsToSet;
    if (!areJointsHealthyAndSet(jointsToSet,"velocity"))
    {
        stopStabilization();
        return false;
    }
    else
    {
        setHeadCtrlModes(jointsToSet,"velocity");
    }

    printMessage(3,"Moving eyes to: %s\n",_dq_E.toString(3,3).c_str());
    std::vector<int> Ejoints;  // indexes of the joints to control
    Ejoints.push_back(3);
    Ejoints.push_back(4);
    Ejoints.push_back(5);
    printMessage(4,"Head joints to be controlled: %i %i %i\n",Ejoints[0],Ejoints[1],Ejoints[2]);

    if (if_mode == "vel2")
    {
        int nJnts = 3;
        ivelH2 -> velocityMove(nJnts,Ejoints.data(),_dq_E.data());
    }
    else if (if_mode == "vel1")
    {
        ivelH1 -> velocityMove(Ejoints[0],_dq_E(0));
        ivelH1 -> velocityMove(Ejoints[1],_dq_E(1));
        ivelH1 -> velocityMove(Ejoints[2],_dq_E(2));
    }
    else
    {
        yWarning("if_mode is neither vel1 or vel2. No velocity will be sent.");
        return false;
    }
    return true;
}

bool gazeStabilizerThread::moveHeadEyes(const Vector &_dq_NE)
{
    bool ret=1;
    ret = ret && moveHead(_dq_NE.subVector(0,2));
    ret = ret && moveEyes(_dq_NE.subVector(3,5));

    return ret;
}

void gazeStabilizerThread::updateEyeChain(iKinChain &_eye, const string &_eyeType)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    _eye.setBlockingValue(0,CTRL_DEG2RAD*torso[2]);
    _eye.setBlockingValue(1,CTRL_DEG2RAD*torso[1]);
    _eye.setBlockingValue(2,CTRL_DEG2RAD*torso[0]);
    _eye.setBlockingValue(3,CTRL_DEG2RAD*head[0]);
    _eye.setBlockingValue(4,CTRL_DEG2RAD*head[1]);
    _eye.setBlockingValue(5,CTRL_DEG2RAD*head[2]);

    _eye.setAng(6,CTRL_DEG2RAD*head[3]);

    // CHANGE THIS: Avoid going low with the vergence
    // (this value is empyrical, but it is what the gaze controller is doing internally)
    if (head[5] < 1.0)
        head[5] = 1.0;

    bool isLeft=(_eyeType == "left_v2" ||
                 _eyeType == "left"    ||
                 _eyeType == "left_v1");

    if (isLeft)
        _eye.setAng(7,CTRL_DEG2RAD*(head[4]+head[5]/2.0));
    else
        _eye.setAng(7,CTRL_DEG2RAD*(head[4]-head[5]/2.0));
}

void gazeStabilizerThread::updateNeckChain(iKinChain &_neck)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    yarp::sig::Vector q(8,0.0);
    q[0] = torso[2];   q[1] = torso[1];   q[2] = torso[0];
    q[3] = head[0];    q[4] = head[1];    q[5] = head[2];

    q = CTRL_DEG2RAD*q;
    _neck.setAng(q);
}

void gazeStabilizerThread::updateIMUChain(iKinChain &_imu)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    yarp::sig::Vector q(6);
    q[0] = torso[2];
    q[1] = torso[1];
    q[2] = torso[0];
    q[3] =  head[0];
    q[4] =  head[1];
    q[5] =  head[2];

    q = CTRL_DEG2RAD*q;
    _imu.setAng(q);
}

bool gazeStabilizerThread::areJointsHealthyAndSet(VectorOf<int> &jointsToSet,const string &_s)
{
    VectorOf<int> modes(encsH->size());
    imodH->getControlModes(modes.getFirst());

    for (size_t i=0; i<modes.size(); i++)
    {
        if ((modes[i]==VOCAB_CM_HW_FAULT) || (modes[i]==VOCAB_CM_IDLE))
            return false;

        if (_s=="velocity")
        {
            if (modes[i]!=VOCAB_CM_MIXED || modes[i]!=VOCAB_CM_VELOCITY)
                jointsToSet.push_back(i);
        }
        else if (_s=="position")
        {
            if (modes[i]!=VOCAB_CM_MIXED || modes[i]!=VOCAB_CM_POSITION)
                jointsToSet.push_back(i);
        }

    }

    return true;
}

bool gazeStabilizerThread::setHeadCtrlModes(const VectorOf<int> &jointsToSet,const string &_s)
{
    if (_s!="position" || _s!="velocity")
        return false;

    if (jointsToSet.size()==0)
        return true;

    VectorOf<int> modes;
    for (size_t i=0; i<jointsToSet.size(); i++)
    {
        if (_s=="position")
        {
            modes.push_back(VOCAB_CM_POSITION);
        }
        else if (_s=="velocity")
        {
            modes.push_back(VOCAB_CM_VELOCITY);
        }
    }

    imodH->setControlModes(jointsToSet.size(),
                           jointsToSet.getFirst(),
                           modes.getFirst());

    return true;
}

bool gazeStabilizerThread::calibrateIMUMeasurements()
{
    // On the simulator we don't have to calibrate the IMU
    if (robot == "icubSim")
    {
        yWarning("  On the simulator we don't have to calibrate the IMU!");
        return true;
    }

    if (inIMUBottle = inIMUPort.read(false))
    {
        Vector w(3,0.0);
        w[0] = inIMUBottle -> get(6).asDouble();
        w[1] = inIMUBottle -> get(7).asDouble();
        w[2] = inIMUBottle -> get(8).asDouble();
        IMUCalib.push_back(w);

        if (IMUCalib.size() == 200)
        {
            Vector v(3,0.0);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < IMUCalib.size(); j++)
                {
                    v[i] += IMUCalib[j][i];
                }
                v[i] /= IMUCalib.size();
            }
            
            IMUCalibratedAvg=v;
            yInfo("  IMU has been calibrated! Calibrated values: %s",IMUCalibratedAvg.toString(3,3).c_str());
            return true;
        }


        return false;
    }
    else
    {
        return false;
    }

    return false;
}

bool gazeStabilizerThread::calibrateIMU()
{
    printMessage(1,"Calibrating IMU...\n");
    IMUCalibratedAvg.resize(3,0.0);
    IMUCalib.clear();
    isIMUCalibrated=false;
    return true;
}

bool gazeStabilizerThread::set_calib_IMU(const bool   &_cIMU)
{
    calib_IMU=_cIMU;
    if (_cIMU==false)
    {
        IMUCalibratedAvg.resize(3,0.0);
    }
    else if (_cIMU==true)
    {
        calibrateIMU();
    }
    return true;
}

bool gazeStabilizerThread::set_if_mode(const string &_ifm)
{
    if (_ifm == "vel1" || _ifm == "vel2")
    {
        if_mode = _ifm;
        return true;
    }
    return false;
}

bool gazeStabilizerThread::set_src_mode(const string &_srcm)
{
    if (_srcm == "torso" || _srcm == "inertial")
    {
        src_mode = _srcm;
        return true;
    }
    else if(_srcm == "wholeBody" || _srcm == "whole_body" || _srcm == "wholebody")
    {
        src_mode = "wholeBody";
        return true;
    }
    return false;
}

bool gazeStabilizerThread::set_ctrl_mode(const string &_ctrlm)
{
    if (_ctrlm == "head" || _ctrlm == "eyes" || _ctrlm == "headEyes")
    {
        ctrl_mode = _ctrlm;
        return true;
    }
    return false;
}

bool gazeStabilizerThread::set_integrator_gain(const double &_intG)
{
    if (_intG < 20.0)
    {
        integrator_gain = _intG;
        return true;
    }

    return false;
}

bool gazeStabilizerThread::startStabilization()
{
    isRunning = true;
    return true;
}

bool gazeStabilizerThread::stopStabilization()
{
    FFstate = FF_STATE_IDLE;
    isRunning = false;
    dx_FP.resize(6,0.0);
    integrator->reset(Vector(3,0.0));
    return true;
}

bool gazeStabilizerThread::goHome()
{
    if (!isRunning)
    {
        VectorOf<int> jointsToSet;
        if (!areJointsHealthyAndSet(jointsToSet,"position"))
        {
            stopStabilization();
            return false;
        }
        else
        {
            setHeadCtrlModes(jointsToSet,"position");
        }

        Vector pos0(6,0.0);
        iposH -> positionMove(pos0.data());
        return true;
    }
    return false;
}

Vector gazeStabilizerThread::root2Eyes(const Vector &_v)
{
    Vector v=_v;
    Matrix H_RE = chainNeck->getH();        // matrix from root to RF_E
    v.push_back(1);
    return SE3inv(H_RE) * v;
}

int gazeStabilizerThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s]",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void gazeStabilizerThread::closePort(Contactable &_port)
{
    _port.interrupt();
    _port.close();
}

void gazeStabilizerThread::threadRelease()
{
    yDebug(" Stopping stabilization..");
        stopStabilization();

    yDebug(" Closing ports...");
        closePort(inTorsoPort);
        closePort(inIMUPort);
        closePort(inFFPort);

    yDebug(" Closing controllers..");
        ddH->close();
        delete ddH;
        ddH = NULL;

        ddT->close();
        delete ddT;
        ddT = NULL;

        if (encsH)
        {
            delete encsH;
            encsH = NULL;
        }

        if (encsT)
        {
            delete encsT;
            encsT = NULL;
        }

        if (eyeR)
        {
            delete eyeR;
            eyeR = NULL;
        }

        if (eyeL)
        {
            delete eyeL;
            eyeL = NULL;
        }

        if (neck)
        {
            delete neck;
            neck = NULL;
        }

        if (IMU)
        {
            delete IMU;
            IMU = NULL;
        }

        delete integrator; integrator = NULL;
}

// empty line to make gcc happy
