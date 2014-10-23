#include "gazeStabilizerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define GYRO_BIAS_STABILITY_IMU_CALIB      1.1     // [deg/s]
#define GYRO_BIAS_STABILITY                4.0     // [deg/s]

gazeStabilizerThread::gazeStabilizerThread(int _rate, string &_name, string &_robot, int _v, string &_if_mode,
                                           string &_src_mode, string &_ctrl_mode, bool _calib_IMU) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v), if_mode(_if_mode),
                                           src_mode(_src_mode), ctrl_mode(_ctrl_mode), calib_IMU(_calib_IMU)
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

    inTorsoPort = new BufferedPort<Bottle>;
    inIMUPort   = new BufferedPort<Bottle>;
    inWBPort    = new BufferedPort<Bottle>;

    xFP_R.resize(3,0.0);
    J_E.resize(3,3);
    J_E.zero();
    dq_T.resize(3,0.0);
    dx_FP.resize(6,0.0);
    dx_FP_filt.resize(6,0.0);
    dx_FP_ego.resize(6,0.0);

    // Create the filter
    y0.resize(3,0.0);

    num.resize(4,0.0);
    num(0) = 8.177728287846837e-06;
    num(1) = 2.453318486354051e-05;
    num(2) = 2.453318486354051e-05;
    num(3) = 8.177728287846837e-06;

    den.resize(4,0.0);
    den(0) =                     1;
    den(1) =    -2.918324259165458;
    den(2) =     2.839949961758724;
    den(3) =    -0.921560280766964;

    filt = new Filter(num,den,y0);

    isRunning = false;
    isIMUCalibrated = false;
    IMUCalibratedAvg.resize(3,0.0);
}

bool gazeStabilizerThread::threadInit()
{
    inTorsoPort -> open(("/"+name+"/torsoController:i").c_str());
    inIMUPort   -> open(("/"+name+"/inertial:i").c_str());
    inWBPort    -> open(("/"+name+"/wholeBody:i").c_str());

    Network::connect("/torsoController/gazeStabilizer:o",("/"+name+"/torsoController:i").c_str());
    Network::connect("/torsoController/rpc:o",("/"+name+"/rpc:i").c_str());
    Network::connect(("/"+robot+"/inertial").c_str(),("/"+name+"/inertial:i").c_str());

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
        printMessage(0,"ERROR: could not open head PolyDriver!\n");
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
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    Vector headAcc(jntsH,1e9);
    ivelH1 -> setRefAccelerations(headAcc.data());

    Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());

    ddT = new PolyDriver();
    if (!ddT->open(OptT))
    {
        printMessage(0,"ERROR: could not open torso PolyDriver!\n");
        return false;
    }

    if (ddT->isValid())
    {
        ok = ok && ddT->view(iencsT);
        ok = ok && ddT->view(ilimT);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring torso interfaces!!!!\n");
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

        // 3 - Compute Fixation Point Data (x_FP and J_E) for later use
        //          x_FP = position of the fixation point
        //          J_E  = Jacobian that relates the eyes' joints to the motion of the FP
        if (CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,xFP_R,J_E))
        {
            printMessage(2,"xFP_R:\t%s\n", xFP_R.toString(3,3).c_str());
            printMessage(4,"J_E:\n%s\n",   J_E.toString(3,3).c_str());

            // 3A - Compute the velocity of the fixation point. It is src_mode dependent
            // dx_FP.resize(6,0.0);
            // dx_FP_filt.resize(6,0.0);

            if (src_mode == "torso")
            {
                compute_dxFP_torsoMode(dx_FP);
                dx_FP_filt = dx_FP;
            }
            else if (src_mode == "inertial")
            {
                compute_dxFP_inertialMode(dx_FP,dx_FP_filt);
            }
            else if (src_mode == "wholeBody")
            {
                compute_dxFP_wholeBodyMode(dx_FP);
                dx_FP_filt = dx_FP;
            }
            printMessage(0,"dx_FP     :\t%s\n", dx_FP.toString(3,3).c_str());
            if (src_mode=="inertial")
            {
                printMessage(1,"dx_FP_filt:\t%s\n", dx_FP_filt.toString(3,3).c_str());
            }

            // 3B - Compute the stabilization command and send it to the robot.
            //      It is ctrl_mode dependent
            if (ctrl_mode == "head")
            {
                Vector dq_H=stabilizeHead(dx_FP);
                printMessage(0,"dq_H:\t%s\n", dq_H.toString(3,3).c_str());
                computeEgoMotion(dq_H);
                moveHead(dq_H);
            }
            else if (ctrl_mode == "eyes")
            {
                Vector dq_E=stabilizeEyes(dx_FP);
                printMessage(0,"dq_E:\t%s\n", dq_E.toString(3,3).c_str());
                moveEyes(dq_E);
            }
            else if (ctrl_mode == "headEyes")
            {
                Vector dq_NE=stabilizeHeadEyes(dx_FP,dx_FP_filt);
                printMessage(0,"dq_NE:\t%s\n", dq_NE.toString(3,3).c_str());
                for (int i = 0; i < 6; i++)
                {
                    if (dq_NE[i]>40.0)
                    {
                        printf("\t\t\t\t\t\t\t\t\t\t\t\tBAMMMMMMMMMMMMMM\n");
                    }
                }
                computeEgoMotion(dq_NE.subVector(0,2));
                moveHeadEyes(dq_NE);
            }
        }
        else
        {
            printMessage(0,"computeFixationPointData() returned false!\n");
        }
    }
}

bool gazeStabilizerThread::computeEgoMotion(const Vector &_dq_N)
{
    if (src_mode == "inertial")
    {
        Vector dq_TN(6,0.0);
        dq_TN.setSubvector(0,dq_T);
        dq_TN.setSubvector(3,_dq_N);
        printMessage(1,"dq_TN:\t%s\n", dq_TN.toString(3,3).c_str());
        Vector d2R_dq_TN = CTRL_DEG2RAD * dq_TN;

        dx_FP_ego    = compute_dxFP_kinematics(d2R_dq_TN);
        dx_FP_ego[0] = 0;   dx_FP_ego[1] = 0;   dx_FP_ego[2] = 0;
        printMessage(0,"dx_FP_ego:\t%s\t\n", dx_FP_ego.toString(3,3).c_str());
        return true;
    }
    else
        return false;
}

Vector gazeStabilizerThread::stabilizeHead(const Vector &_dx_FP)
{
    // 0  - Take only the rotational part of the velocity of the
    //      fixation point, because we will only compensate that with the head
    Vector dx_FP = _dx_FP.subVector(3,5);
    
    // 1  - Convert x_FP from root to RF_E
    Vector xFP_E = root2Eyes(xFP_R);
    printMessage(2,"xFP_E:\t%s\n", xFP_E.toString(3,3).c_str());

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
    return -CTRL_RAD2DEG * (J_N_pinv * dx_FP);
}

Vector gazeStabilizerThread::stabilizeEyes(const Vector &_dx_FP)
{
    // 0  - Take only the translational part of the velocity of the
    //      fixation point, because we will only compensate that with the eyes
    Vector dx_FP = _dx_FP.subVector(0,2);

    // 1 - Compute dq_E  = J_E# * dx_FP;
    Matrix J_E_pinv = pinv(J_E);
    Vector dq_E = -CTRL_RAD2DEG * (J_E_pinv * dx_FP);

    return dq_E;
}

Vector gazeStabilizerThread::stabilizeHeadEyes(const Vector &_dx_FP, const Vector &_dx_FP_filt)
{
    Vector dq_NE(6,0.0);
    Vector dq_N(3,0.0);
    Vector dq_E(3,0.0);
    dq_N = stabilizeHead(_dx_FP_filt);
    dq_NE.setSubvector(0,dq_N);

    if(src_mode == "torso")// || src_mode == "inertial")
    {
        Vector dq_TN(6,0.0);
        dq_TN.setSubvector(0,dq_T);
        dq_TN.setSubvector(3,dq_N);
        Vector d2R_dq_TN = CTRL_DEG2RAD * dq_TN;
        printMessage(1,"dq_TN:\t%s\n", dq_TN.toString(3,3).c_str());

        Vector dx_FP_2   = compute_dxFP_kinematics(d2R_dq_TN);
        printMessage(0,"dx_FP_ego:\t%s\t\n", dx_FP_2.toString(3,3).c_str());
        dq_E=stabilizeEyes(dx_FP_2);
    }
    else
    {
        dq_E=stabilizeEyes(_dx_FP);
    }

    dq_NE.setSubvector(3,dq_E);

    return dq_NE;
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

    if (inWBBottle = inWBPort->read(false))
    {
        // 4  - Read data from the wholeBody port 
        //      (if there is no data, nothing is commanded)
        // Translational part
        Vector v(3,0.0);
        double vX = inWBBottle -> get(0).asDouble(); v[0] = vX;
        double vY = inWBBottle -> get(1).asDouble(); v[1] = vY;
        double vZ = inWBBottle -> get(2).asDouble(); v[2] = vZ;

        // Rotational part
        Vector w(3,0.0);
        double wX = inWBBottle -> get(3).asDouble(); w[0] = wX;
        double wY = inWBBottle -> get(4).asDouble(); w[1] = wX;
        double wZ = inWBBottle -> get(5).asDouble(); w[2] = wX;

        // 5  - Compute the lever arm between the fixation point and the Neck Base
        Matrix H = neck -> getH(2);     // get the H from ROOT to Neck Base
        H(0,3) = xFP_R[0]-H(0,3);
        H(1,3) = xFP_R[1]-H(1,3);
        H(2,3) = xFP_R[2]-H(2,3);

        // 6 - Do the magic v_FP = v+w^r
        Vector dx_FP = v+wX*cross(H,0,H,3)+wY*cross(H,1,H,3)+wZ*cross(H,2,H,3);

        _dx_FP.setSubvector(0, dx_FP);

        w.push_back(1.0);
        w = CTRL_DEG2RAD * H*w;
        w.pop_back();
        _dx_FP.setSubvector(3, w);
        return true;
    }
    else
    {
        printMessage(0,"No signal from the WB port!\n");
        return false;
    }
}

bool gazeStabilizerThread::compute_dxFP_inertialMode(Vector &_dx_FP, Vector &_dx_FP_filt)
{
    /*
    * Conceptual recap:
    * 4  - Read data from the inertial sensor
    *      (if there is no data, nothing is commanded)
    * 5  - Compute the lever arm between the fixation point and the IMU
    * 6A - PUT THE MAGIC HERE ...
    * 6B - ... AND HERE!
    */

    if (inIMUBottle = inIMUPort->read(false))
    {
        // 4  - Read data from the inertial sensor
        //     (if there is no data, nothing is commanded)
        Vector w(3,0.0);
        Vector w_filt(3,0.0);
        double gyrX = inIMUBottle -> get(6).asDouble(); w[0] = gyrX-IMUCalibratedAvg[0];
        double gyrY = inIMUBottle -> get(7).asDouble(); w[1] = gyrY-IMUCalibratedAvg[1];
        double gyrZ = inIMUBottle -> get(8).asDouble(); w[2] = gyrZ-IMUCalibratedAvg[2];

        w_filt = filt->filt(w);
        // w_filt = w;
        
        _dx_FP      = compute_dxFP_inertial(w);
        _dx_FP_filt = compute_dxFP_inertial(w_filt);
        // _dx_FP_filt = _dx_FP;
        
        dx_FP_ego.resize(6,0.0);
    }
    else
    {
        printMessage(0,"No signal from the IMU!\n");
        printMessage(0,"\n");
        
        _dx_FP      = _dx_FP;
        _dx_FP_filt = _dx_FP_filt;
        // _dx_FP_filt = _dx_FP;

        dx_FP_ego.resize(6,0.0);
    }
    return true;
}

Vector gazeStabilizerThread::compute_dxFP_inertial(Vector &_gyro)
{
    printMessage(0,"Gyro: \t%s\n",_gyro.toString(3,3).c_str());

    double gyrX = _gyro(0);
    double gyrY = _gyro(1);
    double gyrZ = _gyro(2);
    Vector _dx_FP(6,0.0);

    // 5  - Compute the lever arm between the fixation point and the IMU
    Matrix H = IMU -> getH();
    H(0,3)   = xFP_R[0]-H(0,3);
    H(1,3)   = xFP_R[1]-H(1,3);
    H(2,3)   = xFP_R[2]-H(2,3);

    // 6A - Filter out the noise on the gyro readouts
    Vector dx_FP(3,0.0);
    double gyrobiasstability=calib_IMU?GYRO_BIAS_STABILITY_IMU_CALIB:GYRO_BIAS_STABILITY;
    // if (robot=="iCubSim")
    // {
    //     gyrobiasstability/=3;
    // }
    
    if ((fabs(gyrX)<gyrobiasstability) && (fabs(gyrY)<gyrobiasstability) &&
        (fabs(gyrZ)<gyrobiasstability))
    {
        // dx_FP_ego.resize(6,0.0);
        // dx_FP.resize(J_E.rows(),0.0);
    }

    // 6B - Do the magic 
    else
    {
        dx_FP=CTRL_DEG2RAD*(gyrX*cross(H,0,H,3)+gyrY*cross(H,1,H,3)+gyrZ*cross(H,2,H,3));

        _dx_FP.setSubvector(0, dx_FP);

        H(0,3) = 0;
        H(1,3) = 0;
        H(2,3) = 0;

        // printMessage(0,"w: \t%s\tH:\n%s\n",w.toString(3,3).c_str(),H.toString(3,3).c_str());
        _gyro.push_back(1.0);
        _gyro = CTRL_DEG2RAD * H * _gyro;
        _gyro.pop_back();

        // 7 - Compute dx_FP
        _dx_FP.setSubvector(3, _gyro);
    }
    return _dx_FP;
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

    if (inTorsoBottle = inTorsoPort->read(false))
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
        printMessage(0,"ERROR: compute_dxFP_kinematics got a wrong dq vector! Requested 6, got %i\n",_dq.size());
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

bool gazeStabilizerThread::moveHead(const Vector &_dq_H)
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
    printMessage(3,"Moving neck to: %s\n",_dq_H.toString(3,3).c_str());
    std::vector<int> Ejoints;  // indexes of the joints to control
    Ejoints.push_back(0);
    Ejoints.push_back(1);
    Ejoints.push_back(2);
    printMessage(4,"Head joints to be controlled: %i %i %i\n",Ejoints[0],Ejoints[1],Ejoints[2]);

    if (if_mode == "vel2")
    {
        int nJnts = 3;
        ivelH2 -> velocityMove(nJnts,Ejoints.data(),_dq_H.data());
    }
    else if (if_mode == "vel1")
    {
        ivelH1 -> velocityMove(Ejoints[0],_dq_H(0));
        ivelH1 -> velocityMove(Ejoints[1],_dq_H(1));
        ivelH1 -> velocityMove(Ejoints[2],_dq_H(2));
    }
    else
    {
        printMessage(0,"if_mode is neither vel1 or vel2. No velocity will be sent.\n");
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
        printMessage(0,"if_mode is neither vel1 or vel2. No velocity will be sent.\n");
        return false;
    }
    return true;
}

bool gazeStabilizerThread::moveHeadEyes(const Vector &_dq_HE)
{
    bool ret=1;
    ret = ret && moveHead(_dq_HE.subVector(0,2));
    ret = ret && moveEyes(_dq_HE.subVector(3,5));

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
        printMessage(0,"On the simulator we don't have to calibrate the IMU!\n");
        return true;
    }

    if (inIMUBottle = inIMUPort->read(false))
    {
        Vector w(3,0.0);
        double gyrX = inIMUBottle -> get(6).asDouble(); w[0] = gyrX;
        double gyrY = inIMUBottle -> get(7).asDouble(); w[1] = gyrY;
        double gyrZ = inIMUBottle -> get(8).asDouble(); w[2] = gyrZ;
        IMUCalib.push_back(w);

        if (IMUCalib.size() == 300)
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
            printMessage(0,"IMU has been calibrated! Calibrated values: %s\n",IMUCalibratedAvg.toString(3,3).c_str());
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

bool gazeStabilizerThread::set_calib_IMU(bool _cIMU)
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

bool gazeStabilizerThread::startStabilization()
{
    isRunning = true;
    return true;
}

bool gazeStabilizerThread::stopStabilization()
{
    isRunning = false;
    dx_FP.resize(6,0.0);
    dx_FP_filt.resize(6,0.0);
    dx_FP_ego.resize(6,0.0);
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
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void gazeStabilizerThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void gazeStabilizerThread::threadRelease()
{
    printMessage(0,"Moving head to home position.. \n");
        stopStabilization();
        // goHome();

    printMessage(0,"Closing ports...\n");
        closePort(inTorsoPort);
        closePort(inIMUPort);
        closePort(inWBPort);

    printMessage(0,"Closing controllers..\n");
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

        if (filt)
        {
            delete filt;
            filt = NULL;
        }
}

// empty line to make gcc happy
