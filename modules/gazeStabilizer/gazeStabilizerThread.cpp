#include "gazeStabilizerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define GYRO_BIAS_STABILITY                 5.0     // [deg/s]

gazeStabilizerThread::gazeStabilizerThread(int _rate, string _name, string _robot, int _v,
                                           string _if_mode, string _src_mode) :
                                           RateThread(_rate), name(_name), robot(_robot),
                                           verbosity(_v), if_mode(_if_mode), src_mode(_src_mode)
{
    eyeL = new iCubEye("left_v2");
    eyeR = new iCubEye("right_v2");
    neck = new iCubHeadCenter("right_v2");
    IMU  = new iCubInertialSensor("v2");

    isRunning = false;

    // Release torso links
    for (int i = 0; i < 3; i++)
    {
        eyeL -> releaseLink(i);
        eyeR -> releaseLink(i);
        neck -> releaseLink(i);
        IMU  -> releaseLink(i);
    }

    // Get the chain objects
    chainNeck = neck -> asChain();
    chainEyeL = eyeL -> asChain();
    chainEyeR = eyeR -> asChain();
    chainIMU  = IMU  -> asChain();

    inTorsoPort = new BufferedPort<Bottle>;
    inIMUPort = new BufferedPort<Bottle>;

    xFP_R.resize(3,0.0);
    J_E.resize(3,3);
    J_E.zero();
}

bool gazeStabilizerThread::threadInit()
{
    inTorsoPort -> open(("/"+name+"/torsoController:i").c_str());
    inIMUPort   -> open(("/"+name+"/inertial:i").c_str());

    Network::connect("/torsoController/gazeStabilizer:o",("/"+name+"/torsoController:i").c_str());
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
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

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
    /*
    * Conceptual recap:
    * 1 - Read encoders for torso and head
    * 2 - Update the iCubHeadCenter, eyeR and eyeL with those values
    * 3 - Compute Fixation Point Data (x_FP and J_E)
    */
    if (isRunning)
    {
        // 1 - Read the encoders for torso and head
        iencsT->getEncoders(encsT->data());
        iencsH->getEncoders(encsH->data());

        // 2 - Update the iCubHeadCenter, eyeR and eyeL with those values
        updateEyeChain(*chainEyeL,"left");
        updateEyeChain(*chainEyeR,"right");
        updateEyeChain(*chainNeck,"right");
        updateIMUChain(*chainIMU);
        printMessage(2,"EyeL: %s\n",(CTRL_RAD2DEG*(eyeL->getAng())).toString().c_str());
        printMessage(2,"EyeR: %s\n",(CTRL_RAD2DEG*(eyeR->getAng())).toString().c_str());
        printMessage(2,"Neck: %s\n",(CTRL_RAD2DEG*(neck->getAng())).toString().c_str());
        printMessage(2,"IMU:  %s\n",(CTRL_RAD2DEG*(IMU ->getAng())).toString().c_str());


        // 3 - Compute Fixation Point Data (x_FP and J_E)
        if (CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,xFP_R,J_E))
        {
            printMessage(1,"xFP_R:\t%s\n", xFP_R.toString().c_str());
            printMessage(1,"J_E:\n%s\n\n",   J_E.toString().c_str());

            // At this point, compute the source-dependent tasks
            if (src_mode == "torso")
            {
                run_torsoMode();
            }
            else if (src_mode == "inertial")
            {
                run_inertialMode();
            }
        }
    }
}

bool gazeStabilizerThread::startStabilization()
{
    isRunning = true;
    return true;
}

bool gazeStabilizerThread::stopStabilization()
{
    isRunning = false;
    return true;
}

void gazeStabilizerThread::run_inertialMode()
{
    /*
    * Conceptual recap:
    * 4  - Read data from the inertial sensor
    *      (if there is no data, nothing is commanded)
    * 5  - Compute the lever arm between the fixation point and the IMU
    * 6A - PUT THE MAGIC HERE ...
    * 6B - ... AND HERE!
    * 8  - Send dq_E
    */

    if (inIMUBottle = inIMUPort->read(false))
    {
        // 4  - Read data from the inertial sensor
        //     (if there is no data, nothing is commanded)
        double gyrX = inIMUBottle -> get(6).asDouble();
        double gyrY = inIMUBottle -> get(7).asDouble();
        double gyrZ = inIMUBottle -> get(8).asDouble();

        // 5  - Compute the lever arm between the fixation point and the IMU
        Matrix H = IMU -> getH();
        H(0,3) = xFP_R[0]-H(0,3);
        H(1,3) = xFP_R[1]-H(1,3);
        H(2,3) = xFP_R[2]-H(2,3);

        // 6A - Filter out the noise on the gyro readouts
        Vector vor_fprelv;
        if ((fabs(gyrX)<GYRO_BIAS_STABILITY) && (fabs(gyrY)<GYRO_BIAS_STABILITY) &&
            (fabs(gyrZ)<GYRO_BIAS_STABILITY))
            vor_fprelv.resize(J_E.rows(),0.0);    // pinv(J_E) => use rows
        // 6B - Do the magic 
        else
            vor_fprelv=CTRL_DEG2RAD*(gyrX*cross(H,0,H,3)+gyrY*cross(H,1,H,3)+gyrZ*cross(H,2,H,3));

        Matrix J_E_pinv = pinv(J_E);
        printMessage(1,"J_E_pinv:\n%s\n",J_E_pinv.toString().c_str());
        Vector dq_E = CTRL_RAD2DEG * 1.0 * (J_E_pinv*vor_fprelv);
        printMessage(0,"dq_E:\t%s\n", dq_E.toString().c_str());

        // 9 - Send dq_E
        moveEyes(dq_E);
    }
}

void gazeStabilizerThread::run_torsoMode()
{
    /*
    * Conceptual recap:
    * 4 - Read dq_H and dq_T (for now only dq_T;
    *     if there is no dq_T, nothing is commanded)
    * 5 - Convert x_FP from root to RF_E
    * 6 - SetHN() with x_FP
    * 7 - Compute dx_FP = J_TH * [dq_T ; dq_H]
    * 8 - Compute dq_E  = J_E+ * dx_FP;
    * 9 - Send dq_E
    */

    if (inTorsoBottle = inTorsoPort->read(false))
    {
        // 4 - Read dq_H and dq_T (for now only dq_T;
        //     if there is no dq_T, nothing is commanded)
        Vector dq(6,0.0);
        dq[2] = inTorsoBottle->get(0).asDouble();
        dq[1] = inTorsoBottle->get(1).asDouble();
        dq[0] = inTorsoBottle->get(2).asDouble();
        dq = CTRL_DEG2RAD * dq;

        // 5 - Convert x_FP from root to RF_E
        chainNeck -> setHN(eye(4));
        Matrix H_RE = chainNeck->getH();        // matrix from root to RF_E
        xFP_R.push_back(1);
        Vector xFP_E = SE3inv(H_RE) * xFP_R;
        xFP_R.pop_back();
        printMessage(1,"xFP_E:\t%s\n", xFP_E.toString().c_str());

        // 6 - SetHN() with xFP_E
        Matrix HN = eye(4);
        HN(0,3)   = xFP_E(0);
        HN(1,3)   = xFP_E(1);
        HN(2,3)   = xFP_E(2);
        chainNeck -> setHN(HN);

        // 7 - Compute dx_FP = J_TH * [dq_T ; dq_H]
        Matrix J_TH = chainNeck -> GeoJacobian();
        printMessage(1,"J_TH:\n%s\n",J_TH.toString().c_str());
        Vector dx_FP = J_TH * dq;
        printMessage(1,"dx_FP:\t%s\n", dx_FP.toString().c_str());

        // 8 - Compute dq_E  = J_E+ * dx_FP;
        Matrix J_E_pinv = pinv(J_E);
        printMessage(1,"J_E_pinv:\n%s\n",J_E_pinv.toString().c_str());
        Vector dq_E = CTRL_RAD2DEG * (J_E_pinv * dx_FP.subVector(0,2));
        printMessage(0,"dq_E:\t%s\n", dq_E.toString().c_str());

        // 9 - Send dq_E
        moveEyes(dq_E);
    }
}

bool gazeStabilizerThread::moveEyes(const Vector &_dq_E)
{
    // if (norm(_dq_E) < 100)
    // {
        std::vector<int> Ejoints;  // indexes of the joints to control
        Ejoints.push_back(3);
        Ejoints.push_back(4);
        Ejoints.push_back(5);
        printMessage(3,"Head joints to be controlled: %i %i %i\n",Ejoints[0],Ejoints[1],Ejoints[2]);

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
    // }
    // else
    // {
    //     printMessage(1,"Desired velocities are higher (in norm) than 100. No velocity will be sent.\n");
    //     return false;
    // }
    return true;
}

void gazeStabilizerThread::updateEyeChain(iKinChain &_eye, const string _eyeType)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    // CHANGE THIS: Avoid going low with the vergence
    // (this value is empyrical, but it is what the gaze controller is doing internally)
    if (head[5] < 1.0)
        head[5] = 1.0;

    bool isLeft=(_eyeType == "left_v2" ||
                 _eyeType == "left"    ||
                 _eyeType == "left_v1");

    yarp::sig::Vector q(8);
    q[0] = torso[2];   q[1] = torso[1];    q[2] = torso[0];
    q[3] = head[0];    q[4] = head[1];
    q[5] = head[2];    q[6] = head[3];
    if (isLeft)
        q[7] = head[4]+head[5]/2.0;
    else
        q[7] = head[4]-head[5]/2.0;
    q = CTRL_DEG2RAD*q;

    _eye.setAng(q);
}

void gazeStabilizerThread::updateIMUChain(iKinChain &_imu)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    yarp::sig::Vector q(6);
    q[0] = torso[2];   q[1] = torso[1];    q[2] = torso[0];
    q[3] =  head[0];   q[4] =  head[1];    q[5] =  head[2];

    q = CTRL_DEG2RAD*q;
    _imu.setAng(q);
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
    // printMessage(0,"Moving head to home position.. \n");
    //     Vector pos0(6,0.0);
    //     iposH -> positionMove(pos0.data());

    printMessage(0,"Closing ports...\n");
        inTorsoPort->close();
        inIMUPort->close();

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

}

// empty line to make gcc happy
