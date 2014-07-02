#include "gazeStabilizerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

gazeStabilizerThread::gazeStabilizerThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name),
                                           robot(_robot), verbosity(_v)
{
    eyeL = new iCubEye("left_v2");
    eyeR = new iCubEye("right_v2");
    neck = new iCubHeadCenter("right_v2");

    // Release torso links
    for (int i = 0; i < 3; i++)
    {
        eyeL -> releaseLink(i);
        eyeR -> releaseLink(i);
        neck -> releaseLink(i);
    }

    // Get the chain objects
    chainNeck = neck -> asChain();
    chainEyeL = eyeL -> asChain();
    chainEyeR = eyeR -> asChain();

    inTorsoPort = new BufferedPort<Bottle>;
}

bool gazeStabilizerThread::threadInit()
{
    inTorsoPort -> open(("/"+name+"/torsoController:i").c_str());
    Network::connect("/torsoController/gazeStabilizer:o",("/"+name+"/torsoController:i").c_str());

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
        ok = ok && ddH->view(ivelH);
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

    return true;
}

void gazeStabilizerThread::run()
{
    /*
    * Conceptual recap:
    * 1 - Read encoders for torso and head
    * 2 - Update the iCubHeadCenter R and L with those values
    * 3 - Compute Fixation Point Data (x_FP and J_E)
    * 4 - Convert x_FP from root to RF_E
    * 5 - SetHN() with x_FP
    * 6 - Read dq_H and dq_T
    * 7 - Compute dx_FP = J_TH * [dq_T ; dq_H]
    * 8 - Compute dq_E  = J_E+ * dx_FP;
    * 9 - Send dq_E
    */

    // 1 - Read encoders for torso and head
    iencsT->getEncoders(encsT->data());
    iencsH->getEncoders(encsH->data());

    // 2 - update the iCubHeadCenter R and L with those values
    updateEyeChain(*chainEyeL,"left");
    updateEyeChain(*chainEyeR,"right");
    printMessage(1,"EyeL: %s\n",(CTRL_RAD2DEG*(eyeL->getAng())).toString().c_str());
    printMessage(1,"EyeR: %s\n",(CTRL_RAD2DEG*(eyeR->getAng())).toString().c_str());

    // 3 - Get fixation point (x_FP and J_E)
    Vector xFP_R(3,0.0);
    Matrix J_E(3,3);
    if (cartHlp.computeFixationPointData(*chainEyeL,*chainEyeR,xFP_R,J_E))
    {
        printMessage(0,"xFP_R:\t%s\n",  xFP_R.toString().c_str());
        printMessage(1,"J_E:\n%s\n\n", J_E.toString().c_str());
    }

    // 4 - Convert x_FP from root to RF_E
    chainNeck -> setHN(eye(4));
    Matrix H_RE = chainNeck->getH();        // matrix from root to RF_E
    xFP_R.push_back(1);
    Vector xFP_E = SE3inv(H_RE) * xFP_R;
    xFP_R.pop_back();
    printMessage(0,"xFP_E:\t%s\n", xFP_E.toString().c_str());

    // 5 - SetHN() with xFP_E
    Matrix HN = eye(4);
    HN(0,3)   = xFP_E(0);
    HN(1,3)   = xFP_E(1);
    HN(2,3)   = xFP_E(2);
    chainNeck -> setHN(HN);

    // 6 - Read dq_H and dq_T (for now only dq_T)
    Vector dq(6,0.0);
    if (inTorsoBottle = inTorsoPort->read(false))
    {
        dq[0] = inTorsoBottle->get(0).asDouble();
        dq[1] = inTorsoBottle->get(1).asDouble();
        dq[2] = inTorsoBottle->get(2).asDouble();
    }

    // 7 - Compute dx_FP = J_TH * [dq_T ; dq_H]
    Matrix J_TH = chainNeck -> GeoJacobian();
    printMessage(1,"J_TH:\n%s\n",J_TH.toString().c_str());
    Vector dx_FP = J_TH * dq;
    printMessage(0,"dx_FP:\t%s\n", dx_FP.toString().c_str());

    // 8 - Compute dq_E  = J_E+ * dx_FP;
    Matrix J_E_pinv = pinv(J_E);
    printMessage(0,"J_E_pinv:\n%s\n",J_E_pinv.toString().c_str());
    Vector dq_E = J_E_pinv * dx_FP.subVector(0,2);
    printMessage(0,"dq_E:\t%s\n", dq_E.toString().c_str());

    // 9 - Send dq_E
    int nJnts = 3;

    std::vector<int> Ejoints;  // indexes of the joints to control
    Ejoints.push_back(0);
    Ejoints.push_back(1);
    Ejoints.push_back(2);

    ivelH -> velocityMove(nJnts,Ejoints.data(),dq_E.data());
}

void gazeStabilizerThread::updateEyeChain(iKinChain &_eye, const string _eyeType)
{
    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    // CHANGE THIS: Avoid going low with the vergence
    // (this value is empyrical, but it is what the gaze controller is doing internally)
    if (head[5] < 0.06)
        head[5] = 0.06;

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
