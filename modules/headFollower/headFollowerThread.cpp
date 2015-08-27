#include "headFollowerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define GYRO_BIAS_STABILITY_IMU_CALIB   1.1     // [deg/s]
#define GYRO_BIAS_STABILITY             4.0     // [deg/s]

headFollowerThread::headFollowerThread(int _rate, string &_name, string &_robot,
                                       int _v, string &_if_mode, bool _isRunning) :
                                       RateThread(_rate), name(_name), robot(_robot),
                                       verbosity(_v), if_mode(_if_mode), isRunning(_isRunning)
{
    
}

bool headFollowerThread::threadInit()
{
    inGlassPort.open(("/"+name+"/glassensor:i").c_str());
    Network::connect("/yarpdroid/glassensor:o",("/"+name+"/glassensor:i").c_str());

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

    return true;
}

void headFollowerThread::run()
{
    /*
    * Conceptual recap:
    * 1 - Read encoders for torso and head
    * 2 - Update the iCubHeadCenter, eyeR and eyeL with those values
    * 3 - Compute Fixation Point Data (x_FP and J_E)
    */
    if (isRunning)
    {
        // 3 - Read from the glass port. If there is something, send the command to the neck
        if (inGlassBottle = inGlassPort.read(false))
        {
            Vector w(3,0.0);
            // yInfo("Received: %s",inGlassBottle->toString().c_str());

            double gyrX = inGlassBottle -> get(3).asDouble() * CTRL_RAD2DEG; w[0] = gyrX;
            double gyrY = inGlassBottle -> get(4).asDouble() * CTRL_RAD2DEG; w[2] = gyrY;
            double gyrZ = inGlassBottle -> get(5).asDouble() * CTRL_RAD2DEG; w[1] = gyrZ;

            yInfo("Moving head with: %s",w.toString(3,3).c_str());

            moveNeck(w);
        }
        // else
        // {
        //     yDebug("Nothing to be done here..\n");
        // }
    }
}

bool headFollowerThread::moveNeck(const Vector &_dq_H)
{
    VectorOf<int> jointsToSet;
    if (!areJointsHealthyAndSet(jointsToSet,"velocity"))
    {
        stopFollowing();
        return false;
    }
    else
    {
        printf("I'm setting head control modes: %i\n",setHeadCtrlModes(jointsToSet,"velocity"));
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
        yWarning("if_mode is neither vel1 or vel2. No velocity will be sent.");
        return false;
    }

    return true;
}

bool headFollowerThread::areJointsHealthyAndSet(VectorOf<int> &jointsToSet,const string &_s)
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

bool headFollowerThread::setHeadCtrlModes(const VectorOf<int> &jointsToSet,const string &_s)
{
    if (_s!="position" && _s!="velocity")
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

bool headFollowerThread::set_if_mode(const string &_ifm)
{
    if (_ifm == "vel1" || _ifm == "vel2")
    {
        if_mode = _ifm;
        return true;
    }
    return false;
}

bool headFollowerThread::startFollowing()
{
    isRunning = true;
    return true;
}

bool headFollowerThread::stopFollowing()
{
    isRunning = false;
    return true;
}

bool headFollowerThread::goHome()
{
    if (!isRunning)
    {
        VectorOf<int> jointsToSet;
        if (!areJointsHealthyAndSet(jointsToSet,"position"))
        {
            stopFollowing();
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

int headFollowerThread::printMessage(const int l, const char *f, ...) const
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

void headFollowerThread::closePort(Contactable &_port)
{
    _port.interrupt();
    _port.close();
}

void headFollowerThread::threadRelease()
{
    yInfo("  Moving head to home position..");
        stopFollowing();
        goHome();

    yInfo("  Closing ports...");
        closePort(inGlassPort);

    yInfo("  Closing controllers..");
        ddH->close();
        delete ddH;
        ddH = NULL;

        if (encsH)
        {
            delete encsH;
            encsH = NULL;
        }
}

// empty line to make gcc happy
