#include "torsoControllerThread.h"

#define CTRL_PERIOD 2

torsoControllerThread::torsoControllerThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name),
                                           robot(_robot), verbosity(_v)
{
    timeNow = yarp::os::Time::now();
    cmdcnt  = -2;

    Vector vec(3,0.0);
    
    vec[0] =  10;
    ctrlCommands.push_back(vec);

    vec[0] = -10;
    ctrlCommands.push_back(vec);
    ctrlCommands.push_back(vec);

    vec[0] =  10;
    ctrlCommands.push_back(vec);

    vec[0] =   0;
    vec[2] =  10;
    ctrlCommands.push_back(vec);
    ctrlCommands.push_back(vec);

    vec[2] = -10;
    ctrlCommands.push_back(vec);
    ctrlCommands.push_back(vec);    

    vec.zero();
    ctrlCommands.push_back(vec);    
}

bool torsoControllerThread::threadInit()
{
    outPort.open(("/"+name+"/gazeStabilizer:o").c_str());
    Network::connect(("/"+name+"/gazeStabilizer:o").c_str(),"/gazeStabilizer/torsoController:i");

    bool ok = 1;
    Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name +"/torso").c_str());

    if (!ddT.open(OptT))
    {
        printMessage(0,"ERROR: could not open torso PolyDriver!\n");
        return false;
    }

    if (ddT.isValid())
    {
        ok = ok && ddT.view(iposT);
        ok = ok && ddT.view(ivelT);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring torso interfaces!!!!\n");
        return false;
    }

    return true;
}

void torsoControllerThread::run()
{
    if (cmdcnt == -2)   // put torso in 0 0 0 (positionMove)
    {
        printMessage(0,"Putting torso in home position..\n");
        Vector pos0(3,0.0);
        iposT -> positionMove(pos0.data());
        timeNow = yarp::os::Time::now();
        cmdcnt +=1;
    }
    else if (cmdcnt == -1)
    {
        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            timeNow = yarp::os::Time::now();
            cmdcnt += 1;
            printMessage(0,"Sending command #%i\n",cmdcnt);
        }
    }
    else if (cmdcnt < ctrlCommands.size())
    {
        ivelT -> velocityMove(ctrlCommands[cmdcnt].data());
        sendCommand();

        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            timeNow = yarp::os::Time::now();
            cmdcnt += 1;
            printMessage(0,"Sending command #%i\n",cmdcnt);
        }
    }
    else
    {
        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            timeNow = yarp::os::Time::now();
            printMessage(0,"Finished.\n");
        }
    }
}

void torsoControllerThread::sendCommand()
{
    Bottle b;
    b.clear();
    for (size_t i = 0; i < 3; i++)
    {
        b.addDouble(ctrlCommands[cmdcnt](i));
    }
    outPort.write(b);
}

int torsoControllerThread::printMessage(const int l, const char *f, ...) const
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

void torsoControllerThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void torsoControllerThread::threadRelease()
{
    printMessage(0,"Closing controllers..\n");
        ddT.close();
}

// empty line to make gcc happy
