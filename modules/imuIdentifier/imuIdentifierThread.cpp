#include "imuIdentifierThread.h"

#define CTRL_PERIOD 2

string int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

wayPoint::wayPoint()
{
    name = "";
    jntlims.resize(3,0.0);
    vels.resize(3,0.0);
}

wayPoint::wayPoint(string _name)
{
    name = _name;
    jntlims.resize(3,0.0);
    vels.resize(3,0.0);
}

wayPoint::wayPoint(string _name, Vector _jntlims, Vector _vels)
{
    name    = _name;
    jntlims = _jntlims;
    vels    = _vels;
}

wayPoint & wayPoint::operator= (const wayPoint &jv)
{
    name    = jv.name;
    jntlims = jv.jntlims;
    vels    = jv.vels;
}

void wayPoint::print()
{
    printf("*** %s\n",name.c_str());
    printf("jntl: %s\n",jntlims.toString(3,3).c_str());
    printf("vels: %s\n",vels.toString(3,3).c_str());
    printf("**********\n");
}

void wayPoint::printCompact()
{
    printf("*** %s\t",name.c_str());
    printf("jntl: %s\t",jntlims.toString(3,3).c_str());
    printf("vels: %s\n",vels.toString(3,3).c_str());
}

imuIdentifierThread::imuIdentifierThread(int _rate, string _name, string _robot, int _v, int _nW, const ResourceFinder &_rf) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v), numWaypoints(_nW)
{
    timeNow = yarp::os::Time::now();
    ResourceFinder &rf = const_cast<ResourceFinder&>(_rf);
    step               = 0;
    currentWaypoint    = 0;

    //******************* ITERATIONS ******************
    iterations=rf.check("iterations",Value(1)).asInt();
    printf(("*** "+name+": number of iterations set to %g\n").c_str(),iterations);
}

bool imuIdentifierThread::threadInit()
{
    outPort.open(("/"+name+"/gazeStabilizer:o").c_str());
    GSrpcPort.open(("/"+name+"/rpc:o").c_str());
    Network::connect(("/"+name+"/gazeStabilizer:o").c_str(),"/gazeStabilizer/torsoController:i");
    Network::connect(("/"+name+"/rpc:o").c_str(),"/gazeStabilizer/rpc:i");

    bool ok = 1;
    Property OptH;
    OptH.put("robot",  robot.c_str());
    OptH.put("part",   "torso");
    OptH.put("device", "remote_controlboard");
    OptH.put("remote",("/"+robot+"/torso").c_str());
    OptH.put("local", ("/"+name +"/torso").c_str());

    if (!ddH.open(OptH))
    {
        printMessage(0,"ERROR: could not open torso PolyDriver!\n");
        return false;
    }

    if (ddH.isValid())
    {
        ok = ok && ddH.view(iposH);
        ok = ok && ddH.view(ivelH);
        ok = ok && ddH.view(iencsH);
        ok = ok && ddH.view(imodH);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring torso interfaces!!!!\n");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    return true;
}

void imuIdentifierThread::run()
{
    switch (step)
    {
        case 0:
            Time::delay(0.1); // only to avoid a printing issue in the terminal
            printMessage(0,"Starting..\n");
            step++;
            break;
        case 4:
            printMessage(0,"Finished. Stopping stabilization..\n");
            step++;
            break;              
        default:
            printMessage(1,"Finished.\n");
            break;
    }
}

bool imuIdentifierThread::setHeadCtrlModes(const string _s)
{
    if (_s!="position" || _s!="velocity")
        return false;

    VectorOf<int> jointsToSet;
    jointsToSet.push_back(0);
    jointsToSet.push_back(1);
    jointsToSet.push_back(2);
    VectorOf<int> modes;

    if (_s=="position")
    {
        modes.push_back(VOCAB_CM_POSITION);
        modes.push_back(VOCAB_CM_POSITION);
        modes.push_back(VOCAB_CM_POSITION);
    }
    else if (_s=="velocity")
    {
        modes.push_back(VOCAB_CM_VELOCITY);
        modes.push_back(VOCAB_CM_VELOCITY);
        modes.push_back(VOCAB_CM_VELOCITY);
    }

    imodH -> setControlModes(jointsToSet.size(),
                             jointsToSet.getFirst(),
                             modes.getFirst());

    return true;
}

bool imuIdentifierThread::goHome()
{
    setHeadCtrlModes("position");
    Vector pos0(6,0.0);
    iposH -> positionMove(pos0.data());
    return true;
}

bool imuIdentifierThread::redoCycle()
{
    if (step <= 4)
    {
        return false; // it means that it didn't finish its cycle yet
    }
    else
    {
        step            = 0;
        return true;
    }
    return true;
}

int imuIdentifierThread::printMessage(const int l, const char *f, ...) const
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

void imuIdentifierThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void imuIdentifierThread::threadRelease()
{
    printMessage(0,"Putting torso in home position..\n");
        goHome();

    printMessage(0,"Closing controllers..\n");
        ddH.close();
}

// empty line to make gcc happy
