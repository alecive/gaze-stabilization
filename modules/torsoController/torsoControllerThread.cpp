#include "torsoControllerThread.h"

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


torsoControllerThread::torsoControllerThread(int _rate, string _name, string _robot, int _v, int _nW, const ResourceFinder &_rf) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v), numWaypoints(_nW)
{
    timeNow = yarp::os::Time::now();
    cmdcnt  = -2;
    ResourceFinder &rf = const_cast<ResourceFinder&>(_rf);
    step               = 0;
    currentWaypoint    = 0;

    //******************* ITERATIONS ******************
    iterations=rf.check("iterations",Value(1)).asInt();
    printf(("*** "+name+": number of iterations set to %g\n").c_str(),iterations);

    //******************* WAYPOINTS ******************
    wayPoints.push_back(wayPoint("START      "));        // The first group is the home position

    for (int j = 0; j < iterations; j++)
    {
        for (int i = 0; i < numWaypoints; i++)
        {
            string wayPointName = "WAYPOINT_"+int_to_string(i);
            
            Bottle &b = rf.findGroup(wayPointName.c_str());
            if (!b.isNull())
            {
                printMessage(1,"%s found: %s\n",wayPointName.c_str(),b.toString().c_str());
                Vector _jntlims(3,0.0);
                Vector _vels(3,0.0);
                Bottle *bj = b.find("jntlims").asList();
                Bottle *bv = b.find("vels").asList();

                for (int j = 0; j < _jntlims.size(); j++)
                {
                    _jntlims[j] = bj->get(j).asDouble();
                    _vels[j] = bv->get(j).asDouble();
                }
                wayPoints.push_back(wayPoint(wayPointName,_jntlims,_vels));
            }
        }
    }

    wayPoints.push_back(wayPoint("END        "));   // The last group will be the home position as well
    numWaypoints = wayPoints.size();                // The number of waypoints is simply the size of the vector

    for (int i = 0; i < numWaypoints; i++)
    {
        wayPoints[i].print();
    }
}

bool torsoControllerThread::threadInit()
{
    outPort.open(("/"+name+"/gazeStabilizer:o").c_str());
    GSrpcPort.open(("/"+name+"/rpc:o").c_str());
    Network::connect(("/"+name+"/gazeStabilizer:o").c_str(),"/gazeStabilizer/torsoController:i");
    Network::connect(("/"+name+"/rpc:o").c_str(),"/gazeStabilizer/rpc:i");

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
        ok = ok && ddT.view(iencsT);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring torso interfaces!!!!\n");
        return false;
    }

    iencsT -> getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

    return true;
}

void torsoControllerThread::run()
{
    switch (step)
    {
        case 0:
            Time::delay(0.1); // only to avoid a printing issue in the terminal
            printMessage(0,"Starting..\n");
            printMessage(0,"Going to wayPoint #%i: ",currentWaypoint);
            wayPoints[currentWaypoint].printCompact();
            step++;
            break;
        case 1:
            if (!processWayPoint())
            {
                printMessage(0,"Starting stabilization..\n");
                gateStabilization("start");
                currentWaypoint++;
                step++;
                timeNow = yarp::os::Time::now();
            }
            break;
        case 2:
            printMessage(0,"Going to wayPoint #%i: ",currentWaypoint);
            wayPoints[currentWaypoint].printCompact();
            step++;
            break;
        case 3:
            if (!processWayPoint())
            {
                timeNow = yarp::os::Time::now();
                currentWaypoint++;
                if (currentWaypoint < numWaypoints)
                {
                    step = 2;
                }
                else
                    step++;
            }
            break;
        case 4:
            printMessage(0,"Finished. Stopping stabilization..\n");
            gateStabilization("stop");
            step++;
            break;              
        default:
            printMessage(1,"Finished.\n");
            break;
    }
}

bool torsoControllerThread::processWayPoint()
{
    if (wayPoints[currentWaypoint].name == "START      " ||
        wayPoints[currentWaypoint].name == "END        ")
    {
        printMessage(1,"Putting torso in home position..\n");
        Vector pos0(3,0.0);
        iposT -> positionMove(pos0.data());

        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            return false;
        }
    }
    else
    {
        Vector jls = wayPoints[currentWaypoint].jntlims;
        Vector vls = wayPoints[currentWaypoint].vels;
        bool flag = false;

        iencsT->getEncoders(encsT->data());
        yarp::sig::Vector torso = *encsT;

        ivelT -> velocityMove(vls.data());
        sendCommand();

        for (int i = 0; i < 3; i++)
        {
            if      (vls(i) > 0.0)
            {
                if (jls(i) - torso(i) > 0.0)
                {
                    flag = true;
                }
            }
            else if (vls(i) < 0.0)
            {
                if (jls(i) - torso(i) < 0.0)
                {
                    flag = true;
                }
            }
        }

        return flag;
    }

    return true;
}

bool torsoControllerThread::redoCycle()
{
    if (step <= 4)
    {
        return false; // it means that it didn't finish its cycle yet
    }
    else
    {
        step            = 0;
        currentWaypoint = 0;
        return true;
    }
    return true;
}

void torsoControllerThread::gateStabilization(const string _g)
{
    Bottle cmdGS;
    Bottle respGS;
    cmdGS.clear();
    respGS.clear();
    cmdGS.addString(_g);
    GSrpcPort.write(cmdGS,respGS);
}

void torsoControllerThread::sendCommand()
{
    Bottle b;
    b.clear();
    for (size_t i = 0; i < 3; i++)
    {
        b.addDouble(wayPoints[currentWaypoint].vels(i));
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
    printMessage(0,"Putting torso in home position..\n");
        Vector pos0(3,0.0);
        iposT -> positionMove(pos0.data());

    printMessage(0,"Stopping gazeStabilizer..\n");
        gateStabilization("stop");

    printMessage(0,"Closing controllers..\n");
        ddT.close();
}

// empty line to make gcc happy
