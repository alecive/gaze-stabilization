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

    inIMUPort   = new BufferedPort<Bottle>;
    outPort     = new BufferedPort<Bottle>;

    //******************* ITERATIONS ******************
    iterations=rf.check("iterations",Value(1)).asInt();
    printf(("*** "+name+": number of iterations set to %g\n").c_str(),iterations);

    //******************* WAYPOINTS ******************
    wayPoints.push_back(wayPoint("START     "));        // The first group is the home position

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
                wayPoints.push_back(wayPoint("MIDDLE    "));    // In order to come back every time
            }
        }
    }

    wayPoints.pop_back();
    wayPoints.push_back(wayPoint("END       "));   // The last group will be the home position as well
    numWaypoints = wayPoints.size();                // The number of waypoints is simply the size of the vector

    for (int i = 0; i < numWaypoints; i++)
    {
        wayPoints[i].print();
    }
}

bool imuIdentifierThread::threadInit()
{
    outPort  ->open(("/"+name+"/IMU:o").c_str());
    inIMUPort->open(("/"+name+"/inertial:i").c_str());

    Network::connect(("/"+robot+"/inertial").c_str(),("/"+name+"/inertial:i").c_str());

    bool ok = 1;
    Property OptH;
    OptH.put("robot",  robot.c_str());
    OptH.put("part",   "head");
    OptH.put("device", "remote_controlboard");
    OptH.put("remote",("/"+robot+"/head").c_str());
    OptH.put("local", ("/"+name +"/head").c_str());

    if (!ddH.open(OptH))
    {
        printMessage(0,"ERROR: could not open head PolyDriver!\n");
        return false;
    }

    if (ddH.isValid())
    {
        ok = ok && ddH.view(iposH);
        ok = ok && ddH.view(ivelH);
        ok = ok && ddH.view(iencsH);
        ok = ok && ddH.view(imodH);
        ok = ok && ddH.view(ipidH);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    Vector headAcc(jntsH,1e9);
    ivelH -> setRefAccelerations(headAcc.data());

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
        case 1:
            printMessage(0,"Going to wayPoint #%i: ",currentWaypoint);
            setHeadCtrlModes("velocity");
            wayPoints[currentWaypoint].printCompact();
            step++;
            break;
        case 2:
            if (!processWayPoint())
            {
                timeNow = yarp::os::Time::now();
                currentWaypoint++;
                if (currentWaypoint < numWaypoints)
                {
                    step = 1;
                }
                else
                    step++;
            }
            break;
        case 3:
            printMessage(0,"Finished.\n");
            step++;
            break;              
        default:
            printMessage(1,"Finished.\n");
            break;
    }
}

bool imuIdentifierThread::setHeadCtrlModes(const string _s)
{
    printMessage(1,"Setting %s mode for head joints..\n",_s.c_str());

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

bool imuIdentifierThread::processIMU()
{
    if (inIMUBottle = inIMUPort->read(false))
    {
        Vector w(3,0.0);
        double gyrX = inIMUBottle -> get(6).asDouble(); w[0] = gyrX;
        double gyrY = inIMUBottle -> get(7).asDouble(); w[1] = gyrY;
        double gyrZ = inIMUBottle -> get(8).asDouble(); w[2] = gyrZ;

        iencsH->getEncoders(encsH->data());

        printMessage(1,"Gyro: \t%s\n",w.toString(3,3).c_str());

        Bottle &b=outPort->prepare();
        b.clear();
        for (size_t i = 0; i < 3; i++)
        {
            b.addDouble(wayPoints[currentWaypoint].vels(i));
        }
        for (size_t i = 0; i < 3; i++)
        {
            double ref;
            ipidH->getReference(i,&ref);
            b.addDouble(ref);
        }
        for (size_t i = 0; i < 3; i++)
        {
            b.addDouble((*encsH)(i));
        }
        for (size_t i = 0; i < 3; i++)
        {
            b.addDouble(w(i));
        }
        outPort->write();

        return true;
    }
    return false;
}

bool imuIdentifierThread::goHome()
{
    setHeadCtrlModes("position");
    Vector pos0(6,0.0);
    iposH -> positionMove(pos0.data());
    return true;
}

bool imuIdentifierThread::processWayPoint()
{
    processIMU();
    if (wayPoints[currentWaypoint].name == "START     " ||
        wayPoints[currentWaypoint].name == "END       " ||
        wayPoints[currentWaypoint].name == "MIDDLE    " )
    {
        printMessage(1,"Putting head in home position..\n");
        goHome();

        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            return false;
        }
    }
    else
    {
        Vector jls = wayPoints[currentWaypoint].jntlims;
        Vector vls(6,0.0);
        vls.setSubvector(0,wayPoints[currentWaypoint].vels);
        bool flag = false;

        iencsH->getEncoders(encsH->data());
        yarp::sig::Vector head = *encsH;

        ivelH -> velocityMove(vls.data());

        for (int i = 0; i < 3; i++)
        {
            if      (vls(i) > 0.0)
            {
                if (jls(i) - head(i) > 0.0)
                {
                    flag = true;
                }
            }
            else if (vls(i) < 0.0)
            {
                if (jls(i) - head(i) < 0.0)
                {
                    flag = true;
                }
            }
        }

        return flag;
    }

    return true;
}


bool imuIdentifierThread::redoCycle()
{
    if (step <= 3)
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
        fprintf(stdout,"*** %s:  ",name.c_str());

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
    printMessage(0,"Putting head in home position..\n");
        goHome();

    printMessage(0,"Closing controllers..\n");
        ddH.close();

    printMessage(0,"Closing ports...\n");
        closePort(inIMUPort);
        closePort(outPort);
}

// empty line to make gcc happy
