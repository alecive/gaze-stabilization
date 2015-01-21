#include "torsoControllerThread.h"

#define CTRL_PERIOD 1.0

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
    printf("[%s]\n",name.c_str());
    printf("jntl: %s\n",jntlims.toString(3,3).c_str());
    printf("vels: %s\n",vels.toString(3,3).c_str());
    printf("**********\n");
}

void wayPoint::printCompact()
{
    printf("[%s]    ",name.c_str());
    printf("jntl: %s\t",jntlims.toString(3,3).c_str());
    printf("vels: %s\n",vels.toString(3,3).c_str());
}

yarp::os::ConstString wayPoint::toString()
{
    yarp::os::ConstString result = "["+name+"]  jntl: "+jntlims.toString(3,3).c_str()+"\tvels: "+vels.toString(3,3).c_str();
    return result;
}


torsoControllerThread::torsoControllerThread(int _rate, string _name, string _robot, int _v, int _nW, const ResourceFinder &_rf) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v), numWaypoints(_nW)
{
    neck = new iCubHeadCenter("right_v2");
    neck -> setAllConstraints(false);
    // Release torso links
    for (int i = 0; i < 3; i++)
    {
        neck -> releaseLink(i);
    }
    chainNeck = neck -> asChain();

    timeNow = yarp::os::Time::now();
    ResourceFinder &rf = const_cast<ResourceFinder&>(_rf);
    step               = 0;
    currentWaypoint    = 0;

    //******************* ITERATIONS ******************
    iterations=rf.check("iterations",Value(1)).asInt();
    printMessage(0,"Number of iterations set to %i\n",iterations);

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
            }
        }
    }

    wayPoints.push_back(wayPoint("END       "));   // The last group will be the home position as well
    numWaypoints = wayPoints.size();               // The number of waypoints is simply the size of the vector

    for (int i = 0; i < numWaypoints; i++)
    {
        wayPoints[i].print();
    }
}

bool torsoControllerThread::threadInit()
{
    outPortQTorso.open(("/"+name+"/torsoVels:o").c_str());
    outPortVNeck.open(("/"+name+"/neckVel:o").c_str());
    gazeStabRPC.open(("/"+name+"/rpc:o").c_str());

    // Network::connect(("/"+name+"/torsoVels:o").c_str(),"/gazeStabilizer/torsoController:i");
    Network::connect(("/"+name+"/neckVel:o").c_str(),"/gazeStabilizer/wholeBody:i");
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
        ok = ok && ddT.view(imodT);
        ok = ok && ddT.view(ilimT);
    }

    if (!ok)
    {
        yError("Problems acquiring torso interfaces!!!!");
        return false;
    }

    iencsT -> getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

    Vector headAcc(jntsT,1e9);
    ivelT -> setRefAccelerations(headAcc.data());

    Vector headVel(jntsT,10.0);
    iposT -> setRefSpeeds(headVel.data());
    iposT -> setRefAccelerations(headAcc.data());

    Property OptH;
    OptH.put("robot",  robot.c_str());
    OptH.put("part",   "head");
    OptH.put("device", "remote_controlboard");
    OptH.put("remote",("/"+robot+"/head").c_str());
    OptH.put("local", ("/"+name +"/head").c_str());

    if (!ddH.open(OptH))
    {
        yError("could not open head PolyDriver!\n");
        return false;
    }

    // open the view
    if (ddH.isValid())
    {
        ok = ok && ddH.view(iencsH);
        ok = ok && ddH.view(ilimH);
    }

    if (!ok)
    {
        yError(" Problems acquiring head interfaces!!!!");
        return false;
    }

    iencsH -> getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    // joints bounds alignment
    deque<IControlLimits*> lim;
    lim.push_back(ilimT);
    lim.push_back(ilimH);

    neck -> alignJointsBounds(lim);

    return true;
}

void torsoControllerThread::run()
{
    switch (step)
    {
        case 0:
            Time::delay(0.1); // only to avoid a printing issue in the terminal
            yInfo("  Starting..");
            yInfo("  Going to wayPoint #%i: %s",currentWaypoint,wayPoints[currentWaypoint].toString().c_str());
            step++;
            break;
        case 1:
            if (!processWayPoint())
            {
                yDebug(" Starting stabilization..\n");
                gateStabilization("start");
                currentWaypoint++;
                step++;
                timeNow = yarp::os::Time::now();
            }
            break;
        case 2:
            yInfo("  Going to wayPoint #%i: %s",currentWaypoint,wayPoints[currentWaypoint].toString().c_str());
            setTorsoCtrlModes("velocity");
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
            yInfo(0,"  Finished. Stopping stabilization..\n");
            gateStabilization("stop");
            step++;
            break;              
        default:
            printMessage(2,"Finished.\n");
            break;
    }
}

bool torsoControllerThread::processWayPoint()
{
    sendTorsoVels();
    sendNeckVel();
    if (wayPoints[currentWaypoint].name == "START     ")
    {
        yDebug(" Putting torso in home position..");
        goHome();

        if (yarp::os::Time::now() - timeNow > CTRL_PERIOD)
        {
            return false;
        }
    }
    else if (wayPoints[currentWaypoint].name == "END       ")
    {
        yDebug(" Putting torso in home position..");
        goHome();
        return false;
    }
    else
    {
        Vector jls = wayPoints[currentWaypoint].jntlims;
        Vector vls = wayPoints[currentWaypoint].vels;
        bool flag = false;

        iencsT->getEncoders(encsT->data());
        yarp::sig::Vector torso = *encsT;

        ivelT -> velocityMove(vls.data());

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

void torsoControllerThread::sendTorsoVels()
{
    Bottle &b = outPortQTorso.prepare();

    for (size_t i = 0; i < 3; i++)
    {
        b.addDouble(wayPoints[currentWaypoint].vels(i));
    }

    outPortQTorso.write();
}

void torsoControllerThread::sendNeckVel()
{
    updateNeckChain(*chainNeck);
    printMessage(2,"qNeck:\t%s\n",(CTRL_RAD2DEG*(neck->getAng())).toString(3,3).c_str());

    // Find the jacobian of the torso:
    Matrix H_T = chainNeck->getH(2);
    printMessage(4,"H_T:\n%s\n", H_T.toString(3,3).c_str());

    Matrix J_T = chainNeck-> GeoJacobian(2);
    printMessage(3,"J_T:\n%s\n", J_T.toString(3,3).c_str());

    Vector neckVel(6,0.0);
    Vector dq_T(3,0.0);
    dq_T[0] = wayPoints[currentWaypoint].vels[2];
    dq_T[1] = wayPoints[currentWaypoint].vels[1];
    dq_T[2] = wayPoints[currentWaypoint].vels[0];

    neckVel = J_T * CTRL_DEG2RAD * dq_T;

    printMessage(1," vNeck:\t%s  %s\n", neckVel.subVector(0,2).toString(3,3).c_str(),
                          (CTRL_RAD2DEG*neckVel.subVector(3,5)).toString(3,3).c_str());

    Bottle neckvelocity;
    neckvelocity.addList().read(neckVel);

    Property& p = outPortVNeck.prepare();
    p.put("Ts",(*this).getRate());
    p.put("neck_velocity",neckvelocity.get(0));

    yTrace(" Sending neck velocities: %s",p.toString().c_str());

    outPortVNeck.write();
}

void torsoControllerThread::updateNeckChain(iKinChain &_neck)
{
    iencsT->getEncoders(encsT->data());
    iencsH->getEncoders(encsH->data());

    yarp::sig::Vector torso = *encsT;
    yarp::sig::Vector  head = *encsH;

    yarp::sig::Vector q(8,0.0);
    q[0] = torso[2];   q[1] = torso[1];   q[2] = torso[0];
    q[3] =  head[0];    q[4] = head[1];    q[5] = head[2];

    q = CTRL_DEG2RAD*q;
    _neck.setAng(q);
}

bool torsoControllerThread::setTorsoCtrlModes(const string _s)
{
    if (_s!="position" && _s!="velocity")
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

    bool res = imodT -> setControlModes(jointsToSet.size(),
                                        jointsToSet.getFirst(),
                                        modes.getFirst());

    yTrace("I have put the torso in %s mode: result %d\n",_s.c_str(),res);

    return true;
}

bool torsoControllerThread::goHome()
{
    setTorsoCtrlModes("position");
    Vector pos0(3,0.0);
    iposT -> positionMove(pos0.data());
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
    gazeStabRPC.write(cmdGS,respGS);
}

int torsoControllerThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

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
        goHome();

    printMessage(0,"Stopping gazeStabilizer..\n");
        gateStabilization("stop");
        gateStabilization("home");

    printMessage(0,"Closing controllers..\n");
        ddT.close();
}

// empty line to make gcc happy
