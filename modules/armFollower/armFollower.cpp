/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
\mainpage Head Following for the iCub Humanoid Robot
\defgroup icub_armFollower GazeStabilization
@ingroup icub_contrib_modules

A module for implementing the GAZE STABILIZER on the iCub.

Date first release: 07/07/2014

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the GAZE STABILIZER on the iCub.

\section lib_sec Libraries 
YARP and iCub

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default armFollower).

--robot      \e rob
- The name of the robot (either "icub" or "icub"). Default "icub".
  If you are guessing: Yes, the test HAS to be performed on the real robot!

--rate       \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 1). The higher is the verbosity, the more
  information is printed out.

\section portsc_sec Ports Created


\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
 
\section tested_os_sec Tested OS
Linux (Ubuntu 14.04, Debian Wheezy).

\author Alessandro Roncone
*/ 

#include <stdio.h>
#include <string>
#include <cctype>
#include <algorithm>
#include <map>
#include <math.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#define PPS_AVOIDANCE_RADIUS        0.2     // [m]
#define PPS_AVOIDANCE_VELOCITY      0.10    // [m/s]
#define TIMEOUT       1.0     // [s]
#define THRESHOLD     0.001

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

//*************************************
class armFollower: public RFModule,
                   public PortReader
{
protected:
    PolyDriver driverCart;
    PolyDriver driverJoint;
    int context;
    string part;
    Port dataPort;

    bool isRunning;

    RpcServer          rpcSrvr;

    Mutex mutex;
    struct Data {
        ICartesianControl *iarm;
        Vector accelerometer, gyroscope, orientation;
        Vector home_x, home_o;
        double timeout;
        string type;
        Data(): accelerometer(3,0.0), gyroscope(3,0.0), orientation(3,0.0),
                home_x(3,0.0), home_o(4,0.0),
                timeout(-1.0), type("translation") { }
    };
    Data data;
    
    //********************************************
    bool read(ConnectionReader &connection)
    {
        Bottle input;
        input.read(connection);
        if (input.size()>=10)
        {
            LockGuard lg(mutex);

            data.accelerometer[0] = input.get(0).asDouble();
            data.accelerometer[1] = input.get(1).asDouble();
            data.accelerometer[2] = input.get(2).asDouble();
            data.gyroscope[0]     = input.get(3).asDouble();
            data.gyroscope[1]     = input.get(4).asDouble();
            data.gyroscope[2]     = input.get(5).asDouble();
            data.orientation[3]   = input.get(6).asDouble();
            data.orientation[4]   = input.get(7).asDouble();
            data.orientation[5]   = input.get(8).asDouble();
            data.type             = input.get(9).asString();

            if (fabs(data.gyroscope[0])<THRESHOLD)
            {
                data.gyroscope[0]=0.0;
            }
            // else
            //     data.gyroscope[0]=(data.gyroscope[0]>0.0?1:-1)*0.2;
            
            if (fabs(data.gyroscope[1])<THRESHOLD)
            {
                data.gyroscope[1]=0.0;
            }
            // else 
            //     data.gyroscope[1]=(data.gyroscope[1]>0.0?1:-1)*0.2;
            
            if (fabs(data.gyroscope[2])<THRESHOLD)
            {
                data.gyroscope[2]=0.0;
            }
            // else
            //     data.gyroscope[2]=(data.gyroscope[2]>0.0?1:-1)*0.2;
            

            yInfo("Receiving data. acc: %s gyro: %s type: %s",
                    data.accelerometer.toString(3,3).c_str(),
                    data.gyroscope.toString(3,3).c_str(), data.type.c_str());
        }
        return true;
    }

    //********************************************
    void manageArm(Data &data)
    {
        if (isRunning)
        {
            Vector x,o;
            data.iarm->getPose(x,o);
            // if (norm(x-data.home_x)>PPS_AVOIDANCE_RADIUS)
            //     data.iarm->stopControl();
            // else 
            if (norm(data.gyroscope)!=0)
            {
                data.iarm->setTaskVelocities(0.125*data.gyroscope,Vector(4,0.0));
                data.timeout=TIMEOUT; 
            }
            else
            {
                data.iarm->setTaskVelocities(Vector(3,0.0),Vector(4,0.0)); 
            }
        }
        else if (data.timeout>0.0)
        {
            data.timeout=std::max(data.timeout-getPeriod(),0.0);
        }
        else if (data.timeout==0.0)
        {
            data.iarm->goToPose(data.home_x,data.home_o);
            data.timeout=-1.0;
        }
    }
    
public:
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('s','t','a','r'):
                {
                    int res=Vocab::encode("started");
                    if (startFollowing())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('s','t','o','p'):
                {
                    int res=Vocab::encode("stopped");
                    if (stopFollowing())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                case VOCAB4('h','o','m','e'):
                {
                    int res=Vocab::encode("home");
                    if (stopFollowing()&&goHome())
                    {
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                    
                    reply.addVocab(res);
                    return true;
                }
                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    //********************************************
    bool configure(ResourceFinder &rf)
    {
        string  name=rf.check("name",Value("armFollower")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
                part=rf.check("part",Value("right_arm")).asString().c_str();

        data=Data();

        if (part=="left_arm" || part=="left")
        {
            part="left_arm";

            data.home_x[0]=-0.30;
            data.home_x[1]=-0.20;
            data.home_x[2]=+0.05;
            Matrix R(4,4);
            R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0; R(3,3)=1.0;
            data.home_o=dcm2axis(R);
        }
        else if (part=="right_arm" || part=="right")
        {
            part="right_arm";

            data.home_x[0]=-0.30;
            data.home_x[1]=+0.20;
            data.home_x[2]=+0.05;
            Matrix R(4,4);
            R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0; R(3,3)=1.0;
            data.home_o=dcm2axis(R);
        }

        bool autoConnect=rf.check("autoConnect");
        if (autoConnect)
        {
            yWarning("AutoConnect mode set to ON");
        }

        isRunning=rf.check("autoStart");
        if (isRunning)
        {
            yWarning("AutoStart mode set to ON");
        }

        bool stiff=rf.check("stiff");
        if (stiff)
        {
            yInfo("Stiff Mode enabled.");
        }

        Property optionCart;
        optionCart.put("device","cartesiancontrollerclient");
        optionCart.put("remote","/"+robot+"/cartesianController/"+part);
        optionCart.put("local",("/"+name+"/cart/"+part).c_str());
        if (!driverCart.open(optionCart))
        {
            close();
            return false;
        }

        Property optionJoint;
        optionJoint.put("device","remote_controlboard");
        optionJoint.put("remote","/"+robot+"/"+part);
        optionJoint.put("local",("/"+name+"/joint/"+part).c_str());
        if (!driverJoint.open(optionJoint))
        {
            close();
            return false;
        }

        driverCart.view(data.iarm);

        data.iarm->storeContext(&context);

        Vector dof;
        data.iarm->getDOF(dof);
        dof=0.0; dof[3]=dof[4]=dof[5]=dof[6]=dof[7]=dof[8]=dof[9]=1.0;
        data.iarm->setDOF(dof,dof);
        data.iarm->setTrajTime(0.9);

        goHome();

        IInteractionMode  *imode; driverJoint.view(imode);
        IImpedanceControl *iimp;  driverJoint.view(iimp);

        if (!stiff)
        {
            imode->setInteractionMode(0,VOCAB_IM_COMPLIANT); iimp->setImpedance(0,0.4,0.03); 
            imode->setInteractionMode(1,VOCAB_IM_COMPLIANT); iimp->setImpedance(1,0.4,0.03);
            imode->setInteractionMode(2,VOCAB_IM_COMPLIANT); iimp->setImpedance(2,0.4,0.03);
            imode->setInteractionMode(3,VOCAB_IM_COMPLIANT); iimp->setImpedance(3,0.2,0.01);
            imode->setInteractionMode(4,VOCAB_IM_COMPLIANT); iimp->setImpedance(4,0.2,0.0);
        }

        dataPort.open(("/"+name+"/mobilesensor:i").c_str());
        dataPort.setReader(*this);

        rpcSrvr.open(("/"+name+"/rpc:i").c_str());
        attach(rpcSrvr);

        if (autoConnect)
        {
            Network::connect("/yarpdroid/mobilesensor:o",("/"+name+"/mobilesensor:i").c_str());
            Network::connect("/yarpdroid/rpc:o",("/"+name+"/rpc:i").c_str());
        }
        return true; 
    }

    bool goHome()
    {
        yDebug("Going home..");
        data.iarm->goToPoseSync(data.home_x,data.home_o);
        data.iarm->waitMotionDone();
        return true;
    }

    bool startFollowing()
    {
        isRunning = true;
        return true;
    }

    bool stopFollowing()
    {
        isRunning = false;
        return true;
    }

    //********************************************
    bool updateModule()
    {
        LockGuard lg(mutex);
        manageArm(data);
        return true;
    }

    //********************************************
    double getPeriod()
    {
        return 0.05;
    }

    //********************************************
    bool close()
    {
        dataPort.close();

        if (driverCart.isValid())
        {
            data.iarm->stopControl();
            data.iarm->restoreContext(context);
            driverCart.close(); 
        }

        if (driverJoint.isValid())
        {
            IInteractionMode *imode;
            driverJoint.view(imode);
            for (int j=0; j<5; j++)
                imode->setInteractionMode(j,VOCAB_IM_STIFF);

            driverJoint.close();
        }

        return true;
    }
};


//********************************************
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("gazeStabilization");
    rf.setDefaultConfigFile("armFollower.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context [gazeStabilization]:  where to find the called resource.");
        yInfo("   --from      [armFollower.ini]:  the name of the .ini file.");
        yInfo("   --name          [armFollower]:  the name of the module.");
        yInfo("   --robot                [icub]:  the name of the robot.");
        yInfo("   --rate                   [10]:  the period used by the thread [ms].");
        yInfo("   --verbosity               [0]:  verbosity level.");
        yInfo("   --stiff                [flag]:  Either to be stiff or not.");
        
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!\n");
        return -1;
    }

    armFollower gzStab;
    return gzStab.runModule(rf);
}


