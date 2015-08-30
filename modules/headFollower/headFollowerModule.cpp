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
\defgroup icub_headFollower GazeStabilization
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
- The name of the module (default headFollower).

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

#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string.h> 
#include <ctime>
#include <sstream>

#include "headFollowerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup headFollowerModule
*
* The module that achieves the headFollower task.
*  
*/
class headFollower: public RFModule
{
    private:
        headFollowerThread *headFollowerThrd;
        RpcServer           rpcSrvr;
        string              name;

    public:
        headFollower()
        {
            headFollowerThrd=0;
        }

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
                        if (headFollowerThrd -> startFollowing())
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
                        if (headFollowerThrd -> stopFollowing())
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
                        bool homing = true;
                        homing = homing && headFollowerThrd -> stopFollowing();
                        homing = homing && headFollowerThrd -> goHome();
                        if (homing)
                        {
                            reply.addVocab(ack);
                        }
                        else
                            reply.addVocab(nack);
                        
                        reply.addVocab(res);
                        return true;
                    }
                    case VOCAB3('g','e','t'):
                    {
                        reply.addString(command.get(1).asString());

                        if (command.get(1).asString() == "if_mode")
                        {
                            reply.addVocab(ack);
                            reply.addString(headFollowerThrd -> get_if_mode());
                        }
                        else
                            reply.addVocab(nack);

                        return true;
                    }
                    case VOCAB3('s','e','t'):
                    {
                        reply.addString(command.get(1).asString());

                        if (command.get(1).asString() == "if_mode")
                        {
                            if (headFollowerThrd -> set_if_mode(command.get(2).asString()))
                            {
                                reply.addVocab(ack);
                            }
                            else
                                reply.addVocab(nack);
                        }
                        else
                            reply.addVocab(nack);

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

        bool configure(ResourceFinder &rf)
        {
            name             = "headFollower";
            string robot     =         "icub";
            int    verbosity =              0; // verbosity
            int    rate      =             10; // rate of the headFollowerThread
            string if_mode   =         "vel2"; // it can be either vel1 or vel2
            bool   autoStart =          false;

            //******************* NAME ******************
                if (rf.check("name"))
                {
                    name = rf.find("name").asString();
                    yDebug(("["+name+"] Module name set to %s").c_str(),name.c_str());  
                }
                else yDebug(("["+name+"] Module name set to default, i.e. %s").c_str(),name.c_str());
                setName(name.c_str());

            //****************** rate ******************
                if (rf.check("rate"))
                {
                    rate = rf.find("rate").asInt();
                    yDebug(("["+name+"] thread working at %i ms").c_str(), rate);
                }
                else yDebug(("["+name+"] could not find rate in the conf file; using %i ms as default.").c_str(), rate);

            //******************* VERBOSE ******************
                if (rf.check("verbosity"))
                {
                    verbosity = rf.find("verbosity").asInt();
                    yDebug(("["+name+"] verbosity set to %i").c_str(),verbosity);
                }
                else yDebug(("["+name+"] could not find verbosity option in the conf file; using %i as default.").c_str(),verbosity);

            //******************* ROBOT ******************
                if (rf.check("robot"))
                {
                    robot = rf.find("robot").asString();
                    yDebug(("["+name+"] robot is %s").c_str(),robot.c_str());
                }
                else yDebug(("["+name+"] could not find robot option in the conf file; using %s as default.").c_str(),robot.c_str());

            //************** INTERFACE_MODE **************
                if (rf.check("if_mode"))
                {
                    if (rf.find("if_mode").asString() == "vel1" || rf.find("if_mode").asString() == "vel2")
                    {
                        if_mode = rf.find("if_mode").asString();
                        yDebug(("["+name+"] if_mode set to %s").c_str(),if_mode.c_str());
                    }
                    else yDebug(("["+name+"] if_mode option found but not allowed; using %s as default.").c_str(),if_mode.c_str());
                }
                else yDebug(("["+name+"] could not find if_mode option in the conf file; using %s as default.").c_str(),if_mode.c_str());

            //************** AUTOSTART **************
                autoStart = rf.check("autoStart");
                if (autoStart)
                {
                    yDebug(("["+name+"] module set to autoStart %s").c_str(),if_mode.c_str());
                }

            //****************** THREAD ******************
                headFollowerThrd = new headFollowerThread(rate, name, robot, verbosity, if_mode, autoStart);

                headFollowerThrd -> start();
                bool strt = 1;
                if (!strt)
                {
                    delete headFollowerThrd;
                    headFollowerThrd = 0;
                    yError("["+name+"] headFollowerThread wasn't instantiated!!");
                    return false;
                }
                yInfo("["+name+"] headFollowerThread istantiated...");

            //************************ RPC ***********************
                rpcSrvr.open(("/"+name+"/rpc:i").c_str());
                attach(rpcSrvr);

            return true;
        }

        bool close()
        {
            yInfo(("["+name+"] Stopping threads..").c_str());
            if (headFollowerThrd)
            {
                headFollowerThrd->stop();
                delete headFollowerThrd;
                headFollowerThrd=0;
            }

            return true;
        }

        double getPeriod()  { return 1.0; }
        bool updateModule() { return true; }
};

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("gazeStabilization");
    rf.setDefaultConfigFile("headFollower.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context [gazeStabilization]:  where to find the called resource.");
        yInfo("   --from   [headFollower.ini]:  the name of the .ini file.");
        yInfo("   --name       [headFollower]:  the name of the module.");
        yInfo("   --robot                [icub]:  the name of the robot.");
        yInfo("   --rate                   [10]:  the period used by the thread [ms].");
        yInfo("   --verbosity               [0]:  verbosity level.");
        yInfo("   --if_mode              [vel2]:  interface to use for velocity control. It can be either vel1 or vel2.");
        
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!\n");
        return -1;
    }

    headFollower gzStab;
    return gzStab.runModule(rf);
}
// empty line to make gcc happy
