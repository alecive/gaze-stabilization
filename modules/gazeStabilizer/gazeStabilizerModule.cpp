/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
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
\mainpage Gaze Stabilization for the iCub Humanoid Robot
\defgroup icub_gazeStabilizer GazeStabilization
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
- The name of the module (default gazeStabilizer).

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

#include "gazeStabilizerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup gazeStabilizerModule
*
* The module that achieves the gazeStabilizer task.
*  
*/
class gazeStabilizer: public RFModule
{
    private:
        gazeStabilizerThread *gazeStabilizerThrd;
        RpcServer             rpcSrvr;

    public:
        gazeStabilizer()
        {
            gazeStabilizerThrd=0;
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
                        if (gazeStabilizerThrd -> startStabilization())
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
                        if (gazeStabilizerThrd -> stopStabilization())
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
                        if (gazeStabilizerThrd -> goHome())
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
                            reply.addString(gazeStabilizerThrd -> get_if_mode());
                        }
                        else if (command.get(1).asString() == "src_mode")
                        {
                            reply.addVocab(ack);
                            reply.addString(gazeStabilizerThrd -> get_src_mode());
                        }
                        else if (command.get(1).asString() == "ctrl_mode")
                        {
                            reply.addVocab(ack);
                            reply.addString(gazeStabilizerThrd -> get_ctrl_mode());
                        }
                        else if (command.get(1).asString() == "calib_IMU")
                        {
                            reply.addVocab(ack);
                            reply.addInt(gazeStabilizerThrd -> get_calib_IMU());
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
                            if (gazeStabilizerThrd -> set_if_mode(command.get(2).asString()))
                            {
                                reply.addVocab(ack);
                            }
                            else
                                reply.addVocab(nack);
                        }
                        else if (command.get(1).asString() == "src_mode")
                        {
                            if (gazeStabilizerThrd -> set_src_mode(command.get(2).asString()))
                            {
                                if ((command.get(2).asString()) == "inertial" &&
                                    gazeStabilizerThrd -> get_calib_IMU())
                                {
                                    gazeStabilizerThrd -> calibrateIMU();
                                }
                                reply.addVocab(ack);
                            }
                            else
                                reply.addVocab(nack);
                        }
                        else if (command.get(1).asString() == "ctrl_mode")
                        {
                            if (gazeStabilizerThrd -> set_ctrl_mode(command.get(2).asString()))
                            {
                                reply.addVocab(ack);
                            }
                            else
                                reply.addVocab(nack);
                        }
                        else if (command.get(1).asString() == "calib_IMU")
                        {
                            if (gazeStabilizerThrd -> set_calib_IMU(bool(command.get(2).asInt())))
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
                    case VOCAB4('c','a','l','i'):
                    {
                        if (gazeStabilizerThrd -> get_calib_IMU())
                        {
                            gazeStabilizerThrd -> calibrateIMU();
                            reply.addVocab(ack);
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
            string name      = "gazeStabilizer";
            string robot     =           "icub";
            int    verbosity =                0; // verbosity
            int    rate      =               10; // rate of the gazeStabilizerThread
            string if_mode   =           "vel2"; // it can be either vel1 or vel2
            string src_mode  =       "inertial"; // it can be either torso or inertial or wholeBody
            string ctrl_mode =           "eyes"; // it can be either head, eyes or headEyes
            bool   calib_IMU =             true;
            double int_gain  =              2.0; // the gein sent to the integrator

            //******************* NAME ******************
                if (rf.check("name"))
                {
                    name = rf.find("name").asString();
                    yDebug("*** Module name set to %s",name.c_str());  
                }
                else yDebug("*** Module name set to default, i.e. %s",name.c_str());
                setName(name.c_str());

            //****************** rate ******************
                if (rf.check("rate"))
                {
                    rate = rf.find("rate").asInt();
                    yDebug(("*** "+name+": thread working at %i ms").c_str(), rate);
                }
                else yDebug(("*** "+name+": could not find rate in the conf file; using %i ms as default.").c_str(), rate);

            //******************* VERBOSE ******************
                if (rf.check("verbosity"))
                {
                    verbosity = rf.find("verbosity").asInt();
                    yDebug(("*** "+name+": verbosity set to %i").c_str(),verbosity);
                }
                else yDebug(("*** "+name+": could not find verbosity option in the conf file; using %i as default.").c_str(),verbosity);

            //******************* ROBOT ******************
                if (rf.check("robot"))
                {
                    robot = rf.find("robot").asString();
                    yDebug(("*** "+name+": robot is %s").c_str(),robot.c_str());
                }
                else yDebug(("*** "+name+": could not find robot option in the conf file; using %s as default.").c_str(),robot.c_str());

            //************** INTERFACE_MODE **************
                if (rf.check("if_mode"))
                {
                    if (rf.find("if_mode").asString() == "vel1" || rf.find("if_mode").asString() == "vel2")
                    {
                        if_mode = rf.find("if_mode").asString();
                        yDebug(("*** "+name+": if_mode set to %s").c_str(),if_mode.c_str());
                    }
                    else yDebug(("*** "+name+": if_mode option found but not allowed; using %s as default.").c_str(),if_mode.c_str());
                }
                else yDebug(("*** "+name+": could not find if_mode option in the conf file; using %s as default.").c_str(),if_mode.c_str());

            //************** SOURCE_MODE **************
                if (rf.check("src_mode"))
                {
                    if (rf.find("src_mode").asString() == "torso" || rf.find("src_mode").asString() == "inertial" || rf.find("src_mode").asString() == "wholeBody")
                    {
                        src_mode = rf.find("src_mode").asString();
                        yDebug(("*** "+name+": src_mode set to %s").c_str(),src_mode.c_str());
                    }
                    else yDebug(("*** "+name+": src_mode option found but not allowed; using %s as default.").c_str(),src_mode.c_str());
                }
                else yDebug(("*** "+name+": could not find src_mode option in the conf file; using %s as default.").c_str(),src_mode.c_str());

            //************** CONTROL_MODE **************
                if (rf.check("ctrl_mode"))
                {
                    if (rf.find("ctrl_mode").asString() == "head" ||
                        rf.find("ctrl_mode").asString() == "eyes" ||
                        rf.find("ctrl_mode").asString() == "headEyes")
                    {
                        ctrl_mode = rf.find("ctrl_mode").asString();
                        yDebug(("*** "+name+": ctrl_mode set to %s").c_str(),ctrl_mode.c_str());
                    }
                    else yDebug(("*** "+name+": ctrl_mode option found but not allowed; using %s as default.").c_str(),ctrl_mode.c_str());
                }
                else yDebug(("*** "+name+": could not find ctrl_mode option in the conf file; using %s as default.").c_str(),ctrl_mode.c_str());

            //**************** CALIB_IMU *****************
                if (rf.check("calib_IMU"))
                {
                    calib_IMU=true;
                    yDebug(("*** "+name+": calib_IMU option has been set to %s.").c_str(),calib_IMU?"true":"false");
                }
                else
                {
                    yDebug(("*** "+name+": could not find calib_IMU option in the conf file; using %s as default.").c_str(),calib_IMU?"true" :"false");
                }


            //****************** THREAD ******************
                gazeStabilizerThrd = new gazeStabilizerThread(rate, name, robot, verbosity, if_mode, src_mode, ctrl_mode, calib_IMU);

                gazeStabilizerThrd -> start();
                bool strt = 1;
                if (!strt)
                {
                    delete gazeStabilizerThrd;
                    gazeStabilizerThrd = 0;
                    yError(" gazeStabilizerThread wasn't instantiated!!");
                    return false;
                }
                yInfo("GAZE STABILIZER: gazeStabilizerThread istantiated...");

            //************************ RPC ***********************
                rpcSrvr.open(("/"+name+"/rpc:i").c_str());
                attach(rpcSrvr);

            return true;
        }

        bool close()
        {
            yInfo("GAZE STABILIZER: Stopping threads..");
            if (gazeStabilizerThrd)
            {
                gazeStabilizerThrd->stop();
                delete gazeStabilizerThrd;
                gazeStabilizerThrd=0;
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
    rf.setDefaultConfigFile("gazeStabilizer.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        yInfo("");
        yInfo("Options:");
        yInfo("");
        yInfo("   --context    path:   where to find the called resource. Default gazeStabilization.");
        yInfo("   --from       from:   the name of the .ini file. Default gazeStabilizer.ini");
        yInfo("   --name       name:   the name of the module. Default gazeStabilizer.");
        yInfo("   --robot      robot:  the name of the robot. Default icub.");
        yInfo("   --rate       rate:   the period used by the thread. Default 10ms.");
        yInfo("   --verbosity  int:    verbosity level. Default 0.");
        yInfo("   --if_mode    mode:   interface to use for velocity control. It can be either vel1 or vel2, default vel2.");
        yInfo("   --src_mode   source: source to use for compensating. It can be either torso, inertial, or wholeBody; default torso.");
        yInfo("   --ctrl_mode  ctrl:   control to use for deploying the compensation.");
        yInfo("                        It can be either head, eyes or headEyes; default headEyes.");
        yInfo("   --calib_IMU          flag to know if to calibrate the IMU measurements in advance or not (recommended).");
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        yError("No Network!!!\n");
        return -1;
    }

    gazeStabilizer gzStab;
    return gzStab.runModule(rf);
}
// empty line to make gcc happy
