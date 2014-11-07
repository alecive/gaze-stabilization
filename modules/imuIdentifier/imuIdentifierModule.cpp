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
\defgroup icub_imuIdentifier imuIdentifier
@ingroup icub_gazeStabilizer

A module for implementing the IMU IDENTIFIER on the iCub.

Date first release: 07/07/2014

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the IMU IDENTIFIER on the iCub.
It uses a set of waypoints in order to move the iCub head to a specified position with a specified velocity.
In the meanwhile, it reads both references at the PID level, and the gyro output from the inertial, sending them out 
through the IMU:o port for later log and use.

\section lib_sec Libraries 
YARP and OpenCV

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default imuIdentifier).

--robot      \e rob
- The name of the robot (either "icub" or "icub"). Default "icub".
  If you are guessing: Yes, the test HAS to be performed on the real robot!

--rate       \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 1). The higher is the verbosity, the more
  information is printed out.

\section portsc_sec Ports Created
- <i> /<name>/inertial:i </i> it reads values from the inertial sensor

- <i> /<name>/IMU:o </i> it prints out useful data in order to track both the pid controllers and the inertial sensor

\section in_files_sec Input Data Files
- <i> <name>.ini </i> it is a (mandatory) file from which the module can retrieve the waypoints used in the experiment.

\section out_data_sec Output Data Files
 
\section tested_os_sec Tested OS
Linux (Ubuntu 14.04, Debian Wheezy).

\author Alessandro Roncone
*/ 

#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string.h> 
#include <ctime>
#include <sstream>

#include "imuIdentifierThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup imuIdentifierModule
*
* The module that achieves the imuIdentifier task.
*  
*/
class imuIdentifier: public RFModule
{
    private:
        imuIdentifierThread *imuIdentifierThrd;
        RpcServer            rpcSrvr;

    public:
        imuIdentifier()
        {
            imuIdentifierThrd=0;
        }

        bool respond(const Bottle &command, Bottle &reply)
        {
            int ack =Vocab::encode("ack");
            int nack=Vocab::encode("nack");

            if (command.size()>0)
            {
                switch (command.get(0).asVocab())
                {
                    case VOCAB4('r','e','d','o'):
                    {
                        int res=Vocab::encode("started");
                        if (imuIdentifierThrd -> redoCycle())
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

        bool configure(ResourceFinder &rf)
        {
            string name         = "imuIdentifier";
            string robot        = "icub";
            int    verbosity    =      0;    // verbosity
            int    rate         =     10;    // rate of the imuIdentifierThread
            int    numWaypoints =      1;

            //******************* NAME ******************
                if (rf.check("name"))
                {
                    name = rf.find("name").asString();
                    printf("*** Module name set to %s\n",name.c_str());  
                }
                else printf("*** Module name set to default, i.e. %s\n",name.c_str());
                setName(name.c_str());

            //****************** rate ******************
                if (rf.check("rate"))
                {
                    rate = rf.find("rate").asInt();
                    printf(("*** "+name+": thread working at %i ms\n").c_str(), rate);
                }
                else printf(("*** "+name+": could not find rate in the config file; using %i ms as default\n").c_str(), rate);

            //******************* VERBOSE ******************
                if (rf.check("verbosity"))
                {
                    verbosity = rf.find("verbosity").asInt();
                    printf(("*** "+name+": verbosity set to %i\n").c_str(),verbosity);
                }
                else printf(("*** "+name+": could not find verbosity option in the config file; using %i as default\n").c_str(),verbosity);

            //******************* ROBOT ******************
                if (rf.check("robot"))
                {
                    robot = rf.find("robot").asString();
                    printf(("*** "+name+": robot is %s\n").c_str(),robot.c_str());
                }
                else printf(("*** "+name+": could not find robot option in the config file; using %s as default\n").c_str(),robot.c_str());

            //******************* NUMWAYPOINTS ******************
                if (rf.check("numWaypoints"))
                {
                    numWaypoints = rf.find("numWaypoints").asInt();
                    printf(("*** "+name+": numWaypoints set to %i\n").c_str(),numWaypoints);
                }
                else printf(("*** "+name+": could not find numWaypoints option in the config file; using %i as default\n").c_str(),numWaypoints);

            //****************** THREAD ******************
                imuIdentifierThrd = new imuIdentifierThread(rate, name, robot, verbosity, numWaypoints, rf);

                imuIdentifierThrd -> start();
                bool strt = 1;
                if (!strt)
                {
                    delete imuIdentifierThrd;
                    imuIdentifierThrd = 0;
                    cout << "\nERROR!!! imuIdentifierThread wasn't instantiated!!\n";
                    return false;
                }
                cout << "imu IDENTIFIER: imuIdentifierThread istantiated...\n";

            //************************ RPC ***********************
                rpcSrvr.open(("/"+name+"/rpc:i").c_str());
                attach(rpcSrvr);

            return true;
        }

        bool close()
        {
            cout << "imu IDENTIFIER: Stopping threads.." << endl;
            if (imuIdentifierThrd)
            {
                imuIdentifierThrd->stop();
                delete imuIdentifierThrd;
                imuIdentifierThrd=0;
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
    rf.setDefaultConfigFile("imuIdentifier.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        cout << endl << "IMU IDENTIFIER module" << endl;
        cout << endl << "Options:" << endl;
        cout << "   --context      path:  where to find the called resource. Default gazeStabilization." << endl;
        cout << "   --from         from:  the name of the .ini file. Default imuIdentifier.ini." << endl;
        cout << "   --name         name:  the name of the module. Default imuIdentifier." << endl;
        cout << "   --robot        robot: the name of the robot. Default icub." << endl;
        cout << "   --rate         rate:  the period used by the thread. Default 10ms." << endl;
        cout << "   --verbosity    int:   verbosity level. Default 0." << endl;
        cout << "   --iterations   int:   number of times the identifier iterates over the waypoints. Default 1." << endl;
        cout << endl;
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    imuIdentifier imuIdntfr;
    return imuIdntfr.runModule(rf);
}
// empty line to make gcc happy
