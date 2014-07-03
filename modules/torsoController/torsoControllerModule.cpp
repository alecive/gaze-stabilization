/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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
\defgroup icub_torsoController torsoController
@ingroup icub_contrib_modules

A module for implementing the TORSO CONTROLLER on the iCub.
It tests the positions of the chessboard's corners on both the image planes.

Date first release: 13/11/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the TORSO CONTROLLER on the iCub.
It tests the positions of the chessboard's corners on both the image planes.

\section lib_sec Libraries 
YARP and OpenCV

\section parameters_sec Parameters

--context    \e path
- Where to find the called resource.

--from       \e from
- The name of the .ini file with the configuration parameters.

--name       \e name
- The name of the module (default torsoController).

--robot      \e rob
- The name of the robot (either "icub" or "icub"). Default "icub".
  If you are guessing: Yes, the test HAS to be performed on the real robot!

--rate       \e rate
- The period used by the thread. Default 100ms.

--verbosity  \e verb
- Verbosity level (default 1). The higher is the verbosity, the more
  information is printed out.

--type		 \e type
- The type of task (default '0').
  It's deprecated, but kept for retrocompatibility and for eventual future use.

\section portsc_sec Ports Created
- <i> /<name>/imgL:i </i> port that receives input from the left  camera
- <i> /<name>/imgR:i </i> port that receives input from the right camera
- <i> /<name>/imgL:o </i> port that sends the output of the left  camera
- <i> /<name>/imgR:o </i> port that sends the output of the right camera
- <i> /<name>/disp:i </i> port that receives the stereoDisparity
- <i> /<name>/disp:o </i> port that sends the stereoDisparity output

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
- <i> left_START.png </i>  left  image acquired at the beginning of the test 
- <i> right_START.png </i> right image acquired at the beginning of the test
- <i> results.log </i>     log file. It stores the shift (on the x and y axis)
		in both the left and right images for any of the 48 corners detected, the
		configuration file used, some statistics on the errors (min, max, avg), and
		the values of the encoders at the beginning and the end of the test (they 
		should usually be equal).
- <i> left_END.png </i>    left  image acquired at the end of the test
- <i> right_END.png </i>   right image acquired at the end of the test
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Debian Squeeze).

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

#include "torsoControllerThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup torsoControllerModule
*
* The module that achieves the torsoController task.
*  
*/
class torsoController: public RFModule
{
    private:
        torsoControllerThread *torsoControllerThrd;
        RpcServer             rpcSrvr;

    public:
        torsoController()
        {
            torsoControllerThrd=0;
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
                        if (torsoControllerThrd -> redoCycle())
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
            string name      = "torsoController";
            string robot     = "icub";
            int    verbosity =      0;    // verbosity
            int    rate      =     10;    // rate of the torsoControllerThread

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

            //****************** THREAD ******************
                torsoControllerThrd = new torsoControllerThread(rate, name, robot, verbosity);

                torsoControllerThrd -> start();
                bool strt = 1;
                if (!strt)
                {
                    delete torsoControllerThrd;
                    torsoControllerThrd = 0;
                    cout << "\nERROR!!! torsoControllerThread wasn't instantiated!!\n";
                    return false;
                }
                cout << "TORSO CONTROLLER: torsoControllerThread istantiated...\n";

            return true;
        }

        bool close()
        {
            cout << "TORSO CONTROLLER: Stopping threads.." << endl;
            if (torsoControllerThrd)
            {
                torsoControllerThrd->stop();
                delete torsoControllerThrd;
                torsoControllerThrd=0;
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
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("torsoController");
    rf.setDefaultConfigFile("torsoController.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context    path:  where to find the called resource" << endl;
        cout << "   --from       from:  the name of the .ini file." << endl;
        cout << "   --name       name:  the name of the module (default torsoController)." << endl;
        cout << "   --robot      robot: the name of the robot. Default icub." << endl;
        cout << "   --rate       rate:  the period used by the thread. Default 100ms." << endl;
        cout << "   --verbosity  int:   verbosity level (default 1)." << endl;
        cout << endl;
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    torsoController trsCtrl;
    return trsCtrl.runModule(rf);
}
// empty line to make gcc happy