#include "gazeStabilizerThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

gazeStabilizerThread::gazeStabilizerThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name),
                                           robot(_robot), verbosity(_v)
{

}

bool gazeStabilizerThread::threadInit()
{
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

    // open the view
    if (ddT.isValid())
    {
        ok = ok && ddT.view(iencsT);
        ok = ok && ddT.view(iposT);
        ok = ok && ddT.view(ilimT);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsT->getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

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

    // open the view
    if (ddH.isValid())
    {
        ok = ok && ddH.view(iencsH);
        ok = ok && ddH.view(iposH);
        ok = ok && ddH.view(ilimH);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsH->getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);

    return true;
}

void gazeStabilizerThread::run()
{

}

int gazeStabilizerThread::printMessage(const int l, const char *f, ...) const
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

void gazeStabilizerThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void gazeStabilizerThread::threadRelease()
{

}

// empty line to make gcc happy
