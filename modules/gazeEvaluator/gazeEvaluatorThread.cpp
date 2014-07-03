#include "gazeEvaluatorThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define GYRO_BIAS_STABILITY                 5.0     // [deg/s]

gazeEvaluatorThread::gazeEvaluatorThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    imagePortIn  = new BufferedPort<ImageOf<PixelRgb> >;
    imagePortOut = new BufferedPort<ImageOf<PixelRgb> >;
}

bool gazeEvaluatorThread::threadInit()
{
    imagePortIn  -> open(("/"+name+"/img:i").c_str());
    imagePortOut -> open(("/"+name+"/img:o").c_str());

    return true;
}

void gazeEvaluatorThread::run()
{

}

int gazeEvaluatorThread::printMessage(const int l, const char *f, ...) const
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

void gazeEvaluatorThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void gazeEvaluatorThread::threadRelease()
{
    // printMessage(0,"Moving head to home position.. \n");
    //     Vector pos0(6,0.0);
    //     iposH -> positionMove(pos0.data());

    printMessage(0,"Closing ports...\n");
        inTorsoPort->close();
        inIMUPort->close();

    printMessage(0,"Closing controllers..\n");
        ddH->close();
        delete ddH;
        ddH = NULL;

        ddT->close();
        delete ddT;
        ddT = NULL;

        if (encsH)
        {
            delete encsH;
            encsH = NULL;
        }

        if (encsT)
        {
            delete encsT;
            encsT = NULL;
        }

        if (eyeR)
        {
            delete eyeR;
            eyeR = NULL;
        }

        if (eyeL)
        {
            delete eyeL;
            eyeL = NULL;
        }

}

// empty line to make gcc happy
