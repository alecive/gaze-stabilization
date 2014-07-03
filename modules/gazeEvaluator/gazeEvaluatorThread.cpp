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
    imagePortOutFlow = new BufferedPort<ImageOf<PixelBgr> >;
    imagePortOutNorm = new BufferedPort<ImageOf<PixelBgr> >;

    imageIn=new ImageOf<PixelRgb>;

    isStarting = 1;
}

bool gazeEvaluatorThread::threadInit()
{
    imagePortIn  -> open(("/"+name+"/img:i").c_str());
    imagePortOutFlow -> open(("/"+name+"/optFlow:o").c_str());
    imagePortOutNorm -> open(("/"+name+"/optFModule:o").c_str());

    return true;
}

void gazeEvaluatorThread::run()
{
    if (isStarting)
    {
        ImageOf<PixelRgb> *tmp = imagePortIn->read(false);

        if(tmp!=NULL)
        {
            *imageIn   = *tmp;
            imgInNext  = (IplImage*) imageIn->getIplImage();
            imgInPrev  = (IplImage*)cvClone(imgInNext);
            isStarting = false;
            printMessage(0,"Starting..\n");
        }
    }
    else
    {
        // 1  - Get the image
        ImageOf<PixelRgb> *tmp = imagePortIn->read(false);
        if(tmp!=NULL)
        {
            *imageIn=*tmp;
        }
        // 2A - Put the read image into imgInNext
        imgInNext = (IplImage*) imageIn->getIplImage();

        // 2B - Smooth it out
        cvSmooth(imgInNext, imgInNext, CV_GAUSSIAN, 3, 0, 0, 0);

        // 3  - Set the new image
        Mat tmpPrev(imgInPrev);
        Mat tmpNext(imgInNext);
        setImages(tmpPrev,tmpNext);

        // 4  - Make it gray
        Mat imgPrevGray;
        Mat imgNextGray;
        cvtColor(imgPrev,imgPrevGray,CV_RGB2GRAY,1);
        cvtColor(imgNext,imgNextGray,CV_RGB2GRAY,1);

        // 5 - Compute the optical flow
        int flag=0;
        if(!optFlow.empty())
            optFlow.setTo(Scalar(0));
        
        calcOpticalFlowFarneback(imgPrevGray,imgNextGray,optFlow,0.25,5,9,5,7,1.5,flag);

        imgInPrev=(IplImage*)cvClone(imgInNext);

        if (!optFlow.empty() && imgInPrev!=NULL)
        {
            sendOptFlow();
        }
    }
}

IplImage* gazeEvaluatorThread::draw2DMotionField()
{
    int xSpace=7;
    int ySpace=7;
    float cutoff=1;
    float multiplier=5;
    CvScalar color=CV_RGB(255,0,0);

    CvPoint p0 = cvPoint(0,0);
    CvPoint p1 = cvPoint(0,0);

    IplImage* imgMotion=(IplImage*) cvClone(imgInPrev);

    float deltaX, deltaY, angle, hyp;

    for(int i=0; i<optFlow.rows; i+=5) 
    {
        for (int j=0; j<optFlow.cols; j+=5)
        {
            p0.x = j;
            p0.y = i;
            deltaX = optFlow.ptr<float>(i)[2*j];
            deltaY = -optFlow.ptr<float>(i)[2*j+1];
            angle = atan2(deltaY, deltaX);
            hyp = sqrt(deltaX*deltaX + deltaY*deltaY);

            if(hyp > cutoff)
            {
                p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
                p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
                p0.x = p1.x + cvRound(3*cos(angle-CV_PI + CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI + CV_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);

                p0.x = p1.x + cvRound(3*cos(angle-CV_PI - CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI - CV_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
            }
        }
    }

    return imgMotion;
}

void gazeEvaluatorThread::drawFlowModule(IplImage* imgMotion)
{
    IplImage * module =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),32,1);
    IplImage * moduleU =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),8,1);
    Mat vel[2];
    split(optFlow,vel);
    IplImage tx=(Mat)vel[0];
    IplImage ty=(Mat)vel[1];

    IplImage* velxpow=cvCloneImage(&tx);
    IplImage* velypow=cvCloneImage(&ty);

    cvPow(&tx, velxpow, 2);
    cvPow(&ty, velypow, 2);

    cvAdd(velxpow, velypow, module, NULL);
    cvPow(module, module, 0.5);
    cvNormalize(module, module, 0.0, 1.0, CV_MINMAX, NULL);
    cvZero(imgMotion);
    cvConvertScale(module,moduleU,255,0);
    cvMerge(moduleU,moduleU,moduleU,NULL,imgMotion);
    cvReleaseImage(&module);
    cvReleaseImage(&moduleU);
}

void gazeEvaluatorThread::sendOptFlow()
{
    IplImage* imgOptFlow = (IplImage*)draw2DMotionField();

    if(imgOptFlow!=NULL)
    {
        printMessage(0,"I've got an optical flow!\n");

        ImageOf<PixelBgr>& outim = imagePortOutFlow->prepare();

        printMessage(0,"imgOptFlow depth %i\n",imgOptFlow->depth);
        outim.wrapIplImage(imgOptFlow);

        imagePortOutFlow->write();
    }

    cvReleaseImage(&imgOptFlow);

    IplImage* imgOptFlowModule=cvCreateImage(cvSize(imgInPrev->width,imgInPrev->height),8,3);
    drawFlowModule(imgOptFlowModule);

    if(imgOptFlow!=NULL)
    {
        printMessage(0,"I've got an optical flow!\n");

        ImageOf<PixelBgr>& outim = imagePortOutNorm->prepare();

        printMessage(0,"imgOptFlow depth %i\n",imgOptFlowModule->depth);
        outim.wrapIplImage(imgOptFlowModule);

        imagePortOutNorm->write();
    }

    cvReleaseImage(&imgOptFlowModule);
}

void gazeEvaluatorThread::setImages(Mat &_prev, Mat &_next) 
{
    imgPrev=_prev;
    imgNext=_next;
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
    printMessage(0,"Closing ports...\n");
        imagePortIn  -> close();
        imagePortOutFlow -> close();
        imagePortOutNorm -> close();
}

// empty line to make gcc happy
