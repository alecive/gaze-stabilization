#include "gazeEvaluatorThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define GYRO_BIAS_STABILITY                 5.0     // [deg/s]
#define PIXELS_TO_DISCARD                  50.0

gazeEvaluatorThread::gazeEvaluatorThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    imgPortIn = new BufferedPort<ImageOf<PixelRgb> >;
    imageIn   = new ImageOf<PixelRgb>;

    isStarting = 1;
}

bool gazeEvaluatorThread::threadInit()
{
    imgPortIn  -> open(("/"+name+"/img:i").c_str());
    imgOutportFlow.open(("/"+name+"/optFlow:o").c_str());
    imgOutportModule.open(("/"+name+"/optFlowModule:o").c_str());
    outPortModuleAvg.open(("/"+name+"/optFlowModuleAvg:o").c_str());

    return true;
}

void gazeEvaluatorThread::run()
{
    if(!optFlow.empty())
        optFlow.setTo(Scalar(0));

    if (isStarting)
    {
        ImageOf<PixelRgb> *tmp = imgPortIn->read(false);

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
        ImageOf<PixelRgb> *tmp = imgPortIn->read(false);
        if(tmp!=NULL)
        {
            *imageIn=*tmp;
        }
        // 2A - Put the read imag into imgInNext
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

IplImage* gazeEvaluatorThread::draw2DMotionField(double &_avg)
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
    float sum = 0;
    float cnt = 0;

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
            if (hyp > 1e-1 && 
                i > PIXELS_TO_DISCARD && i < optFlow.rows - PIXELS_TO_DISCARD &&
                j > PIXELS_TO_DISCARD && j < optFlow.cols - PIXELS_TO_DISCARD)
            {
                sum = sum + hyp;
                cnt++;
            }

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
    if (cnt!=0)
    {
        _avg = sum/cnt;
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
    // Send the optical flow as a superimposition of the input image
    double avg = 0;
    IplImage* imgOptFlow = (IplImage*)draw2DMotionField(avg);

    Bottle b;
    b.clear();
    // b.addDouble(cnt);
    // b.addDouble(sum);
    b.addDouble(avg);
    outPortModuleAvg.write(b);

    if(imgOptFlow!=NULL)
    {
        printMessage(0,"I've got an optical flow! Avg: %g\n",avg);
        printMessage(2,"imgOptFlow depth %i\n",imgOptFlow->depth);
        ImageOf<PixelRgb> outim;
        outim.wrapIplImage(imgOptFlow);
        imgOutportFlow.write(outim);
    }
    cvReleaseImage(&imgOptFlow);

    // Send the pixel-by-pixel norm of the optical flow in a standalone port
    IplImage* imgOptFlowModule=cvCreateImage(cvSize(imgInPrev->width,imgInPrev->height),8,3);
    drawFlowModule(imgOptFlowModule);

    if(imgOptFlowModule!=NULL)
    {
        printMessage(1,"imgOptFlowModule depth %i\n",imgOptFlowModule->depth);
        ImageOf<PixelBgr> outim;
        outim.wrapIplImage(imgOptFlowModule);
        imgOutportModule.write(outim);
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
        closePort(imgPortIn);
        if(imageIn)
        {
            delete imageIn;
            imageIn = NULL;
        }
}

// empty line to make gcc happy
