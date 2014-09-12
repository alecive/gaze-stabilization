#include "gazeEvaluatorThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define IMG_CROP_SIZE         50.0

gazeEvaluatorThread::gazeEvaluatorThread(int _rate, string _name, string _robot, int _v) :
                                           RateThread(_rate), name(_name), robot(_robot), verbosity(_v)
{
    inPort = new BufferedPort<ImageOf<PixelRgb> >;
    outPort = new BufferedPort<ImageOf<PixelRgb> >;
    imgPortOutMod  = new BufferedPort<ImageOf<PixelRgb> >;

    portOutModAvg  = new BufferedPort<Bottle >;

    isStarting = 1;
}

bool gazeEvaluatorThread::threadInit()
{
    inPort       ->open(("/"+name+"/img:i").c_str());
    outPort  ->open(("/"+name+"/optFlow:o").c_str());
    imgPortOutMod   ->open(("/"+name+"/optFlowModule:o").c_str());
    portOutModAvg   ->open(("/"+name+"/optFlowModuleAvg:o").c_str());

    Network::connect("/icub/camcalib/left/out",("/"+name+"/img:i").c_str());
    Network::connect("/icub/cam/left",("/"+name+"/img:i").c_str());
    Network::connect(("/"+name+"/optFlow:o").c_str(),"/gazeEvalFlow");
    Network::connect(("/"+name+"/optFlowModule:o").c_str(),"/gazeEvalFlowModule");

    return true;
}

void gazeEvaluatorThread::run()
{
// // 1  - Get the image
// ImageOf<PixelRgb> *imageIn = inPort->read(false);

// if(imageIn!=NULL)
// {
//     // Wrap the input image into a cv::Mat
//     cv::Mat matIn((IplImage*)imageIn->getIplImage());

//     // Prepare the output port
//     ImageOf<PixelRgb> imageOut;

//     // Resize the output image to be equal to the input image
//     imageOut.resize(*imageIn);

//     // Wrap the output image into a cv::Mat
//     cv::Mat matOut((IplImage*)imageOut.getIplImage());

//     // Copy the input mat into the output mat
//     matOut=matIn.clone();

//     // Send data
//     outPort->prepare()=imageOut;
//     outPort->write();   
// }
// 

    if(!optFlow.empty())
        optFlow.setTo(Scalar(0));

    if (isStarting)
    {
        ImageOf<PixelRgb> *tmp = inPort->read(false);

        if(tmp!=NULL)
        {
            imageInNext = *tmp;
            imageInPrev =  imageInNext;
            isStarting  = false;
            printMessage(0,"Starting..\n");
        }
    }
    else
    {
        // 1  - Get the image
        ImageOf<PixelRgb> *tmp = inPort->read(false);
        if(tmp!=NULL)
        {
            imageInNext = *tmp;
        }

        
        cv::Mat imgInNext((IplImage*)imageInNext.getIplImage());
        cv::Mat imgInPrev((IplImage*)imageInPrev.getIplImage());

        // 2B - Smooth it out
        cv::boxFilter(imgInNext, imgInNext, -1, cv::Size(4,3));

        // 4  - Make it gray
        Mat imgPrevGray;
        Mat imgNextGray;
        cvtColor(imgInPrev,imgPrevGray,CV_RGB2GRAY);
        cvtColor(imgInNext,imgNextGray,CV_RGB2GRAY);

        // 5 - Compute the optical flow
        if(!optFlow.empty())
            optFlow.setTo(Scalar(0));
        
        calcOpticalFlowFarneback(imgPrevGray,imgNextGray,optFlow,0.5,5,9,5,7,1.5,0);

        imageInPrev=imageInNext;

        if (!optFlow.empty() && !imgInPrev.empty())
        {
            sendOptFlow();
        }
    }
}


void gazeEvaluatorThread::sendOptFlow()
{
    // Send the optical flow as a superimposition of the input image
    double avg = 0;
    ImageOf<PixelRgb> imageOutFlow;
    imageOutFlow=imageInNext;

    if (draw2DMotionField(avg,imageOutFlow))
    {
        Bottle &b=portOutModAvg->prepare();
        b.clear();
        b.addDouble(avg);
        portOutModAvg->write();

        printMessage(0,"I've got an optical flow: Avg %g\n",avg);
        // printMessage(0,"I've got an optical flow: Avg %i\n",int(avg));
        outPort->prepare()=imageOutFlow;
        outPort->write();
    }

    // // Send the pixel-by-pixel norm of the optical flow in a standalone port
    // Mat imgOptFlowModule=cv::Mat::zeros(imgInPrev.rows,imgInPrev.cols,CV_8UC1);

    // if(drawFlowModule(imgOptFlowModule))
    // {
    //     // ImageOf<PixelBgr> outim;
    //     // outim.wrapIplImage(imgOptFlowModule);
    //     // imgPortOutMod.write(outim);
    //     // 
    //     // ImageOf<PixelRgb> &imageOut=imgPortOutMod->prepare();
    //     // imageOut.wrapIplImage(imgOptFlowModule);
    //     // imgPortOutMod->write();

    //     ImageOf<PixelRgb> &imageOut=imgPortOutMod->prepare();
    //     imageOut.resize(imageIn->width(),imageIn->height());
    //     cv::Mat imageOutMat((IplImage*)imageOut.getIplImage());
    //     // imageOutMat=cv::Mat::zeros(imgInPrev.rows,imgInPrev.cols,CV_8UC1);
    //     imgPortOutMod->write();
    // }
}

bool gazeEvaluatorThread::draw2DMotionField(double &_avg,ImageOf<PixelRgb> &_iFlow)
{
    cv::Mat iFlowMat((IplImage*)_iFlow.getIplImage());
    int xSpace=7;
    int ySpace=7;
    float cutoff=1;
    float multiplier=5;
    cv::Scalar color=cv::Scalar(125,255,0);

    CvPoint p0 = cvPoint(0,0);
    CvPoint p1 = cvPoint(0,0);

    float deltaX, deltaY, angle, hyp;
    float sum = 0;
    float cnt = 0;

    for(int i=IMG_CROP_SIZE; i<optFlow.rows-IMG_CROP_SIZE; i+=7) 
    {
        for (int j=IMG_CROP_SIZE; j<optFlow.cols-IMG_CROP_SIZE; j+=7)
        {
            p0.x = j;
            p0.y = i;
            deltaX = optFlow.ptr<float>(i)[2*j];
            deltaY = -optFlow.ptr<float>(i)[2*j+1];
            angle = atan2(deltaY, deltaX);
            hyp = sqrt(deltaX*deltaX + deltaY*deltaY);
            if (hyp > 1e-1)
            {
                sum = sum + hyp;
                cnt++;
            }

            if(hyp > cutoff)
            {
                p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
                p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
                cv::line(iFlowMat,p0,p1,color,1,CV_AA);
                p0.x = p1.x + cvRound(3*cos(angle-CV_PI + CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI + CV_PI/4));
                cv::line(iFlowMat,p0,p1,color,1,CV_AA);
                p0.x = p1.x + cvRound(3*cos(angle-CV_PI - CV_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-CV_PI - CV_PI/4));
                cv::line(iFlowMat,p0,p1,color,1,CV_AA);
            }
        }
    }
    if (cnt!=0)
    {
        _avg = sum/cnt;
        return true;
    }

    return false;
}

bool gazeEvaluatorThread::drawFlowModule(Mat &optFlow)
{
    Mat module  =cv::Mat::zeros(optFlow.rows,optFlow.cols,CV_32F);
    Mat moduleU =cv::Mat::zeros(optFlow.rows,optFlow.cols,CV_8UC1);
    Mat vel[2];
    split(optFlow,vel);
    Mat tx=(Mat)vel[0];
    Mat ty=(Mat)vel[1];

    Mat velxpow=tx;
    Mat velypow=ty;

    cv::pow(tx,2,velxpow);
    cv::pow(ty,2,velypow);

    cv::add(velxpow, velypow, module);
    cv::pow(module,0.5,module);
    cv::normalize(module, module, 0.0, 1.0, NORM_MINMAX);
    optFlow=cv::Mat::zeros(optFlow.rows,optFlow.cols,CV_8UC1);
    cv::convertScaleAbs(module,moduleU,255,0);
    std::vector<cv::Mat> mod;
    mod.push_back(moduleU);
    mod.push_back(moduleU);
    mod.push_back(moduleU);
    cv::merge(mod,optFlow);
    return true;
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
        closePort(inPort);
        closePort(outPort);
        closePort(imgPortOutMod);
        closePort(portOutModAvg);
}

// empty line to make gcc happy
