#include "lwocv.h"

cv::Mat img, imgF, iPX, iPY;
cv::Point cPoint;
bool isStarted = false;
std::vector<cv::Point> pathTot, path;


///////////////////////////////////////////////
//double dist(const cv::Point)

static void onMouse( int event, int x, int y, int flags, void* )
{

    bool isKeyShift = (flags&CV_EVENT_FLAG_SHIFTKEY);
    bool isKeyCtrl  = (flags&CV_EVENT_FLAG_CTRLKEY);

    if(event==CV_EVENT_LBUTTONDOWN) {
        isStarted = true;
        calcLiveWireP(imgF,x,y,iPX,iPY,100);
        cv::Mat iPXY;
        cPoint = cv::Point(x,y);
        if(!isKeyShift) {
            cPoint = calcIdealAnchor(imgF, cPoint);
        }
        cv::hconcat(iPX,iPY,iPXY);
        cv::imshow("win-iPXY", normImage(iPXY));
        if(path.size()>0) {
            pathTot.insert(pathTot.end(), path.begin(), path.end());
        }
    } else {
        cv::Point pxy(x,y);
        if(!isKeyShift) {
            pxy = calcIdealAnchor(imgF, pxy);
        }
        if(isStarted) {
            double pDST = cv::norm(pxy-cPoint);
            if( (pDST>1.0) && (pDST<200)) {
                calcLiveWireGetPath(iPX,iPY,pxy,path);
            } else {
                path.clear();
                path.push_back(cPoint);
                path.push_back(pxy);
            }
            cv::Mat imgOUT;
            img.copyTo(imgOUT);
            for(int ii=1; ii<pathTot.size(); ii++) {
                cv::line(imgOUT,pathTot[ii-1],pathTot[ii],cv::Scalar(255,0,0),2);
            }
            for(int ii=1; ii<path.size(); ii++) {
                cv::line(imgOUT,path[ii-1],path[ii],cv::Scalar(0,255,0),2);
            }
            cv::circle(imgOUT,cPoint,3,cv::Scalar(0,0,255),1);
            cv::imshow("win", imgOUT);
        }
    }
}

///////////////////////////////////////////////
int main(int argc, char* argv[]) {

    img = cv::imread("/home/ar/img/lena.png", 1);
//    img = cv::imread("/home/ar/img/MERKAARTOR/velikoselskoye_LC81850232014073LGN00_crop_psharp8.tif", 1);

    imgF = calcLiveWireCostFcn(img);

    cv::imshow("win", normImage(imgF));
    cv::setMouseCallback("win", onMouse, 0);
    while(true) {
        int key = cv::waitKey(0);
        if(key==27) {
            break;
        }
    }
    return 0;
}
