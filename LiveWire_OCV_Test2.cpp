#include "lwocv.h"


OCVLiveWire lw;
cv::Mat img;

///////////////////////////////////////////////
static void onMouse( int event, int x, int y, int flags, void* )
{

    bool isKeyShift = (flags&CV_EVENT_FLAG_SHIFTKEY);
    bool isKeyCtrl  = (flags&CV_EVENT_FLAG_CTRLKEY);

    if(event==CV_EVENT_LBUTTONDOWN) {
        lw.calcLWP(cv::Point(x,y), !isKeyShift);
        lw.incPath();
        lw.helpDrawImgPXY();
    } else {
        cv::Point pxy(x,y);
        if(lw.isStartedPath()) {
            lw.calcLWPath(pxy, !isKeyShift);
            cv::Mat imgOUT;
            img.copyTo(imgOUT);
            for(int ii=1; ii<lw.pathTot.size(); ii++) {
                cv::line(imgOUT,lw.pathTot[ii-1],lw.pathTot[ii],cv::Scalar(255,0,0),2);
            }
            std::vector<cv::Point> tpath;
            tpath.push_back(lw.getCPoint());
            tpath.insert(tpath.end(), lw.path.begin(), lw.path.end());
            for(int ii=1; ii<tpath.size(); ii++) {
                cv::line(imgOUT,tpath[ii-1],tpath[ii],cv::Scalar(0,255,0),2);
            }
            cv::circle(imgOUT,lw.getCPoint(),3,cv::Scalar(0,0,255),1);
            cv::imshow("win", imgOUT);
        }
    }
}

///////////////////////////////////////////////
int main(int argc, char* argv[]) {

    img = cv::imread("/home/ar/img/lena.png", 1);
//    img = cv::imread("/home/ar/img/MERKAARTOR/velikoselskoye_LC81850232014073LGN00_crop_psharp8.tif", 1);

//    imgF = calcLiveWireCostFcn(img);
    lw.loadImage(img);

    cv::imshow("win", img);
    cv::setMouseCallback("win", onMouse, 0);
    while(true) {
        int key = cv::waitKey(0);
        if(key==27) {
            break;
        }
    }
    return 0;
}
