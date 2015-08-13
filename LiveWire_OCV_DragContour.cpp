#include "lwocv.h"


OCVLiveWire lw;
cv::Mat img, imgc, markersT;
cv::Point pCenter;
bool isDrawCircle;
int radC;
std::vector<cv::Point> contour1;

void drawContour(cv::Mat& img, const std::vector<std::vector<cv::Point> >& v) {
    for(int vv=0; vv<v.size(); vv++) {
        const std::vector<cv::Point>& tptr = v.at(vv);
        for(int ii=1; ii<tptr.size(); ii++) {
            cv::line(img, tptr.at(ii-1), tptr.at(ii), cv::Scalar(0,255,0), 2);
        }
    }
}

void drawContour1(cv::Mat& img, const std::vector<cv::Point>& v) {
    for(int ii=1; ii<v.size(); ii++) {
        cv::line(img, v.at(ii-1), v.at(ii), cv::Scalar(0,255,0), 2);
    }
}

void findNGBHPoint(const std::vector<cv::Point>& v, const cv::Point& p0, cv::Point& pNGBH) {
    int     idxMin = 0;
    double  dstMin = DBL_MAX;
    double  dstTmp;
    for(int ii=0; ii<v.size(); ii++) {
        double dx = (double)v[ii].x - (double)p0.x;
        double dy = (double)v[ii].y - (double)p0.y;
        dstTmp  = sqrt(dx*dx + dy*dy);
        if(dstTmp<dstMin) {
            dstMin = dstTmp;
            idxMin = ii;
        }
    }
    pNGBH = v[idxMin];
}

void drawResults(bool isShowCircle=true) {
    cv::Mat tmp;
    imgc.copyTo(tmp);
    if(isDrawCircle) {
        cv::circle(tmp, pCenter, 3, cv::Scalar(255,0,0));
        cv::circle(tmp, pCenter, radC, cv::Scalar(255,0,0));
    }
    drawContour1(tmp, contour1);
    cv::Point pNGBH;
    findNGBHPoint(contour1, pCenter, pNGBH);
    cv::circle(tmp, pNGBH, 3, cv::Scalar(0,0,255));
    cv::imshow("win", tmp);
}

static void onMouse( int event, int x, int y, int flags, void* )
{

    bool isKeyShift = (flags&CV_EVENT_FLAG_SHIFTKEY);
    bool isKeyCtrl  = (flags&CV_EVENT_FLAG_CTRLKEY);

    if(event==CV_EVENT_LBUTTONDOWN) {
        std::cout << "(" << x << ", " << y << ")" << std::endl;
        pCenter = cv::Point(x,y);
        isDrawCircle = true;
    } else {
        isDrawCircle = false;
    }
    //
    drawResults();
}


//////////////////////////////////////
int main(int argc, char* argv[]) {

    radC    = 50;
    isDrawCircle = false;
    int siz = 512;
    int rad = 100;
    cv::Point p0=cv::Point(siz/2, siz/2);


    img = cv::Mat::zeros(siz,siz,CV_8U);
    cv::circle(img,p0,rad,cv::Scalar::all(255),CV_FILLED);

    img.convertTo(markersT, CV_8U);
    cv::threshold(markersT, markersT, 150, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(markersT, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    contour1 = contours.at(0);

    cv::cvtColor(img,imgc, cv::COLOR_GRAY2BGR);
    drawContour1(imgc, contour1);


    cv::imshow("win", imgc);
    cv::setMouseCallback("win", onMouse, 0);
    while(true) {
        int key = cv::waitKey(0);
        if(key==27) {
            break;
        }
    }
    return 0;
}
