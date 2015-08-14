#include "lwocv.h"


OCVLiveWire lw1,lw2;
cv::Mat img, imgc, markersT;
cv::Mat imgs;
cv::Point pCenter;
cv::Point pC1, pC2, pNGBH;
int idxC1, idxC2, idxNGBH;
bool isDrawCircle;
int radC;
std::vector<cv::Point> contour1;
std::vector<cv::Point> path1, path2;

int sliderVal, sliderMin=50, sliderMax=300;

void drawContour(cv::Mat& img, const std::vector<std::vector<cv::Point> >& v) {
    for(int vv=0; vv<v.size(); vv++) {
        const std::vector<cv::Point>& tptr = v.at(vv);
        for(int ii=1; ii<tptr.size(); ii++) {
            cv::line(img, tptr.at(ii-1), tptr.at(ii), cv::Scalar(0,255,0), 2);
        }
    }
}

void drawContour1(cv::Mat& img, const std::vector<cv::Point>& v, cv::Scalar color = cv::Scalar(0,255,0)) {
    for(int ii=1; ii<v.size(); ii++) {
        cv::line(img, v.at(ii-1), v.at(ii), color, 2);
    }
}

void findNGBHPoint(const std::vector<cv::Point>& v, const cv::Point& p0, cv::Point& pNGBH, int& idxMin) {
    idxMin = 0;
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

double calcL2(const cv::Point& p1, const cv::Point& p2) {
    double dx = (double)p1.x - (double)p2.x;
    double dy = (double)p1.y - (double)p2.y;
    return sqrt(dx*dx + dy*dy);
}

void findPtsInContour(const std::vector<cv::Point>& v, const cv::Point& p0, int& idx, cv::Point& pf, bool isInc=true) {
    int num     = v.size();
    int num4    = num/7;
    int idxNGBH;
    cv::Point pNGBH;
    findNGBHPoint(v, p0, pNGBH, idxNGBH);
    int cnt = 0;
    int di = +1;
    if(!isInc) {
        di = -1;
    }
    int cpos = idxNGBH;
    int cold = cpos;
    double dstC = 0.;
    double dstA = 0.;
    while (true) {
        cpos += di;
        if(cpos<0) {
            cpos = num-1;
        }
        if(cpos>=num) {
            cpos = 0;
        }
        dstC += calcL2(v[cpos], v[cold]);
        dstA  = calcL2(v[cpos], v[idxNGBH]);
        if(dstC>70) {
            std::cout << "break-by-dstC" << std::endl;
            break;
        }
        cold = cpos;
        cnt++;
        if(cnt>num4) {
            std::cout << "break-by-CNT" << std::endl;
            break;
        }
    }
    idx = cpos;
    pf  = v[idx];
}

void generateContour() {
    img = cv::Mat::zeros(imgs.rows,imgs.cols,CV_8U);
    cv::circle(img,pCenter,radC,cv::Scalar::all(255),CV_FILLED);

    img.convertTo(markersT, CV_8U);
    cv::threshold(markersT, markersT, 150, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(markersT, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    contour1 = contours.at(0);
}

void drawResults(bool isShowCircle=true) {
    cv::Mat tmp;
//    imgc.copyTo(tmp);
    imgs.copyTo(tmp);
//    if(isDrawCircle) {
        cv::circle(tmp, pCenter, 3, cv::Scalar(255,0,0));
        cv::circle(tmp, pCenter, radC, cv::Scalar(255,0,0));
//    }
    drawContour1(tmp, contour1);
//    std::cout << path1.size() << "/" << path2.size() << std::endl;
    drawContour1(tmp, path1, cv::Scalar(0,0,255));
    drawContour1(tmp, path2, cv::Scalar(0,0,255));
    cv::circle(tmp, pNGBH, 3, cv::Scalar(0,0,255));
    cv::circle(tmp, pC1, 3, cv::Scalar(255,0,0));
    cv::circle(tmp, pC2, 3, cv::Scalar(255,100,0));
    cv::imshow("win", tmp);
}

void printCnt(const std::vector<cv::Point>& c) {
    std::cout << "c(" << c.size() << ") = { ";
    for(int ii=0; ii<c.size(); ii++) {
        std::cout << "(" << ii << ":" << c.at(ii) << "), ";
    }
    std::cout << "};" << std::endl;
}

void on_trackbar( int, void* ) {
    std::cout << "::on_trackbar()" << std::endl;
}

static void onMouse( int event, int x, int y, int flags, void* )
{
    pCenter = cv::Point(x,y);
    bool isKeyShift = (flags&CV_EVENT_FLAG_SHIFTKEY);
    bool isKeyCtrl  = (flags&CV_EVENT_FLAG_CTRLKEY);
    bool isKeyMLB    = (flags&CV_EVENT_FLAG_LBUTTON);
    bool isKeyMRB    = (flags&CV_EVENT_FLAG_RBUTTON);

    if(isKeyMRB) {
        contour1.clear();
//        lw1.clean();
//        lw2.clean();
        generateContour();
    }

    if(event==CV_EVENT_LBUTTONDOWN) {
//        pCenter = cv::Point(x,y);
        findNGBHPoint(contour1, pCenter, pNGBH, idxNGBH);
        findPtsInContour(contour1, pCenter, idxC1, pC1, true);
        findPtsInContour(contour1, pCenter, idxC2, pC2, false);
        //
        lw1.calcLWP(pC1,true);
        lw2.calcLWP(pC2,true);
//        drawResults();
        isDrawCircle = true;
    }
    if(event==CV_EVENT_LBUTTONUP) {
        std::cout << "\n\n" << std::endl;
        std::cout << pCenter << " : " << lw1.getCPoint() << "/" << lw2.getCPoint() << std::endl;
        printCnt(lw1.path);
        std::cout << pC1 << " / " << pC2 << std::endl;
//        std::cout << "STOP" << std::endl;
        std::vector<cv::Point> newcnt;
        int numC1 = lw1.path.size();
        int numC2 = lw2.path.size();
        int sizC0 = contour1.size();
        for(int ii=numC1; ii-->0;) {
            newcnt.push_back(lw1.path.at(ii));
        }
        for(int ii=idxC1; ; ii++) {
            if(ii>=sizC0) {
                ii=0;
            }
            newcnt.push_back(contour1.at(ii));
            if(ii==idxC2) {
                break;
            }
        }
        for(int ii=0; ii<numC2; ii++) {
            newcnt.push_back(lw2.path.at(ii));
        }
        contour1 = newcnt;
        drawResults();
    }
    if(isKeyMLB) {
//        std::cout << "(" << x << ", " << y << ")" << std::endl;
        if(lw1.isStartedPath()) {
            lw1.calcLWPath(pCenter);
            path1.clear();
            path1.push_back(lw1.getCPoint());
            path1.insert(path1.end(), lw1.path.begin(), lw1.path.end());
        }
        if(lw2.isStartedPath()) {
            lw2.calcLWPath(pCenter);
            path2.clear();
            path2.push_back(lw2.getCPoint());
            path2.insert(path2.end(), lw2.path.begin(), lw2.path.end());
        }
        isDrawCircle = true;
    } else {
        isDrawCircle = false;
    }
    //
    drawResults();
}


template <typename T>
void printVector(const std::vector<T>& v) {
    std::cout << "v(" << v.size() << ") = {" <<std::endl;
    for(int ii=0; ii<v.size(); ii++) {
        std::cout << "\t" << ii << " : " << v.at(ii) << std::endl;
    }
    std::cout << "}" << std::endl;
}

//////////////////////////////////////
int main(int argc, char* argv[]) {

    /*
    int N=10;
    std::vector<int> v;
    for(int ii=0; ii<N; ii++) {
        v.push_back(ii);
    }
    printVector(v);
    std::cout << "---" << std::endl;
    int sizv = v.size();
    for(int ii=sizv; ii-->0;) {
        std::cout << ii << " : " << v.at(ii) << std::endl;
    }
    exit(1);
    */

//    imgs = cv::imread("/home/ar/img/lena.png",CV_LOAD_IMAGE_COLOR);
//    imgs = cv::imread("/home/ar/img/doge2.jpg",CV_LOAD_IMAGE_COLOR);
    imgs = cv::imread("/home/ar/img/example_gomel_pole_1.png",CV_LOAD_IMAGE_COLOR);
    lw1.loadImage(imgs);
    lw2.loadImage(imgs);

    radC    = 50;
    isDrawCircle = false;
    int siz = 512;
    int rad = 100;
    cv::Point p0=cv::Point(imgs.cols/2, imgs.rows/2);


    img = cv::Mat::zeros(imgs.rows,imgs.cols,CV_8U);
    cv::circle(img,p0,rad,cv::Scalar::all(255),CV_FILLED);

    img.convertTo(markersT, CV_8U);
    cv::threshold(markersT, markersT, 150, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(markersT, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    contour1 = contours.at(0);

    cv::cvtColor(img,imgc, cv::COLOR_GRAY2BGR);
    drawContour1(imgc, contour1);


    cv::imshow("win", imgc);
    cv::createTrackbar( "Radius", "win", &radC, sliderMax, on_trackbar );
    cv::setMouseCallback("win", onMouse, 0);
    while(true) {
        char key = cv::waitKey(0);
        if(key==27) {
            break;
        }
    }
    return 0;
}
