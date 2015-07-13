#ifndef _LW_OCV_
#define _LW_OCV_

#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <cstdio>

///////////////////////////////////////////////////
cv::Mat calcImgGrad(const cv::Mat& img);
cv::Mat calcCanny(const cv::Mat& img);
cv::Mat calcLiveWireCostFcn(const cv::Mat& img);
cv::Mat normImage(const cv::Mat& img);

///////////////////////////////////////////////////
// Structure definitin of the active list entries
struct SEntry {
    short    sX;         // X-coordinate
    short    sY;         // Y-coordinate
    long     lLinInd;    // Linear index from x and y for 1D-array
    float    flG;        // The current cost from seed to (X,Y)^T
};

// Inline function to determin minimum of two numbers
inline long ifMin(long a, long b)
{
    return a < b ? a : b;
}

// Inline function to determin maximum of two numbers
inline long ifMax(long a, long b)
{
    return a > b ? a : b;
}

// Inline function to calculate linear index from subscript indices.
//inline long ifLinInd(short sX, short sY, short sNY)
inline long ifLinInd(short sX, short sY, short sNX)
{
//    return long(sX)*long(sNY) + long(sY);
    return long(sY)*long(sNX) + long(sX);
}

//
//  FUNCTION fFindMinG
//
//  Get the Index of the vector entry with the smallest dQ in pV
//
long fFindMinG(SEntry *pSList, long lLength);

//
//  FUNCTION fFindLinInd
//
//  Get the Index of the list entry in *pSList lLinInd == lInd
//
long fFindLinInd(SEntry *pSList, long lLength, long lInd);

void calcLiveWireP(const cv::Mat& imgS,
                   int dX, int dY,
                   cv::Mat& iPX, cv::Mat& iPY,
                   double dRadius = 10000.0, int LISTMAXLENGTH = 10000);

void calcLiveWireGetPath(const cv::Mat& ipx,
                         const cv::Mat& ipy,
                         cv::Point pxy,
                         std::vector<cv::Point>& path, int iMAXPATH = 1000);

cv::Point calcIdealAnchor(const cv::Mat& imgS, cv::Point pxy, int rad=4);

///////////////////////////////////////////////////
class OCVLiveWire {
public:
    OCVLiveWire();
    OCVLiveWire(const cv::Mat& img, bool modeDebug = false);
    ~OCVLiveWire() {}
    void loadImage(const cv::Mat& img, bool modeDebug = false);
    void initParams();

    bool isLoadedData() const;
    bool isStartedPath() const;
    void setDebugMode(bool mode);
    cv::Point getCPoint() const;
    cv::Point getMPoint() const;
    void clean();
    //
    void calcLWP(cv::Point p, bool isAnchorIdeal=true);
    void calcLWPath(const cv::Point& p, bool isAnchorIdeal=true);
    void incPath();

    void helpDrawImgPXY(const std::string& winName = "win-pxy");
    void helpDrawImgF(const std::string& winName = "win-pxy");

private:
    bool    isLoaded;
    bool    isDebug;
    cv::Mat imgf;
    cv::Mat iPX,iPY;
    cv::Point cPoint;
    cv::Point mPoint;
    bool isStartPath;
public:
    std::vector<cv::Point> pathTot;
    std::vector<cv::Point> path;
    int parRadiusP; // P-Map Radius
    int parRadiusA; // anchor search radis
    int parRadiusPath; // max distance between cPoint and mPoint
};

#endif
