#ifndef BOXHANDLER_H
#define BOXHANDLER_H

#include <opencv2/opencv.hpp>
#include "LandmarkCoreIncludes.h"

class BoxHandler
{
    cv::Rect2d Rec_old;
    cv::Rect Rec_new;
    cv::Size mImageSize;
    cv::Size mMinSize;
    double mScall = 1.8;
    double fx = 1.0;
    bool mAutoSize = true;
public:
    BoxHandler(int width, int height);
    ~BoxHandler();
    void getSection(int &x, int &y, int &w, int &h);
    void getSection(cv::Rect &rect);
    cv::Rect getRect();
    void getImage(cv::Mat Image, cv::Mat &Part);

    void toSection(LandmarkDetector::CLNF &clnf);
    void toImage(LandmarkDetector::CLNF &clnf);
    void setAutoSize(bool use);
    double getImageScall();

    void setBoxScall(double s);
    double getBoxScall();

    void setBoxMinSize(int w, int h);
    void setImageSize(int w, int h);

    void setNewRect(cv::Rect rec);
    void setOldRect(cv::Rect2d rec);
    void setOldRect(double X, double Y, double W, double H);

    bool existBox();
};

#endif // BOXHANDLER_H
