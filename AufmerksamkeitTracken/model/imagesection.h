#ifndef IMAGESECTION_H
#define IMAGESECTION_H

#include <opencv2/opencv.hpp>
#include "LandmarkCoreIncludes.h"

class ImageSection
{
    cv::Rect Rec_old;
    cv::Rect Rec_new;
    cv::Size mImageSize;
    cv::Size mMinSize;
    double mScall = 1.3;
    double fx = 1.0;
    bool mAutoSize = true;
public:
    ImageSection(int width, int height);
    ~ImageSection();
    bool getSection(int &x, int &y, int &w, int &h);
    cv::Rect getRect();
    void getImage(cv::Mat Image, cv::Mat &Part);
    void newRect(cv::Rect rec);
    void toSection(LandmarkDetector::CLNF &clnf);
    void toImage(LandmarkDetector::CLNF &clnf);
    void setAutoSize(bool use);
    double getImageScall();

    void setBoxScall(double s);
    double getBoxScall();

    void setBoxMinSize(int w, int h);
};

#endif // IMAGESECTION_H
