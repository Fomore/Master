#ifndef IMAGESECTION_H
#define IMAGESECTION_H

#include <opencv2/opencv.hpp>
#include "LandmarkCoreIncludes.h"

class ImageSection
{
    cv::Rect Rec_old;
    cv::Rect Rec_new;
    cv::Size mImageSize;
    cv::Size2d mMinSize;
    double mScall = 1.3;
    double fx = 1.0;
public:
    ImageSection(int width, int height);
    ~ImageSection();
    bool getSection(int &x, int &y, int &w, int &h);
    void getImage(cv::Mat Image, cv::Mat &Part);
    void newRect(cv::Rect rec);
    void setScall(double s);
    void toSection(LandmarkDetector::CLNF &clnf);
    void toImage(LandmarkDetector::CLNF &clnf);
};

#endif // IMAGESECTION_H
