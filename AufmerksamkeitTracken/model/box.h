#ifndef BOX_H
#define BOX_H

#include <opencv2/opencv.hpp>

class Box
{
private:
    cv::Rect mRect;
    std::string mName;
    std::string mEvent;
    double mLandmarks[5][2];
    bool mIsLandmarksSet;
public:
    Box(cv::Rect rec, std::string name, std::string event);
    Box(cv::Rect rec, std::string name, std::string event, double land[5][2]);
    std::string getName();
    std::string getEvent();
    cv::Rect getRect();
    void setLandmarks(double land[5][2]);
    void getLandmarks(double land[5][2]);
    bool isLandmark();
};

#endif // BOX_H
