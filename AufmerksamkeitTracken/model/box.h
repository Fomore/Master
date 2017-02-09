#ifndef BOX_H
#define BOX_H

#include <opencv2/opencv.hpp>

class Box
{
private:
    cv::Rect mRect;
    std::string mName;
    std::string mEvent;
public:
    Box(cv::Rect rec, std::string name, std::string event);
    std::string getName();
    std::string getEvent();
    cv::Rect getRect();
};

#endif // BOX_H
