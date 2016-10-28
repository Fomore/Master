#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <opencv2/opencv.hpp>

class Ellipse
{
public:
    Ellipse();
    ~Ellipse();
    cv::RotatedRect calculate_Ellipse(cv::Mat image);
};

#endif // ELLIPSE_H
