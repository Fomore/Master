#ifndef NEIGUNGSWINKEL_H
#define NEIGUNGSWINKEL_H

#include <opencv2/opencv.hpp>

class Neigungswinkel
{
private:
    double fehler[4*7][2];
public:
    Neigungswinkel();
    ~Neigungswinkel();
    void calculate(cv::RotatedRect ellipse, double x, double y);
    void calculate2(cv::RotatedRect ellipse, double x, double y);
};

#endif // NEIGUNGSWINKEL_H
