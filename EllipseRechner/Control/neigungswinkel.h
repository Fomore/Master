#ifndef NEIGUNGSWINKEL_H
#define NEIGUNGSWINKEL_H

#include <opencv2/opencv.hpp>

class Neigungswinkel
{
public:
    Neigungswinkel();
    ~Neigungswinkel();
    void calculate(cv::RotatedRect ellipse);
};

#endif // NEIGUNGSWINKEL_H
