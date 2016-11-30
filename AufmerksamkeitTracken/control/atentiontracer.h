#ifndef ATENTIONTRACER_H
#define ATENTIONTRACER_H

#include <opencv2/opencv.hpp>

class AtentionTracer
{
public:
    AtentionTracer();
    ~AtentionTracer();
    void newEulerPosition(int model, cv::Vec6d headPose);
};

#endif // ATENTIONTRACER_H
