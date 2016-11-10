#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
private:
    cv::Mat default_cameraMatrix ;
    cv::Mat default_distCoeffs;
public:
    Camera();
    ~Camera();
    void camera_calibration();
};

#endif // CAMERA_H
