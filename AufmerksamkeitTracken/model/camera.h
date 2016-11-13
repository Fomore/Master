#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
private:
    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;
    void correct_Image();
public:
    Camera();
    ~Camera();
    void camera_calibration();
};

#endif // CAMERA_H
