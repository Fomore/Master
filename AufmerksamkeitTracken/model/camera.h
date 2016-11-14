#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
private:
    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;
public:
    Camera(int id);
    Camera(std::string path);
    ~Camera();
    void camera_calibration(std::string path);
    void get_camera_params(double &fx, double &fy, double &cx, double &cy);
    void correct_Image();
};

#endif // CAMERA_H
