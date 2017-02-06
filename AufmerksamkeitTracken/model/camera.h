#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
private:
    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;

    cv::Mat map1, map2;

    int ID;
    void setCameraParameter(int id);
public:
    Camera(int id);
    Camera();
    ~Camera();
    void get_camera_params(double &fx, double &fy, double &cx, double &cy);
    void correct_Image_Init(cv::Size imageSize);
};

#endif // CAMERA_H
