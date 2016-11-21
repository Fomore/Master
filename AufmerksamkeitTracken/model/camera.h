#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera
{
private:
    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;

    bool init;
    cv::Mat map1, map2;

    std::vector<std::vector<cv::Point2f> > get_perfect_Points(std::vector<std::vector<cv::Point2f> > points, const cv::Size dim, int maxImages);

    int ID;
    void setCameraParameter(int id);
public:
    Camera(int id);
    Camera();
    ~Camera();
    void camera_calibration(std::string path);
    void get_camera_params(double &fx, double &fy, double &cx, double &cy);
    void correct_Image();
    void correct_Image(cv::Mat frame);
    void correct_Image_Init(int height, int width);
};

#endif // CAMERA_H
