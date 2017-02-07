#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <QString>

class Camera
{
private:
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat map1, map2;
    int ImageWight, ImageHeight;
    int ID;

    cv::VideoCapture video;

public:
    Camera(int id = 0);
    ~Camera();
    void get_camera_params(double &fx, double &fy, double &cx, double &cy, int &x, int &y);

    int getCameraID();

    bool getFrame(cv::Mat &img);

    bool setPath(QString path);
    void setCameraParameter(int id);

    void correct_Image(cv::Mat img);
};

#endif // CAMERA_H
