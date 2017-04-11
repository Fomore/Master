#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <QString>

class Camera
{
private:
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat map1, map2;
    cv::Matx33d mRotMatrix;
    cv::Vec3d mRotation;
    cv::Vec3d mTranslation;
    int ImageWight, ImageHeight;
    int ID;
    bool mUseCorrection = true;

    cv::VideoCapture video;

public:
    Camera(int id = 0);
    ~Camera();
    void get_camera_params(double &fx, double &fy, double &cx, double &cy, int &x, int &y);

    int getCameraID();

    bool getFrame(cv::Mat &img);
    bool getFrame(cv::Mat &img, size_t pos);

    bool setPath(QString path);
    void setCameraParameter(int id);

    void correct_Image(cv::Mat img);
    cv::Rect correct_Rect(cv::Rect rec);
    void correctTest();

    void setUseCorrection(bool c);

    void setFrame(size_t frame);
    void setImageSize(int Wight, int Height);

    cv::Matx33d rotateWorldToCamera(cv::Matx33d in);
    cv::Matx33d rotateCameraToWorld(cv::Matx33d in);
    cv::Vec3d rotateToWorld(cv::Point3f in);
    cv::Vec3d rotateToWorld(cv::Vec3d in);
    cv::Vec3d rotateToCamera(cv::Vec3d in);

    cv::Vec3d getRotation();
    cv::Matx33d getRotationMatrix();

    size_t getFrameNr();
};

#endif // CAMERA_H
