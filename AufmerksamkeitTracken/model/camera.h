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
    cv::Vec3d mTranslation;
    int ImageWight, ImageHeight;
    int ID;
    bool mUseCorrection = true;
    double mfx, mfy;

    cv::VideoCapture video;

public:
    Camera(int id = 0);
    ~Camera();
    void get_camera_params(double &fx, double &fy, double &cx, double &cy, int &x, int &y);

    int getCameraID();

    bool getFrame(cv::Mat &img);
    bool getFrame(cv::Mat &img, size_t pos);

    cv::Vec3d getTranslation();

    bool setPath(QString path);
    void setCameraParameter(int id);

    double getTimeSec();

    bool UseCorrection();

    void correct_Image(cv::Mat img);
    cv::Rect correct_Rect(cv::Rect rec);
    void correctTest(cv::Scalar col, std::string name);

    void setUseCorrection(bool c);

    void setFrame(size_t frame);
    void setFrame(double frame);
    void setImageSize(int Wight, int Height);

    cv::Vec3d rotateToWorld(cv::Point3f in);
    cv::Vec3d rotateToWorld(cv::Vec3d in);

    cv::Vec3d rotateToCamera(cv::Vec3d in);
    cv::Vec3d rotateToCamera(cv::Point3d in);

    cv::Matx33d getRotationMatrix();

    size_t getFrameNr();

    void setFxFy(double fx, double fy);
    double getFx();
    double getFy();
};

#endif // CAMERA_H
