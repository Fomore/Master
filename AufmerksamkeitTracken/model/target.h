#ifndef TARGET_H
#define TARGET_H

#include <QString>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include "model/camera.h"

class Target
{
private:
    //Koordinaten im Weltkoordinatensystem
    double mPoint[9][3];
    double mPoint2[3][3];
    double mFHeight, mTHeight;
    double mCameraHeight;
    double mCameraHeight2;

    void getWorldPosition(QStringList list, double &x, double &y, double &z);
    Camera* mKamera;
public:
    Target(Camera *cam);
    cv::Point2d calcAngle(double X, double Y, double Z);
    void getPoint(QString Name, cv::Point3d &Point);
    void getPoint(size_t id, double &x, double &y, double &z);
    void getOrienation(QString name, cv::Point2d& WAngle, cv::Point3d& WPosition, cv::Point2d& RAngle);
};

#endif // TARGET_H
