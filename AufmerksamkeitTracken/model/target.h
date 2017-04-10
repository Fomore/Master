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
    double calcAngle(double ge, double an);
    cv::Point3d calcAngle(double x, double y, double z);
    Camera* mKamera;
public:
    Target(Camera *cam);
    void getPoint(QString Name, cv::Point3d &Point);
    void getPoint(size_t id, double &x, double &y, double &z);
    void getOrienation(QString name, cv::Point3d& WAngle, cv::Point3d& WPosition);
};

#endif // TARGET_H
