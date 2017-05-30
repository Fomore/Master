#ifndef TARGET_H
#define TARGET_H

#include <QString>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include "model/camera.h"

class Target
{
protected:
    //Koordinaten im Weltkoordinatensystem
    std::vector<cv::Point3d> mPoints;
    QStringList mReferenceName;
    double mPoint[9][3];
    double mPoint2[3][3];
    double mFHeight, mTHeight;
    double mCameraHeight;
    double mCameraHeight2;

    void getWorldPosition(QStringList list, double &x, double &y, double &z);
    Camera* mKamera;
public:
    Target();
    cv::Point2d calcAngle(double X, double Y, double Z);
    cv::Point2d calcAngle(cv::Point3d Point);
    bool getPoint(QString Name, cv::Point3d &Point);
    /*
    void getPoint(size_t id, double &x, double &y, double &z);
    void getPoint(QString Name, cv::Point3d &Point);
    */
    void getOrienation(QString name, cv::Point2d& WAngle, cv::Point3d& WPosition, cv::Point2d& RAngle, cv::Point3d& Target);
    void addTarget(QString name, cv::Point3d Target);
    void loadFromFile(QString FileName);
    void clear();
};

#endif // TARGET_H
