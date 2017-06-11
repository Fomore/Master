#ifndef TARGET_H
#define TARGET_H

#include <QString>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include "model/camera.h"

class Target
{
protected:
    std::vector<cv::Point3d> mTimePoints;
    //Koordinaten im Weltkoordinatensystem
    std::vector<cv::Point3d> mPoints;
    QStringList mReferenceName;
    double mFHeight, mTHeight;

    void getWorldPosition(QStringList list, double &x, double &y, double &z);
    Camera* mKamera;
public:
    Target();
    cv::Point2d calcAngle(double X, double Y, double Z);
    cv::Point2d calcAngle(cv::Point3d Point);
    bool getPoint(QString Name, cv::Point3d &Point);
    void getOrienation(QString name, cv::Point2d& WAngle, cv::Point3d& WPosition, cv::Point2d& RAngle, cv::Point3d& Target);
    void getOrienation(double VideoTime, cv::Point2d& WAngle, cv::Point3d& WPosition, cv::Point2d& RAngle, cv::Point3d& Target);
    void getOrienation(cv::Point2d& WAngle, cv::Point3d& WPosition, cv::Point2d& RAngle, cv::Point3d& Target);
    void addTarget(QString name, cv::Point3d Target);
    void loadFromFile(QString FileName);
    void clear();
    cv::Point3d getTimeTarget(double Time);
};

#endif // TARGET_H
