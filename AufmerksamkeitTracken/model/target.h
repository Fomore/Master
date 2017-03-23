#ifndef TARGET_H
#define TARGET_H

#include <QString>
#include <QStringList>
#include <opencv2/opencv.hpp>

class Target
{
private:
    //Koordinaten im Weltkoordinatensystem
    double mPoint[9][2];
    double mFHeight, mTHeight;
    double mCameraHeight;

    void getWorldPosition(QStringList list, double &x, double &y);
    double calcAngle(double ge, double an);
public:
    Target();
    void getPoint(QString name, double &x, double &y);
    void getPoint(size_t id, double &x, double &y);
    void getOrienation(QString name, cv::Point3d& point, double &WorldX, double &WorldZ);
};

#endif // TARGET_H
