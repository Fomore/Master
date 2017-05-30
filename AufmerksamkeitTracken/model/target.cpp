#include "target.h"

#include <iostream>
#include <math.h>
#include <fstream>
Target::Target()
{
    mFHeight = 1700;
    mTHeight = 1660;
}

void Target::getWorldPosition(QStringList list, double &x, double &y, double &z)
{
    if(list.size() >= 4){
        int a = list[1].toInt();
        int b = list[3].toInt();
        if(a >= 1 && a <= 12){
            z = a*1000.0;
        }else{
            z = 0.0;
        }

        if(b >= 0 && b <= 7){
            x = (b-4)*1000.0;//Innen
            //x = (4-b)*100.0;//Außen
        }else{
            x = 0.0;
        }

        if(list[4] == "Thomas"){
            y = mTHeight;
        }else if(list[4] == "Falko"){
            y = mFHeight;
        }else{
            y = 0;
        }

    }else{
        x = y = z = 0.0;
    }
}

cv::Point2d Target::calcAngle(double X, double Y, double Z)
{
    double z_x = cv::sqrt(X * X + Z * Z);
    double z_y = cv::sqrt(Y * Y + Z * Z);
    return cv::Point2d(atan2(Y, z_x),atan2(X, z_y));
}

cv::Point2d Target::calcAngle(cv::Point3d Point)
{
    return calcAngle(Point.x,Point.y,Point.z);
}

bool Target::getPoint(QString Name, cv::Point3d &Point){
    for(int i = 0; i < mReferenceName.size(); i++){
        if(mReferenceName[i] == Name){
            Point.x = mPoints[i].x;
            Point.y = mPoints[i].y;
            Point.z = mPoints[i].z;
            return true;
        }
    }
    return false;
}

void Target::getOrienation(QString name, cv::Point2d &WAngle, cv::Point3d &WPosition, cv::Point2d &RAngle, cv::Point3d &Target)
{
    QStringList list = name.split(' ');

    getWorldPosition(list,WPosition.x, WPosition.y, WPosition.z);

    getPoint(list[0],Target);

    WAngle = calcAngle(WPosition.x-Target.x,WPosition.y-Target.y,WPosition.z-Target.z);

    cv::Vec3d pos = mKamera->rotateToCamera(Target);
    cv::Vec3d pnt = mKamera->rotateToCamera(WPosition);

    RAngle = calcAngle(pnt[0]-pos[0],pnt[1]-pos[1],pnt[2]-pos[2]);
}

void Target::addTarget(QString name, cv::Point3d Target)
{
    mReferenceName.push_back(name);
    mPoints.push_back(cv::Point3d(Target));
}

void Target::loadFromFile(QString FileName)
{
    clear();

    std::ifstream file(FileName.toStdString());
    std::string line;
    if(file.is_open()){
        while (std::getline(file, line)){
            std::istringstream iss(line);
            std::string name;
            double x,y,z;
            iss >> name;
            iss >> x, iss >> y, iss >> z;

            mPoints.push_back(cv::Point3d(x*10.0,y*10.0,z*10.0));
            mReferenceName.push_back(QString::fromStdString(name));
        }
    }
}

void Target::clear()
{
    mReferenceName.clear();
    mPoints.clear();
}
