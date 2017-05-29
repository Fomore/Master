#include "target.h"

#include <iostream>
#include <math.h>
#include <fstream>
Target::Target()
{
    // Punkt Zentrum 0
    mPoint[0][0] = 0.0;
    mPoint[0][1] = 170;
    mPoint[0][2] = -41;

    //Links
    mPoint[1][0] = -128.5; //1
    mPoint[1][1] = 194;
    mPoint[1][2] = 0;

    mPoint[2][0] = -308.5; //2
    mPoint[2][1] = 194;
    mPoint[2][2] = 0;

    mPoint[3][0] = -308.5; //3
    mPoint[3][1] = 113.5;
    mPoint[3][2] = 0;

    mPoint[4][0] = -128.5; //4
    mPoint[4][1] = 113.5;
    mPoint[4][2] = 0;

    //Rechts
    mPoint[5][0] = 128.5; //1
    mPoint[5][1] = 194;
    mPoint[5][2] = 0;

    mPoint[6][0] = 308.5; //2
    mPoint[6][1] = 194;
    mPoint[6][2] = 0;

    mPoint[7][0] = 308.5; //3
    mPoint[7][1] = 113.5;
    mPoint[7][2] = 0;

    mPoint[8][0] = 128.5; //4
    mPoint[8][1] = 113.5;
    mPoint[8][2] = 0;

    mFHeight = 170;
    mTHeight = 166;

    mPoint2[0][0] = 0;
    mPoint2[0][1] = 148+40;
    mPoint2[0][2] = 0;


    mPoint2[1][0] = 0;
    mPoint2[1][1] = (70+40);
    mPoint2[1][2] = 0;

    mPoint2[2][0] = 0;
    mPoint2[2][1] = 40;
    mPoint2[2][2] = 50;

    //Tafel bei 23100601S1: 987.072, -110.714, 247.096
}

void Target::getWorldPosition(QStringList list, double &x, double &y, double &z)
{
    if(list.size() >= 4){
        int a = list[list.size()-4].toInt();
        int b = list[list.size()-2].toInt();
        if(a >= 1 && a <= 12){
            z = a*100.0;
        }else{
            z = 0.0;
        }

        if(b >= 0 && b <= 7){
            x = (b-4)*100.0;//Innen
            //x = (4-b)*100.0;//Außen
        }else{
            x = 0.0;
        }

        if(list[list.size()-1] == "Thomas"){
            y = mTHeight;
        }else if(list[list.size()-1] == "Falko"){
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

void Target::getPoint(QString Name, cv::Point3d &Point)
{
    size_t id = 0;
    if(Name.size() >= 2){
        QChar O = Name.at(0);
        int p = (int)Name.at(1).toLatin1()-48;
        if((O == 'R' || O == 'L') && p >= 1 && p <= 4){ //
            id += p;
            if(O == 'R'){
                id += 4;
            }
        }else{
            int a = (int)Name.at(0).toLatin1()-48;
            int b = (int)Name.at(1).toLatin1()-48;

            if(a == b && a < 3){
                if(a == 0){
                    id = 10;
                }else if(a == 1){
                    id = 11;
                }else if(a == 2){
                    id = 12;
                }
            }else if((a == 3 || a == 9) && b >= 0 && b <= 7){
                Point.x = (4-b)*1000;
                Point.z = (a-1)*1000;
                return;
            }
        }
    }
    getPoint(id,Point.x,Point.y,Point.z);
}

void Target::getPoint(size_t id, double &x, double &y, double &z)
{
    if(id < 9){
        x = mPoint[id][0]*10;
        y = mPoint[id][1]*10;
        z = mPoint[id][2]*10;
    }else if(id >= 10 && id <13){
        x = mPoint2[id-10][0]*10;
        y = mPoint2[id-10][1]*10;
        z = mPoint2[id-10][2]*10;
    }else{
        x=y=z=0.0;
    }
}

void Target::getOrienation(QString name, cv::Point2d &WAngle, cv::Point3d &WPosition, cv::Point2d &RAngle, cv::Point3d &Target)
{
    QRegExp rx("(\\ |\\_)");
    QStringList list = name.split(rx);

    getWorldPosition(list,WPosition.x, WPosition.y, WPosition.z);

    getPoint(list[2],Target);

    WAngle = calcAngle(WPosition.x-Target.x,WPosition.y-Target.y,WPosition.z-Target.z);

    cv::Vec3d pos = mKamera->rotateToCamera(Target);
    cv::Vec3d pnt = mKamera->rotateToCamera(WPosition);

    RAngle = calcAngle(pnt[0]-pos[0],pnt[1]-pos[1],pnt[2]-pos[2]);
}
