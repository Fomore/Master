#include "target.h"

#include <iostream>
#include <math.h>
#include <fstream>

Target::Target(Camera *cam)
{
    mKamera = cam;

    // Punkt Zentrum 0
    mPoint[0][0] = 0.0;
    mPoint[0][1] = 176;
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
    mPoint2[1][1] = 70;
    mPoint2[1][2] = 0;

    mPoint2[2][0] = 0;
    mPoint2[2][1] = 40;
    mPoint2[2][2] = 50;
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
            x = (b-4)*100.0;
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

double Target::calcAngle(double ge, double an)
{
    if(an == 0.0){
        return M_PI;
    }else{
        if(ge == 0.0){
            return 0;
        }else{
            return atan(ge/an);
        }
    }
}

cv::Point3d Target::calcAngle(double x, double y, double z)
{
    double bet = sqrt(x*x+y*y+z*z);
    std::cout<<x<<" "<<y<<" "<<z<<": "<<bet<<std::endl;
    return cv::Point3d(acos(x/bet),acos(y/bet),acos(z/bet));
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
            if(a == b ){
                if(a == 0){
                    id = 10;
                }else if(a == 1){
                    id = 11;
                }else if(a == 2){
                    id = 12;
                }
            }else if(b >= 0 && b <= 7){
                Point.x = (b-4)*100;
                Point.y = (a-1)*100;
            }
        }
    }
    getPoint(id,Point.x,Point.y,Point.z);
}

void Target::getPoint(size_t id, double &x, double &y, double &z)
{
    if(id < 9){
        x = mPoint[id][0];
        y = mPoint[id][1];
        z = mPoint[id][2];
    }else if(id >= 10 && id <13){
        x = mPoint2[id-10][0];
        y = mPoint2[id-10][1];
        z = mPoint2[id-10][2];
    }
}

void Target::getOrienation(QString name, cv::Point3d &WAngle, cv::Point3d &WPosition)
{
    QRegExp rx("(\\ |\\_)");
    QStringList list = name.split(rx);

    getWorldPosition(list,WPosition.x, WPosition.y, WPosition.z);

    cv::Point3d point;
    getPoint(list[1],point);

    WAngle.x = calcAngle(point.x-WPosition.x,WPosition.z-point.z);
    WAngle.y = calcAngle(WPosition.y-point.y,WPosition.z-point.z);
    WAngle.z = 0.0;

    std::cout<<name.toStdString()<<": "<<WPosition<<point<<WAngle
             <<calcAngle(WPosition.x-point.x, WPosition.y-point.y, WPosition.z-point.z)<<std::endl;
    cv::Vec3d pos = mKamera->rotateToCamera(point);
    cv::Vec3d pnt = mKamera->rotateToCamera(WPosition);
    std::cout<<pnt<<pos<<calcAngle(pos[0]-pnt[0],pnt[1]-pos[1],pnt[2]-pos[2])<<std::endl;
}
