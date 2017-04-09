#include "target.h"

#include <iostream>
#include <math.h>
#include <fstream>

Target::Target()
{
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

    mCameraHeight = 206;
    mCameraHeight2 = 148+40;

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

void Target::getWorldPosition(QStringList list, double &x, double &y)
{
    if(list.size() >= 3){
        int a = list[3].toInt();
        int b = list[1].toInt();
        if(a >= 1 && a <= 11){
            x = (a-4)*100.0;
        }else{
            x = 0.0;
        }

        if(b >= 1 && b <= 7){
            y = b*100.0;
        }else{
            y = 0.0;
        }
    }else{
        x = y = 0.0;
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

void Target::getPoint(QString name, double &x, double &y)
{
    size_t id = 0;
    if(name.size() >= 2){
        QChar O = name.at(0);
        int p = (int)name.at(1).toLatin1()-48;
        if((O == 'R' || O == 'L') && p >= 1 && p <= 4){ //
            id += p;
            if(O == 'R'){
                id += 4;
            }
        }else{
            int a = (int)name.at(0).toLatin1()-48;
            int b = (int)name.at(1).toLatin1()-48;
            if(a == b ){
                if(a == 0){
                    id = 10;
                }else if(a == 1){
                    id = 11;
                }else if(a == 2){
                    id = 12;
                }
            }else if(b >= 0 && b <= 7){
                x = b-4;
                y = a-1;
            }
        }
    }
    double z;
    getPoint(id,x,y,z);
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

void Target::getOrienation(QString name, cv::Point3d &point, double &WorldX, double &WorldZ)
{
    QStringList list = name.split("_");

    getWorldPosition(list[list.size()-2].split(" "),WorldX,WorldZ);

    double pointX, pointY;
    getPoint(list[list.size()-2],pointX,pointY);
    pointY = mFHeight - pointY;
    pointX -= WorldX;

    point.x = calcAngle(pointX,WorldZ);
    point.y = calcAngle(pointY,WorldZ);

    point.z = 0.0;
}
