#include "target.h"

#include <iostream>
#include <math.h>
#include <fstream>

Target::Target()
{
    // Punkt Zentrum 0
    mPoint[0][0] = 0.0;
    mPoint[0][1] = 176;

    //Links
    mPoint[1][0] = -128.5; //1
    mPoint[1][1] = 194;

    mPoint[2][0] = -308.5; //2
    mPoint[2][1] = 194;

    mPoint[3][0] = -308.5; //3
    mPoint[3][1] = 113.5;

    mPoint[4][0] = -128.5; //4
    mPoint[4][1] = 113.5;

    //Rechts
    mPoint[5][0] = 128.5; //1
    mPoint[5][1] = 194;

    mPoint[6][0] = 308.5; //2
    mPoint[6][1] = 194;

    mPoint[7][0] = 308.5; //3
    mPoint[7][1] = 113.5;

    mPoint[8][0] = 128.5; //4
    mPoint[8][1] = 113.5;

    mFHeight = 170;
    mFHeight = 166;
    mCameraHeight = 206;
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
        if((O == 'R' || O == 'L') && p >= 1 && p <= 4){
            id += p;
            if(O == 'R'){
                id += 4;
            }
        }
    }
    getPoint(id,x,y);
}

void Target::getPoint(size_t id, double &x, double &y)
{
    if(id < 9){
        x = mPoint[id][0];
        y = mPoint[id][1];
    }
}

void Target::getOrienation(QString name, cv::Point3d &point)
{
    QStringList list = name.split("_");

    double worldX,worldY,worldZ;
    worldY = mFHeight;
    getWorldPosition(list[list.size()-2].split(" "),worldX,worldZ);

    double pointX, pointY;
    getPoint(list[list.size()-2],pointX,pointY);
    pointY -= worldY;
    pointX -= worldX;

    point.x = calcAngle(pointX,worldZ);
    point.y = calcAngle(pointY,worldZ);

    point.z = 0.0;

    std::ofstream myfile;
    myfile.open ("./data/Messpunkte.txt", std::ios::in | std::ios::app);
    myfile <<"["<<worldX<<", "<<worldZ<<"] -> "<<point<<std::endl;
    myfile.close();
    std::cout<<"Orginal: ["<<worldX<<", "<<worldZ<<"] "<<point<<std::endl;
}
