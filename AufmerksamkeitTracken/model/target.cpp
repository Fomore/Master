#include "target.h"

Target::Target()
{
    // Punkt Zentrum 0
    mPoint[0][0] = 0.0;
    mPoint[0][1] = 176;

    //Links
    mPoint[1][0] = 128.5;
    mPoint[1][1] = 113.5;

    mPoint[2][0] = 308.5;
    mPoint[2][1] = 113.5;

    mPoint[3][0] = 128.5;
    mPoint[3][1] = 194;

    mPoint[4][0] = 308.5;
    mPoint[4][1] = 194;

    //Rechts
    mPoint[5][0] = 128.5;
    mPoint[5][1] = 113.5;

    mPoint[6][0] = 308.5;
    mPoint[6][1] = 113.5;

    mPoint[7][0] = 128.5;
    mPoint[7][1] = 194;

    mPoint[8][0] = 308.5;
    mPoint[8][1] = 194;
}

void Target::getPoint(QString name, int id, double &x, double &y)
{

}
