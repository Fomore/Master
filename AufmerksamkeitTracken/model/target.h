#ifndef TARGET_H
#define TARGET_H

#include <QString>

class Target
{
private:
    //Koordinaten im Weltkoordinatensystem
    double mPoint[9][2];
public:
    Target();
    void getPoint(QString name, int id, double &x, double &y);
};

#endif // TARGET_H
