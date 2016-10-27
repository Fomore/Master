#ifndef KREIS_H
#define KREIS_H

#include <opencv2/opencv.hpp>

class Kreis
{
private:
    double mRadius;
    //Berechent die Kreiskoordinaten des Kreisen zum Winkel alpha
    void get_circle(double alpha, double &x, double &y, double &z);
    void rotate_X(int alpha, double &x, double &y, double &z);
    void rotate_Y(int alpha, double &x, double &y, double &z);

public:
    Kreis();
    ~Kreis();
    cv::Mat print(int alpha_x, int alpha_y);
    bool busy;
};

#endif // KREIS_H
