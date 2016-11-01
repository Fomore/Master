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
    void rotate_XY(int alpha_x, int alpha_y, double &x, double &y, double &z);
    void rotate_circle(double alpha, int alpha_x, int alpha_y, double &x, double &y, double &z);
public:
    Kreis();
    ~Kreis();
    cv::Mat print(int alpha_x, int alpha_y, int radius);
    void build_Table();
};

#endif // KREIS_H
