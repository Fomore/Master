#include "neigungswinkel.h"

#include <iostream>
#include <math.h>

Neigungswinkel::Neigungswinkel()
{

}

Neigungswinkel::~Neigungswinkel()
{

}

void Neigungswinkel::calculate(cv::RotatedRect ellipse){
    //immer ellipse.size.height >= ellipse.size.width
    double a = ellipse.size.width;
    double b = ellipse.size.height;
    double alpha = acos(a/b)/M_PI*180.0;
    double alpha_1 = alpha*cos(ellipse.angle/180.0*M_PI);
    double alpha_2 = alpha*sin(ellipse.angle/180.0*M_PI);

    std::cout<<"Winkel: "<<ellipse.angle<<" ,Zentrum "<<ellipse.center.x<<"/"<<ellipse.center.y<<" ,Durchmesser: "<<ellipse.size.height<<"/"<<ellipse.size.width
             <<" Neigung: "<<alpha<<"/"<<alpha_1<<"/"<<alpha_2<<std::endl;
}
