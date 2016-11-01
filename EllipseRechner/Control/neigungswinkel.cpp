#include "neigungswinkel.h"

#include <iostream>
#include <math.h>

Neigungswinkel::Neigungswinkel()
{

    for(int i = 0; i < 4*7; i++){
        fehler[i][0]=0;
        fehler[i][1]=0;
    }
}

Neigungswinkel::~Neigungswinkel()
{

}

void Neigungswinkel::calculate(cv::RotatedRect ellipse, double x, double y){
    //immer ellipse.size.height >= ellipse.size.width

    double r[4];

    r[0] = sin(M_PI*ellipse.angle/180.0)*ellipse.size.width;
    r[1] = cos(M_PI*ellipse.angle/180.0)*ellipse.size.width;
    r[2] = sin(M_PI*ellipse.angle/180.0)*ellipse.size.height;
    r[3] = cos(M_PI*ellipse.angle/180.0)*ellipse.size.height;

    std::cout<<"Winkel: "<<ellipse.angle<<" ,Zentrum "<<ellipse.center.x<<"/"<<ellipse.center.y<<" ,Durchmesser: "<<ellipse.size.height<<"/"<<ellipse.size.width
             <<"\n   Strecken r0:"<<r[0]<<" r1: "<<r[1]<<" r2: "<<r[2]<<" r3: "<<r[3]<<std::endl;

    double ret[3];

    for(int i = 0; i < 4; i++){
        double alpha1 = acos(r[i]/165.437)/M_PI*180.0;
        fehler[i*7][0]+= (x-alpha1)*(x-alpha1);
        fehler[i*7][1]+= (y-alpha1)*(y-alpha1);
        double alpha2 = acos(r[i]/ellipse.size.width)/M_PI*180.0;
        fehler[i*7+1][0]+= (x-alpha2)*(x-alpha2);
        fehler[i*7+1][1]+= (y-alpha2)*(y-alpha2);
        double alpha3 = acos(r[i]/ellipse.size.height)/M_PI*180.0;
        fehler[i*7+2][0]+= (x-alpha3)*(x-alpha3);
        fehler[i*7+2][1]+= (y-alpha3)*(y-alpha3);
        double alpha4 = acos(r[i]/r[0])/M_PI*180.0;
        fehler[i*7+3][0]+= (x-alpha4)*(x-alpha4);
        fehler[i*7+3][1]+= (y-alpha4)*(y-alpha4);
        double alpha5 = acos(r[i]/r[1])/M_PI*180.0;
        fehler[i*7+4][0]+= (x-alpha5)*(x-alpha5);
        fehler[i*7+4][1]+= (y-alpha5)*(y-alpha5);
        double alpha6 = acos(r[i]/r[2])/M_PI*180.0;
        fehler[i*7+5][0]+= (x-alpha6)*(x-alpha6);
        fehler[i*7+5][1]+= (y-alpha6)*(y-alpha6);
        double alpha7 = acos(r[i]/r[3])/M_PI*180.0;
        fehler[i*7+6][0]+= (x-alpha7)*(x-alpha7);
        fehler[i*7+6][1]+= (y-alpha7)*(y-alpha7);
        if(i == 0){
            ret[0]=alpha3;
            ret[1]=alpha6;
        }else if(i ==1){
            ret[2]=alpha7;
        }

        std::cout<<"    Neigung r"<<i<<": "<<alpha1<<" / "<<alpha2<<" / "<<alpha3<<" / "<<alpha4<<" / "<<alpha5<<" / "<<alpha6<<" / "<<alpha7<<std::endl;
        std::cout<<"    Fehler X: "<<fehler[i*7][0]<<" / "<<fehler[i*7+1][0]<<" / "<<fehler[i*7+2][0]<<" / "<<fehler[i*7+3][0]<<" / "<<fehler[i*7+4][0]<<" / "<<fehler[i*7+5][0]<<" / "<<fehler[i*7+6][0]<<std::endl;
        std::cout<<"    Fehler Y: "<<fehler[i*7][1]<<" / "<<fehler[i*7+1][1]<<" / "<<fehler[i*7+2][1]<<" / "<<fehler[i*7+3][1]<<" / "<<fehler[i*7+4][1]<<" / "<<fehler[i*7+5][1]<<" / "<<fehler[i*7+6][1]<<std::endl;
    }
    std::cout<<"Ergebnis: "<<ret[0]<<" / "<<ret[1]<<" / "<<ret[2]<<std::endl;
}
