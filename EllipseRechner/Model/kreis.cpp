#include "kreis.h"

#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>

Kreis::Kreis()
{
    mRadius = 1;
}

Kreis::~Kreis()
{

}

void Kreis::get_circle(double alpha, double &x, double &y, double &z){
    x = mRadius*cos(alpha);
    y = mRadius*sin(alpha);
    z = 0;
}

void Kreis::rotate_X(int alpha, double &x, double &y, double &z){
    double ty = cos(M_PI*alpha/180)*y-sin(M_PI*alpha/180)*z;
    double tz = sin(M_PI*alpha/180)*y+cos(M_PI*alpha/180)*z;
    y = ty;
    z = tz;
}

void Kreis::rotate_Y(int alpha, double &x, double &y, double &z){
    double tx = cos(M_PI*alpha/180)*x+sin(M_PI*alpha/180)*z;
    double tz = -sin(M_PI*alpha/180)*x+cos(M_PI*alpha/180)*z;
    x = tx;
    z = tz;
}

cv::Mat Kreis::print(int alpha_x, int alpha_y){
    double s = 250;//Skallierung
    double step = 700;
    cv::Mat mat = cv::Mat::zeros(cv::Size(600,600), CV_8UC4);
    for(double a = 0; a< step*2; a++){
        double pos[3];
        get_circle(a/step*M_PI,pos[0],pos[1],pos[2]);
        rotate_X(alpha_x,pos[0],pos[1],pos[2]);
        rotate_Y(alpha_y,pos[0],pos[1],pos[2]);
        int i = mat.rows/2+(int)(s*(pos[0]/(pos[2]+3))+0.5);
        int j = mat.cols/2+(int)(s*(pos[1]/(pos[2]+3))+0.5);
        if(i >=0 && i < mat.rows && j >= 0 && j < mat.cols){
            cv::Vec4b& bgra = mat.at<cv::Vec4b>(i, j);
            bgra[0] = 0;
            bgra[1] = 255;
            bgra[2] = 0;
            bgra[3] = UCHAR_MAX;
        }
    }
    cv::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::imwrite("alpha.png", mat, compression_params);

    return mat;
}
