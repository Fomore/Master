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
    double ty = cos(M_PI*alpha/180.0)*y-sin(M_PI*alpha/180.0)*z;
    double tz = sin(M_PI*alpha/180.0)*y+cos(M_PI*alpha/180.0)*z;
    y = ty;
    z = tz;
}

void Kreis::rotate_Y(int alpha, double &x, double &y, double &z){
    double tx = cos(M_PI*alpha/180.0)*x+sin(M_PI*alpha/180.0)*z;
    double tz = -sin(M_PI*alpha/180.0)*x+cos(M_PI*alpha/180.0)*z;
    x = tx;
    z = tz;
}

void Kreis::rotate_XY(int alpha_x, int alpha_y, double &x, double &y, double &z){
    double ax = M_PI*alpha_x/180.0;
    double ay = M_PI*alpha_y/180.0;
    double tx = cos(ay)*x+sin(ay)*sin(ax)*y+sin(ay)*cos(ax)*z;
    double ty = cos(ax)*y-sin(ay)*z;
    double tz = -sin(ay)*x+cos(ay)*sin(ax)*y+cos(ax)*cos(ay)*z;
    x=tx;
    y=ty;
    z=tz;
}

void Kreis::rotate_circle(double alpha, int alpha_x, int alpha_y, double &x, double &y, double &z){
    double ax = M_PI*alpha_x/180.0;
    double ay = M_PI*alpha_y/180.0;
    x = mRadius*(sin(alpha)*sin(ax)*sin(ay)+cos(alpha)*cos(ay));
    y = mRadius*sin(alpha)*cos(ax);
    z = mRadius*(sin(alpha)*sin(ax)*cos(ay)-cos(alpha)*sin(ay));
}

cv::Mat Kreis::print(int alpha_x, int alpha_y, int radius, int distanz, int skallierung){
    mRadius = radius/100.0;
    double s = skallierung;
    double step = 700;
    cv::Mat mat = cv::Mat::zeros(cv::Size(600,600), CV_8UC4);

    cv::Vec4b& bgra = mat.at<cv::Vec4b>(mat.rows/2,mat.cols/2);
    bgra[0] = 0;
    bgra[1] = 0;
    bgra[2] = 255;
    bgra[3] = UCHAR_MAX;

    for(double a = 0; a< step*2; a++){
        double pos[3];
        rotate_circle(a/step*M_PI,alpha_x,alpha_y,pos[0],pos[1],pos[2]);
        int i = mat.rows/2+(int)(s*pos[0]/(distanz/10+pos[2]));
        int j = mat.cols/2+(int)(s*pos[1]/(distanz/10+pos[2]));

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

void Kreis::build_Table(){
    cv::Mat matWinkel = cv::Mat::zeros(cv::Size(180,180), CV_8UC4);
    cv::Mat matComp = cv::Mat::zeros(cv::Size(180,180), CV_8UC4);
    for(double  i = -90; i <= 90; i++){
        for(double j = -90; j <= 90; j++){
            double pos_1[3];
            double pos_2[3];
            get_circle(0,pos_1[0],pos_1[1],pos_1[2]);
            get_circle(M_PI/2,pos_2[0],pos_2[1],pos_2[2]);

            rotate_X(i,pos_1[0],pos_1[1],pos_1[2]);
            rotate_Y(j,pos_1[0],pos_1[1],pos_1[2]);

            rotate_X(i,pos_2[0],pos_2[1],pos_2[2]);
            rotate_Y(j,pos_2[0],pos_2[1],pos_2[2]);

        }
    }

}
