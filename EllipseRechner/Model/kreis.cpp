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
        int i = mat.rows/2+(int)(s*pos[0]/(distanz+pos[2]));
        int j = mat.cols/2+(int)(s*pos[1]/(distanz+pos[2]));

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
    cv::Mat matComp = cv::Mat::zeros(cv::Size(181,181), CV_8UC4);
    double distanz = 100;
    int s = 1000000;

    cv::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    for(int  x = -90; x <= 90; x++){
        for(int y = -90; y <= 90; y++){
            cv::Mat matImg = cv::Mat::zeros(cv::Size(420,420), CV_8UC4);
            double step = 700;

            cv::vector<cv::Point> point;

            for(double a = 0; a < step*2; a++){
                double pos[3];
                rotate_circle(a/step*M_PI,x,y,pos[0],pos[1],pos[2]);
                int i = (s*pos[0]/(distanz+pos[2]));
                int j = (s*pos[1]/(distanz+pos[2]));
                point.push_back(cv::Point(j,i));
                cv::Vec4b& bgra = matImg.at<cv::Vec4b>(210+i/50, 210+j/50);
                bgra[0] = 0;
                bgra[1] = 255;
                bgra[2] = 0;
                bgra[3] = UCHAR_MAX;
            }
            cv::RotatedRect ellipse = cv::fitEllipse( cv::Mat(point) );
            /*
            std::cout<<x<<"/"<<y<<" : "<<ellipse.angle<<" "<<ellipse.size.height<<" "<<ellipse.size.width<<std::endl;

            cv::RotatedRect elli(cv::Point2f(210,210), cv::Size2f(ellipse.size.width/50,ellipse.size.height/50), ellipse.angle);
            cv::ellipse( matImg, elli, cv::Scalar(255,0,255,255), 1,1 );

            std::string titel = "Beta_"+std::to_string(x)+"_"+std::to_string(y)+".png";
            cv::imwrite(titel, matImg, compression_params);
*/
            int r = ellipse.angle/180.0*255;
            if(ellipse.angle == 180){
                r = 0;
            }
            int g = ellipse.size.height/20000.0*255;
            int b = ellipse.size.width /20000.0*255;

            if(r >= 0 && r <= 255
                    && g >= 0 && g <= 255
                    && b >= 0 && b <= 255){
                cv::Vec4b& bgra1 = matComp.at<cv::Vec4b>(x+90,y+90);
                bgra1[0] = b;
                bgra1[1] = g;
                bgra1[2] = r;
                bgra1[3] = UCHAR_MAX;
            }else{
                std::cout<<"Fehler: "<<ellipse.angle<<" "<<ellipse.size.height<<" "<<ellipse.size.width<<std::endl;
            }
        }
    }
    cv::imwrite("Tabelle.png", matComp, compression_params);
}
