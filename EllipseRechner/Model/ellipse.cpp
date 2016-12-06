#include "ellipse.h"

#include <iostream>
#include <math.h>

Ellipse::Ellipse()
{

}

Ellipse::~Ellipse()
{

}

cv::RotatedRect Ellipse::calculate_Ellipse(cv::Mat image){

    std::vector<cv::Point> pos;
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            cv::Vec4b& bgra = image.at<cv::Vec4b>(i, j);
            if(bgra[3] != 0){//Schwelenwert für die Ellipse
                pos.push_back(cv::Point(j,i));
            }
        }
    }

    cv::RotatedRect ret_ellipse = cv::fitEllipse( cv::Mat(pos) );
    std::cout<<"Ellipse: "<<ret_ellipse.angle<<" - "<<ret_ellipse.size.height<<" - "<<ret_ellipse.size.width<<std::endl;
    double ax = cos(ret_ellipse.angle/180.0*M_PI)*ret_ellipse.size.height/2;
    double ay = sin(ret_ellipse.angle/180.0*M_PI)*ret_ellipse.size.height/2;
    double bx = cos((ret_ellipse.angle+90)/180.0*M_PI)*ret_ellipse.size.width/2;
    double by = sin((ret_ellipse.angle+90)/180.0*M_PI)*ret_ellipse.size.width/2;

    std::cout<<"   Scheitelpunkt: "<<ax<<" / "<< ay<<" - "<<bx<<" / "<<by<<std::endl;

    return ret_ellipse;
}
