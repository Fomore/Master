#include "ellipse.h"

#include <iostream>

Ellipse::Ellipse()
{

}

Ellipse::~Ellipse()
{

}

cv::RotatedRect Ellipse::calculate_Ellipse(cv::Mat image){

    cv::vector<cv::Point> pos;
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            cv::Vec4b& bgra = image.at<cv::Vec4b>(i, j);
            if(bgra[3] != 0){//Schwelenwert für die Ellipse
                pos.push_back(cv::Point(j,i));
            }
        }
    }

    return cv::fitEllipse( cv::Mat(pos) );
}
