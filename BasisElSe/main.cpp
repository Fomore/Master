#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include "LandmarkCoreIncludes.h"
#include <iostream>
#include <math.h>
#include <QString>
#include <QRegExp>
#include "algo/algo.h"

cv::Mat convert_rgb(cv::Mat in){

    //GaussianBlur(in, in, cv::Size(ww,hh), std );

    cv::Mat out=cv::Mat::zeros(in.size(),CV_32FC1);

    for(int i=0;i<in.rows;i++){
        for(int j=0;j<in.cols;j++){
            cv::Vec4b pix = in.at<cv::Vec4b>(i,j);
            float blue= (float)pix[0];
            float green=(float)pix[1];
            float red=(float)pix[2];

            float gray_val= (0.299*red) + (0.587*green) + (0.114*blue);
            //float gray_val= (0.333*red) + (0.333*green) + (0.333*blue);


            float val_red=abs(gray_val-red);
            float val_green=abs(gray_val-green);
            float val_blue=abs(gray_val-red);


            float maxi= 1;//max(val_blue,max(val_green,val_red));

            if(val_red>val_blue && val_red>val_green)
                maxi = val_red;


            if(val_green>val_blue && val_green>val_red)
                maxi = val_green;


            if(val_blue>val_green && val_blue>val_red)
                maxi = val_blue;


            gray_val=gray_val/sqrt(1+maxi);

            //val=val*val;

            //val=1.0/(1.0+val);

            out.at<float>(i,j)=gray_val;
        }
    }



    //GaussianBlur(out, out, cv::Size(ww,hh), std );


    cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_32FC1);
    out.convertTo(out,CV_8UC1);

    return out;
}

cv::Vec4f compareEllipse(cv::RotatedRect Ellipse1, cv::RotatedRect Ellipse2){
    double abstandCenter = sqrt(pow(Ellipse1.center.x-Ellipse2.center.x,2)+pow(Ellipse1.center.y-Ellipse2.center.y,2));
    float angle = Ellipse1.angle-Ellipse2.angle;
    if(angle < 0)
        angle *= -1.0;
    float width = Ellipse1.size.width-Ellipse2.size.width;
    float height = Ellipse1.size.height - Ellipse2.size.height;
    return cv::Vec4f(abstandCenter,angle,width,height);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    std::vector<std::string> mImagePaths;
    std::vector<cv::RotatedRect> mIris;
    std::vector<cv::RotatedRect> mPupils;
    std::vector<cv::Rect> mBoxes;

    std::ifstream file("/home/falko/Uni/Master/workfile.txt");
    std::string line;
    QRegExp ausdruck("[\\(\\)\\,]");
    if(file.is_open()){
        while (std::getline(file, line)){
            std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
            std::string path;
            float Icx,Icy,Ih,Iw,Ia;
            float Pcx,Pcy,Ph,Pw,Pa;
            int Bx,By,Bw,Bh;
            iss >> path;
            iss >> Icx, iss >> Icy, iss >> Iw, iss >> Ih, iss >> Ia;
            iss >> Pcx, iss >> Pcy, iss >> Pw, iss >> Ph, iss >> Pa;
            iss >> Bx, iss >> By, iss >> Bw, iss >> Bh;
            //(x_centre,y_centre),(minor_axis,major_axis),angle
            //ellipse_i ellipse_p box))
            mImagePaths.push_back(path);
            mIris.push_back(cv::RotatedRect(cv::Point2f(Icx,Icy),cv::Size2f(Iw,Ih),Ia));
            mPupils.push_back(cv::RotatedRect(cv::Point2f(Pcx,Pcy),cv::Size2f(Pw,Ph),Pa));
            mBoxes.push_back(cv::Rect(Bx,By,Bw,Bh));
        }
    }


    std::ofstream myfileGes;
    myfileGes.open ("solution/Auge_ges.txt", std::ios::in | std::ios::app);
    for(double scall = 1.0; scall > 0.0; scall -= 0.02){
        std::cout<<"Skalierung "<<scall<<std::endl;
        std::ofstream myfile;
        myfile.open ("solution/Auge_"+std::to_string(scall)+".txt", std::ios::in | std::ios::app);
        size_t countGes = 0;
        double QualityGes = 0.0;
        for(size_t i = 0; i < mImagePaths.size(); i += 4){
            float quality[] = {-1.0, -1.0, -1.0, -1.0};
            cv::RotatedRect ellipse[4];
            tbb::parallel_for(0, 4, [&](int j){
                if(i+j < mImagePaths.size()){
                    cv::Mat Img =  cv::imread(mImagePaths.at(i+j), -1);
                    cv::Mat ret;
                    cv::resize(Img,ret,cv::Size(0,0),scall,scall);
                    if(Img.data){
                        ellipse[j] = ELSE::run(ret, quality[j]);
                    }
                }
            });
            for(int j = 0; j < 4; j++){
                myfile<<quality[j]<<" "
                     <<compareEllipse(ellipse[j],mIris[i+j])
                        <<compareEllipse(ellipse[j],mPupils[i+j])
                        <<std::endl;
                if(quality[j] > -150){
                    countGes++;
                    QualityGes += quality[j];
                }
            }
        }
        myfile.close();
        std::cout<<"Gefunden "<<countGes<<" mit "<<QualityGes/countGes<<std::endl;
        myfileGes<<scall<<" Gefunden "<<countGes<<" mit "<<QualityGes/countGes<<std::endl;
        if(countGes == 0){
            break;
        }
    }
    myfileGes.close();
    return a.exec();
}
