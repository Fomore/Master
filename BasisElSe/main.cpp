#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include "LandmarkCoreIncludes.h"
#include <iostream>
#include <math.h>
#include <QString>
#include <QRegExp>
#include "algo/algo.h" //CV_8UC1
//#include "else_algosplit/algo.h" //CV_8UC1
//#include "else_morphsplit/algo.h" //CV_8UC1

cv::Mat convert_rgb(cv::Mat in, int abw = 0){
    if(!(in.type() == CV_8UC3 || in.type() == CV_8UC4)){
        cv::Mat out=cv::Mat::zeros(in.size(),CV_8UC1);
        in.convertTo(out,CV_8UC1);
        cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        return out;
    }

    cv::Mat out=cv::Mat::zeros(in.size(),CV_32FC1);

    for(int i=0;i<in.rows;i++){
        for(int j=0;j<in.cols;j++){
            float blue, green, red;
            if(in.type() == CV_8UC3){
                cv::Vec3b pix = in.at<cv::Vec3b>(i,j);
                blue= (float)pix[0];
                green=(float)pix[1];
                red=(float)pix[2];
            }else if(in.type() == CV_8UC4){
                cv::Vec4b pix = in.at<cv::Vec4b>(i,j);
                blue= (float)pix[0];
                green=(float)pix[1];
                red=(float)pix[2];
                if(pix[3] != 255){
                    blue = green = red = 255.0;
                }
            }

            //float gray_val= (0.24079208*red) + (0.16214176*green) + (0.59706616*blue); //Luma coding in video systems
            float gray_val= (0.299*red) + (0.587*green) + (0.114*blue); //Luma coding in video systems
            //float gray_val= (0.333*red) + (0.333*green) + (0.333*blue);

            if(abw == 1){
                float val_red=abs(gray_val-red);
                float val_green=abs(gray_val-green);
                float val_blue=abs(gray_val-red);

                float maxi=1;

                if(val_red>val_blue && val_red>val_green)
                    maxi = val_red;

                if(val_green>val_blue && val_green>val_red)
                    maxi = val_green;

                if(val_blue>val_green && val_blue>val_red)
                    maxi = val_blue;

                gray_val=gray_val/sqrt(1+maxi);
            }else if(abw == 2){
                gray_val=gray_val*sqrt(gray_val/255.0);
            }else if(abw == 3){
                gray_val=gray_val*pow(gray_val/255.0,2);
            }

            out.at<float>(i,j)=gray_val;
        }
    }

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

bool isEllipse(cv::RotatedRect Ellipse){
    return Ellipse.center.x >= 0 && Ellipse.center.y >= 0 && Ellipse.size.width > 0 && Ellipse.size.height > 0;
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

    for(int i = 0; i < 7; i++){
        for(int j = 0; j < 4; j++)
        cv::imwrite(std::to_string(i)+"_"+std::to_string(j)+"b.png",convert_rgb(cv::imread(mImagePaths[i],-1),j));
    }

    return 0;

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
            bool blob[] = {false, false, false, false};
            cv::RotatedRect ellipse[4];
            //for(int j = 0; j < 4; j++){
            tbb::parallel_for(0, 4, [&](int j){
                if(i+j < mImagePaths.size()){
                    cv::Mat Img = cv::imread(mImagePaths.at(i+j));
                    if(Img.data){
                        cv::Mat ret;
                        cv::resize(Img,ret,cv::Size(0,0),scall,scall);
                        //ellipse[j] = ELSE::run(ret, quality[j],blob[j]);
                        ellipse[j] = ELSE::run(convert_rgb(ret), quality[j],blob[j]);
                    }
                }
            });
            for(int j = 0; j < 4; j++){
                myfile<<quality[j]<<" "<<blob[j]<<" "
                     <<compareEllipse(ellipse[j],mIris[i+j])
                        <<compareEllipse(ellipse[j],mPupils[i+j])
                        <<std::endl;
                if(isEllipse(ellipse[j])){
                    countGes++;
                }
            }
        }
        myfile.close();
        std::cout<<"Gefunden "<<countGes<<" mit "<<QualityGes/countGes<<std::endl;
        myfileGes<<scall<<" Gefunden "<<countGes<<"/"<<mImagePaths.size()<<std::endl;
        if(countGes == 0){
            std::cout<<"Abbruch"<<std::endl;
            break;
        }
    }
    myfileGes.close();
    return a.exec();
}
