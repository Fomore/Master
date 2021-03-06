#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <QString>
#include <QRegExp>
#include "LandmarkCoreIncludes.h"

#include "algo/algo.h" //CV_8UC1
//#include "else_algosplit/algo.h" //CV_8UC1
//#include "else_morphsplit/algo.h" //CV_8UC1

cv::Mat toGray(cv::Mat Image, int Norm){
    if(!(Norm > 0 && Norm <= 5)){
        cv::Mat out;
        cv::cvtColor(Image,out , cv::COLOR_BGR2GRAY);
        out.convertTo(out,CV_8UC1);
        cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        return out;
    }else if(Norm == 3){
        cv::Mat grayImage=cv::Mat::zeros(Image.size(),CV_8UC1);
        for(int i=0;i<Image.rows;i++){
            for(int j=0;j<Image.cols;j++){
                if(Image.type() == CV_8UC4){
                    cv::Vec4b pix = Image.at<cv::Vec4b>(i,j);
                    if(pix[3] == 255){
                        grayImage.at<uchar>(i,j) = std::max(pix[0],std::max(pix[1],pix[2]));
                    }else{
                        grayImage.at<uchar>(i,j) = 255;
                    }
                }else if(Image.type() == CV_8UC3){
                    cv::Vec3b pix = Image.at<cv::Vec3b>(i,j);
                    grayImage.at<uchar>(i,j) = std::max(pix[0],std::max(pix[1],pix[2]));
                }
            }
        }
        cv::normalize(grayImage, grayImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        return grayImage;
    }else if(Norm == 5){
    cv::Mat grayImage=cv::Mat::zeros(Image.size(),CV_8UC1);
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            if(Image.type() == CV_8UC4){
                cv::Vec4b pix = Image.at<cv::Vec4b>(i,j);
                if(pix[3] == 255){
                    grayImage.at<uchar>(i,j) = std::min(pix[0],std::min(pix[1],pix[2]));
                }else{
                    grayImage.at<uchar>(i,j) = 255;
                }
            }else if(Image.type() == CV_8UC3){
                cv::Vec3b pix = Image.at<cv::Vec3b>(i,j);
                grayImage.at<uchar>(i,j) = std::min(pix[0],std::min(pix[1],pix[2]));
            }
        }
    }
    cv::normalize(grayImage, grayImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    return grayImage;
}
//    Parameter bestimmen
    int maxRGB[3] = {0,0,0};
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            if(Image.type() == CV_8UC4){
                cv::Vec4b pix = Image.at<cv::Vec4b>(i,j);
                if(pix[3] == 255){
                    maxRGB[0] = std::max(maxRGB[0],(int)pix[0]);
                    maxRGB[1] = std::max(maxRGB[1],(int)pix[1]);
                    maxRGB[2] = std::max(maxRGB[2],(int)pix[2]);
                }
            }else if(Image.type() == CV_8UC3){
                cv::Vec3b pix = Image.at<cv::Vec3b>(i,j);
                maxRGB[0] = std::max(maxRGB[0],(int)pix[0]);
                maxRGB[1] = std::max(maxRGB[1],(int)pix[1]);
                maxRGB[2] = std::max(maxRGB[2],(int)pix[2]);
            }
        }
    }
    //double Val[] = {log(255.0)/log(maxRGB[0]), log(255.0)/log(maxRGB[1]), log(255.0)/log(maxRGB[2])};
    double Val[] = {1.0/2.2, 1.0/2.2, 1.0/2.2};
    if(Norm == 1){
        Val[0] = log(255.0)/log(maxRGB[0]);
        Val[1] = log(255.0)/log(maxRGB[1]);
        Val[2] = log(255.0)/log(maxRGB[2]);
    }else if(Norm == 4){
        Val[0] = 2.0;
        Val[1] = 2.0;
        Val[2] = 2.0;
    }

//    std::cout<<Val[0]<<" "<<Val[1]<<" "<<Val[2]<<std::endl;
//    To Gray
    int minGray = 255, maxGray = 0;
    cv::Mat grayImage=cv::Mat::zeros(Image.size(),CV_8UC1);
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            if(Image.type() == CV_8UC4){
                cv::Vec4b pix = Image.at<cv::Vec4b>(i,j);
                if(pix[3] == 255){
                    double faktor = (pow(pix[0]/255.0,Val[0]) + pow(pix[1]/255.0,Val[1]) + pow(pix[2]/255.0,Val[2]))/3.0;
                    int colore = std::max(0,std::min(255,(int)(255.0*faktor + 0.5)));

                    maxGray = std::max(maxGray,colore);
                    minGray = std::min(minGray,colore);
                    grayImage.at<uchar>(i,j) = colore;
                }else{
                    grayImage.at<uchar>(i,j) = 255;
                }
            }else if(Image.type() == CV_8UC3){
                cv::Vec3b pix = Image.at<cv::Vec3b>(i,j);
                double faktor = (pow(pix[0]/255.0,Val[0]) + pow(pix[1]/255.0,Val[1]) + pow(pix[2]/255.0,Val[2]))/3.0;
                int colore = std::max(0,std::min(255,(int)(255.0*faktor + 0.5)));

                maxGray = std::max(maxGray,colore);
                minGray = std::min(minGray,colore);
                grayImage.at<uchar>(i,j) = colore;
            }
        }
    }

    cv::Mat gaus;
    cv::GaussianBlur(grayImage, gaus, cv::Size(7,7), 1.5, 1.5);

    minGray = 255, maxGray = 0;
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            if(!(Image.type() == CV_8UC4 && Image.at<cv::Vec4b>(i,j)[3] != 255)){
                int colore = gaus.at<uchar>(i,j);
                maxGray = std::max(maxGray,colore);
                minGray = std::min(minGray,colore);
            }
        }
    }

    cv::Mat out=cv::Mat::zeros(Image.size(),CV_8UC1);
    double a = (255.0+minGray)/maxGray;
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            out.at<uchar>(i,j)= std::min(255,std::max((int)(grayImage.at<uchar>(i,j)*a-minGray+0.5),0));
        }
    }
    return out;
}

cv::Vec4f compareEllipse(cv::RotatedRect Ellipse1, cv::RotatedRect Ellipse2, double scall, cv::Rect Box){
    double x = Box.x;
    double y = Box.y;
    double abstandCenter = sqrt(pow(Ellipse1.center.x/scall - (Ellipse2.center.x-x),2)
                               +pow(Ellipse1.center.y/scall - (Ellipse2.center.y-y),2));
//    std::cout<<Ellipse1.center<<Ellipse2.center<<" "<<x<<"/"<<y<<" "<<abstandCenter<<std::endl;
    float angle = std::abs(Ellipse1.angle-Ellipse2.angle);
    if(angle >= 180.0f)
        angle -= 180.0f;
    float width = std::abs(Ellipse1.size.width/scall - Ellipse2.size.width);
    float height = std::abs(Ellipse1.size.height/scall - Ellipse2.size.height);
    return cv::Vec4f(abstandCenter,angle,width,height);
}

bool isEllipse(cv::RotatedRect Ellipse){
    return Ellipse.center.x >= 0 && Ellipse.center.y >= 0 && Ellipse.size.width > 0 && Ellipse.size.height > 0;
}


bool myfunction (double i,double j) { return (i<j); }

cv::Vec3d Auswertung(std::vector<bool> Blob, std::vector<double> Values, int Typ){
    if(Blob.size() != Values.size()){
        std::cout<<"Feher: Blob="<<Blob.size()<<" Values="<<Values.size()<<std::endl;
        return cv::Vec3d(0,0,0);
    }
    size_t count = 0;
    double summe = 0.0;
    std::vector<double> val;
    for(size_t i = 0; i < Values.size(); i++){
        if(Typ == 3 || (Typ == 1 && Blob[i]) || (Typ == 2 && !Blob[i])){
            count++;
            summe += Values[i];
            val.push_back(Values[i]);
        }
    }

    if(val.size() > 0){
        std::sort(val.begin(), val.end(),myfunction);
        size_t q = val.size()/4;
        return cv::Vec3d(val[q], summe/((double) count), val[q*3]);
    }else{
        return cv::Vec3d(0,0,0);
    }
}

void AuswertungFile(){
    std::ofstream myfileGes;
    myfileGes.open ("Auswertung_Radius_18.txt", std::ios::in | std::ios::app);
    std::vector<cv::String> FileNames;
//    cv::glob("/home/falko/Uni/Master/build-BasisOpenFaceEye-Desktop-Debug/solution/*0.txt",FileNames,false);
    cv::glob("solution/*_18.txt",FileNames,false);
    for(size_t i = 0; i < FileNames.size(); i++){
        std::ifstream file(FileNames[i]);
        std::string line;

        size_t p = FileNames[i].size();
        std::string ScalString = FileNames[i].substr(p-15,8);
        double scall = std::stod(ScalString);
        std::cout<<"Berechne "<<scall<<" "<<FileNames[i]<<" "<<ScalString<<std::endl;

        std::vector<std::vector<double>> Values;
        for(int j = 0; j < 9; j++){
            Values.push_back(std::vector<double>());
        }
        std::vector<bool> Blob;
        size_t CountBlob = 0;
        QRegExp ausdruck("[\\[\\]\\,]");
        bool ElSe = true;
        if(file.is_open()){
            while (std::getline(file, line)){
                std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
                double val;
                bool blob;
                if(ElSe){
                    iss >> val;
                    Values[0].push_back(val);
                    iss >> blob;
                    Blob.push_back(blob);
                    if(blob){
                        CountBlob++;
                    }
                }else{
                    Values[0].push_back(0);
                    Blob.push_back(true);
                    CountBlob++;
                }
                for(int j = 1; j < 9; j++){
                    iss >> val;
                    Values[j].push_back(val);
                }
            }
        }
        file.close();
        myfileGes<<scall;
        for(size_t j = 0; j < Values.size(); j++){
            myfileGes<<Auswertung(Blob,Values[j],1)<<Auswertung(Blob,Values[j],2);
        }
        double ant = (double)CountBlob/(double)Blob.size();
        myfileGes<<" "<<std::to_string(ant)<<std::endl;
    }
    myfileGes.close();
}

cv::Rect resizeRect(const cv::Rect &Box){
    return cv::Rect(Box.x-3,Box.y-3,Box.width+6,Box.height+6);
}

cv::Mat getFarbKarte(){
    cv::Mat ret(cv::Size(201,201), CV_8UC3, cv::Scalar(255,255,255));
    int r = 0, g= 0, b= 0;
    for(int i = 0; i < 8; i++){
        for(int j = 0; j < 8; j++){
            cv::rectangle(ret,cv::Rect(1+i*25,1+j*25,24,24),cv::Scalar(85*r,85*g,85*b),-1);
            r++;
            if(r >= 4){
                g++;
                r = 0;
            }
            if(g >= 4){
                b++;
                g=0;
            }
        }
    }
    return ret;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

//    AuswertungFile();
//    return 0;

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

    for(int i = 0; i <5; i++){
        cv::Mat Img = cv::imread("/home/falko/Uni/Master/Bilder/Learn/lena.jpg",-1);
        cv::imwrite("Lena"+std::to_string(i)+".png",toGray(Img,i));//getFarbKarte(),i));
//        cv::imwrite("Auge_"+std::to_string(i)+"Gray.png",toGray(Img(resizeRect(mBoxes[0])),i));
    }
    cv::imwrite("Farbtafel2.png",getFarbKarte());
    return 0;

    int Radius[] = {8,11,18};
    for(int Variante = 0; Variante < 5; Variante++){
    for(int r = 1; r < 3; r++){
        //      int Radius = 1;
        std::ofstream myfileGes;
        myfileGes.open ("solution/Auge_ges_"+std::to_string(Variante)
                        +"_"+std::to_string(Radius[r])+".txt", std::ios::in | std::ios::app);
        for(double scall = 1.0; scall > 0.0; scall -= 0.03){
            //        double scall = 1.0;
            size_t gesWidth = 0;
            size_t gesHeight = 0;
            std::cout<<"Skalierung "<<scall<<" "<<Variante<<" "<<Radius[r]<<std::endl;
            std::ofstream myfile;
            //        myfile.open ("solution/Auge_"+std::to_string(scall)+"_V"+std::to_string(Variante)+".txt", std::ios::in | std::ios::app);
            myfile.open ("solution/Auge_"+std::to_string(scall)
                         +"_"+std::to_string(Radius[r])
                         +"_"+std::to_string(Variante)+".txt", std::ios::in | std::ios::app);
            size_t countGes = 0;
            double QualityGes = 0.0;
            for(size_t i = 0; i < mImagePaths.size(); i += 4){
                float quality[] = {-1.0, -1.0, -1.0, -1.0};
                bool blob[] = {false, false, false, false};
                cv::RotatedRect ellipse[4];
                int retW[] = {0,0,0,0};
                int retH[] = {0,0,0,0};
                //for(int j = 0; j < 4; j++){
                tbb::parallel_for(0, 4, [&](int j){
                    if(i+j < mImagePaths.size()){
                        cv::Mat Img = cv::imread(mImagePaths.at(i+j));
                        if(Img.data && mBoxes[i+j].height*scall > 0.5){
                            cv::Mat ret;
                            cv::resize(Img(resizeRect(mBoxes[i+j])),ret,cv::Size(0,0),scall,scall);
                            retW[j] = ret.cols;
                            retH[j] = ret.rows;
                            ellipse[j] = ELSE::run(toGray(ret,Variante), quality[j],blob[j],Radius[r]);
                            //ellipse[j] = ELSE::run(toGray(Img(resizeRect(mBoxes[i+j])),3), quality[j],blob[j],15);
                        }
                        //cv::ellipse(Img(mBoxes[i+j]),ellipse[j],cv::Scalar(255,255,0),2);
                        //cv::imshow("Ergebnis_"+std::to_string(j),Img);
                    }
                });
                //cv::waitKey(300);
                for(int j = 0; j < 4; j++){
                    if(isEllipse(ellipse[j])){
                        gesHeight += retH[j];
                        gesWidth += retW[j];
                        myfile<<quality[j]<<" "<<blob[j]<<" "
                             <<compareEllipse(ellipse[j],mIris[i+j],scall,mBoxes[i+j])<<" " // <---
                             <<compareEllipse(ellipse[j],mPupils[i+j],scall,mBoxes[i+j])<<std::endl;
                        countGes++;
                        QualityGes += quality[j];
                    }
                }
            }
            myfile.close();
            if(countGes == 0){
                std::cout<<"Abbruch"<<std::endl;
                break;
            }else{
                std::cout<<"Gefunden "<<countGes<<" mit "<<QualityGes/countGes<<" "<<gesWidth/countGes<<" "<<gesHeight/countGes<<std::endl;
                myfileGes<<scall<<" Gefunden "<<countGes<<"/"<<mImagePaths.size()<<" "<<gesWidth/countGes<<" "<<gesHeight/countGes<<std::endl;
            }
            sleep(20);// Vermeidung von überhitzung
        }
        myfileGes.close();
        std::cout<<"Ende "<<Radius[r]<<std::endl;
    }
    std::cout<<"Variante "<<Variante<<"Ende"<<std::endl;
    }
    std::cout<<"Fertig"<<std::endl;
    return a.exec();
}
