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

cv::Mat getFunktionsverlauf(double Min, double Max){
    // the matrix of coefficients
    cv::Mat A = (cv::Mat_<double>(4,4) <<
                 pow(Min,3),pow(Min,2),Min,1,
               3*pow(Min,2),     2*Min,  1,0,

                 pow(Max,3),pow(Max,2),Max,1,
               3*pow(Max,2),     2*Max,  1,0);
    /*
    cv::Mat A = (cv::Mat_<double>(6,6) <<
                 pow(Min,5),   pow(Min,4),  pow(Min,3),pow(Min,2),Min,1,
               5*pow(Min,4), 4*pow(Min,3),3*pow(Min,2),     2*Min,  1,0,
              20*pow(Min,3),12*pow(Min,2),       6*Min,         2,  0,0,

                 pow(Max,5),   pow(Max,4),  pow(Max,3),pow(Max,2),Max,1,
               5*pow(Max,4), 4*pow(Max,3),3*pow(Max,2),     2*Max,  1,0,
              20*pow(Max,3),12*pow(Max,2),       6*Max,         2,  0,0);
    */

    //the vector of constants
    cv::Mat B = (cv::Mat_<double>(4,1) << 0,0, 255.0/Max,0);
    //cv::Mat B = (cv::Mat_<double>(6,1) << 0,0,0, 255.0/Max,0,0);

    //the vector of variables (results)
    cv::Mat x;
    cv::solve(A, B, x);
    return x;
}

cv::Mat convert_rgb(cv::Mat in, int abw = 0){
    if(in.type() != CV_8UC4 || abw == 6){
//        std::cout<<"Schnell"<<std::endl;
        cv::Mat out;
        cv::cvtColor(in, out, CV_BGR2GRAY);
        cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        return out;
    }else if(abw == 5 || abw == 4){
        cv::Mat gray;
        cv::Mat gaus;
        cv::cvtColor(in, gray, CV_BGR2GRAY);
        cv::GaussianBlur(gray, gaus, cv::Size(7,7), 1.5, 1.5);
        double min,max;
        cv::minMaxLoc(gaus, &min, &max);
        cv::Mat out=cv::Mat::zeros(in.size(),CV_8UC1);
        if(abw == 4){
            double a = 255.0/max;
            for(int i=0;i<in.rows;i++){
                for(int j=0;j<in.cols;j++){
                    out.at<uchar>(i,j)= std::min(255,std::max((int)(gray.at<uchar>(i,j)*a-min+0.5),0));
                }
            }
        }else{
            cv::Mat val = getFunktionsverlauf(min,max);
            for(int i=0;i<in.rows;i++){
                for(int j=0;j<in.cols;j++){
                    double p = gray.at<uchar>(i,j);
                    if(p <= min){
                        out.at<uchar>(i,j) = 0;
                    }else if(p >= max){
                        out.at<uchar>(i,j) = 255;
                    }else{
                        double sol = 0;
                        for(int s = 0; s < val.rows; s++){
                            sol += val.at<double>(s,0)*pow(p,val.rows-1-s);
                        }
                        out.at<uchar>(i,j)= std::min(255,std::max((int)(p*sol+0.5),0));
                    }
                }
            }
        }
        return out;
    }

    cv::Mat out=cv::Mat::zeros(in.size(),CV_32FC1);

    for(int i=0;i<in.rows;i++){
        for(int j=0;j<in.cols;j++){
            cv::Vec4b pix = in.at<cv::Vec4b>(i,j);
            float blue= (float)pix[0];
            float green=(float)pix[1];
            float red=(float)pix[2];
            if(pix[3] != 255){
                blue = green = red = 255.0;
            }


            //float gray_val= (0.35663438*blue) + (0.24882321*green) + (0.39454241*red);
            //float gray_val= (0.24079208*red) + (0.16214176*green) + (0.59706616*blue);
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
                double rs = 1;
                double gs = 1;
                double bs = 1;
                if(rs <= 73)
                    rs = -0.00032440*pow(red,2)+0.071533*red;
                if(gs <= 78)
                    gs = -0.00022737*pow(green,2)+0.059648*green;
                if(bs <= 85)
                    bs = -0.00026996*pow(blue,2)+0.053288*blue;
                gray_val= (red*rs + green*gs + blue*bs)/3.0;
            }else if(abw == 3){
                double rs = 0.059077*red-0.81949;
                double gs = 0.053561*green-0.90855;
                double bs = 0.033964*blue-0.66562;
                gray_val=std::max(0.0,std::min(255.0,(red*rs + green*gs + blue*bs)/3.0));
            }
            out.at<float>(i,j)=gray_val;
        }
    }

    cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_32FC1);
    out.convertTo(out,CV_8UC1);

    return out;
}

cv::Vec4f compareEllipse(cv::RotatedRect Ellipse1, cv::RotatedRect Ellipse2, double scall, cv::Rect Box){
    double x = Box.x;
    double y = Box.y;
    double abstandCenter = sqrt(pow(Ellipse1.center.x-scall*(Ellipse2.center.x-x),2)
                               +pow(Ellipse1.center.y-scall*(Ellipse2.center.y-y),2));
//    std::cout<<Ellipse1.center<<Ellipse2.center<<" "<<x<<"/"<<y<<" "<<abstandCenter<<std::endl;
    float angle = Ellipse1.angle-Ellipse2.angle;
    if(angle < 0)
        angle *= -1.0;
    if(angle >= 180.0f)
        angle -= 180.0f;
    float width = std::abs(Ellipse1.size.width - scall*Ellipse2.size.width);
    float height = std::abs(Ellipse1.size.height - scall*Ellipse2.size.height);
    return cv::Vec4f(abstandCenter,angle,width,height);
}

bool isEllipse(cv::RotatedRect Ellipse){
    return Ellipse.center.x >= 0 && Ellipse.center.y >= 0 && Ellipse.size.width > 0 && Ellipse.size.height > 0;
}


bool myfunction (double i,double j) { return (i<j); }

cv::Vec3d Auswertung(std::vector<bool> Blob, std::vector<double> Values, int Typ, double Scall){
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
        return cv::Vec3d(val[q]/Scall,
                         (summe/((double) count))/Scall,
                         val[q*3]/Scall);
    }else{
        return cv::Vec3d(0,0,0);
    }
}

void AuswertungFile(){
    std::ofstream myfileGes;
    myfileGes.open ("Auswertung_algo.txt", std::ios::in | std::ios::app);
    std::vector<cv::String> FileNames;
    cv::glob("solution_algo/*0.txt",FileNames,false);
    for(size_t i = 0; i < FileNames.size(); i++){
        std::ifstream file(FileNames[i]);
        std::string line;

        size_t p = FileNames[i].size();
        std::string ScalString = FileNames[i].substr(p-12,8);
        double scall = std::stod(ScalString);
        std::cout<<"Berechne "<<scall<<std::endl;

        std::vector<std::vector<double>> Values;
        for(int j = 0; j < 9; j++){
            Values.push_back(std::vector<double>());
        }
        std::vector<bool> Blob;
        size_t CountBlob = 0;
        QRegExp ausdruck("[\\[\\]\\,]");
        if(file.is_open()){
            while (std::getline(file, line)){
                std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
                double val;
                bool blob;
                iss >> val;
                Values[0].push_back(val);
                iss >> blob;
                Blob.push_back(blob);
                for(int j = 1; j < 9; j++){
                    iss >> val;
                    Values[j].push_back(val);
                }
                if(blob){
                    CountBlob++;
                }
            }
        }
        myfileGes<<scall;
        for(size_t j = 0; j < Values.size(); j++){
//            Quality [abstandCenter,angle,width,height][abstandCenter,angle,width,height]
            if(j == 0 || j == 2 || j == 6){
                myfileGes<<Auswertung(Blob,Values[j],1,1)<<Auswertung(Blob,Values[j],2,1);
            }else{
                myfileGes<<Auswertung(Blob,Values[j],1,scall)<<Auswertung(Blob,Values[j],2,scall);
            }
        }
        double ant = (double)CountBlob/Blob.size();
        myfileGes<<" "<<std::to_string(ant)<<std::endl;
    }
    myfileGes.close();
}

cv::Rect resizeRect(cv::Rect Box){
    return cv::Rect(Box.x-3,Box.y-3,Box.width+6,Box.height+6);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    AuswertungFile();

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

    for(int i = 0; i < 8; i++){
        for(int j = 0; j <= 6; j++){
        cv::Mat Img = cv::imread(mImagePaths[i],-1);
        cv::imwrite("Ergebnis_"+std::to_string(i)+"_"+std::to_string(j)+".png",convert_rgb(Img(resizeRect(mBoxes[i])),j));
        }
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
                    if(Img.data && mBoxes[i+j].height*scall > 0.5){
                        cv::Mat ret;
                        cv::resize(Img(mBoxes[i+j]),ret,cv::Size(0,0),scall,scall);
                        //ellipse[j] = ELSE::run(ret, quality[j],blob[j]);
                        ellipse[j] = ELSE::run(convert_rgb(ret,0), quality[j],blob[j]);
                    }
                    //cv::ellipse(Img(mBoxes[i+j]),ellipse[j],cv::Scalar(255,255,0),2);
                    //cv::imshow("Ergebnis_"+std::to_string(j),Img);
                }
            });
            //cv::waitKey(300);
            for(int j = 0; j < 4; j++){
                if(isEllipse(ellipse[j])){
                    myfile<<quality[j]<<" "<<blob[j]<<" "
                         <<compareEllipse(ellipse[j],mIris[i+j],scall,mBoxes[i+j])<<" "
                        <<compareEllipse(ellipse[j],mPupils[i+j],scall,mBoxes[i+j])<<std::endl;
                    countGes++;
                    QualityGes += quality[j];
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
    std::cout<<"Ende"<<std::endl;
    return a.exec();
}
