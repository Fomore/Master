#include "einzelbilder.h"

#include <QStringList>

Einzelbilder::Einzelbilder()
{
    mImagePaths.clear();
//    cv::glob("/home/falko/Uni/Master/Bilder/*g", mImagePaths, true);
//    cv::glob("/home/falko/Uni/Master/Bilder/Learn/*.jpgg", mImagePaths, true);
//    cv::glob("/home/falko/Uni/Master/Bilder/HeadPose/*.jpg", mImagePaths, true);
//    cv::glob("/home/falko/Uni/Master/Bilder/Aufnahme2/*.JPG", mImagePaths, true);
    cv::glob("/home/falko/Uni/Master/Bilder/HeadPose/*.jpg", mImagePaths, true);
//    cv::glob("/home/falko/Uni/Master/Bilder/Test/*.*g", mImagePaths, true);
    for(size_t i = 0; i < mImagePaths.size(); i++){
        mFramePos.push_back(-1);
    }

    /*
    mRects.push_back(cv::Rect(230, 65, 48, 64));
    mRects.push_back(cv::Rect(381, 95, 36, 48));
    mRects.push_back(cv::Rect(440, 90, 35, 45));
    mRects.push_back(cv::Rect(304,117, 21, 37));
    mRects.push_back(cv::Rect(161, 72, 35, 50));
    mRects.push_back(cv::Rect(516, 92, 41, 56));
    mRects.push_back(cv::Rect( 75, 80, 40, 50));
    */
}

Einzelbilder::~Einzelbilder()
{

}

bool Einzelbilder::getImage(cv::Mat &Img, size_t &ImageID, std::string &name, int &FramePos, cv::Rect &Rect)
{
    Img.release();
    if(ImageID < mImagePaths.size()){
        if(mFramePos[ImageID] >= 0){
            FramePos = mFramePos[ImageID];
            QStringList list = QString::fromStdString(mImagePaths.at(ImageID)).split("&");
            int pos = list.last().toInt();
            name = list.first().split("/").last().toStdString();
            if(pos >= 0 && pos < mRects.size()){
                Rect = mRects[pos];
            }else{
                Rect.x = Rect.y = Rect.width = Rect.height = 0;
            }
            return true;
        }else{
            FramePos = -1;
            Img =  cv::imread(mImagePaths.at(ImageID), -1);
            name = QString::fromStdString(mImagePaths.at(ImageID)).split("/").last().toStdString();
            if(Img.data){
                Rect.x = Rect.y = 0;
                Rect.width = Img.cols;
                Rect.height = Img.rows;
                return true;
            }else{
                Rect.x = Rect.y = Rect.width = Rect.height = 0;
                return false;
            }
        }
    }else{
        return false;
    }
}

void Einzelbilder::addImage(std::string name, int FramePos, int X, int Y, int Width, int Height)
{
    if(FramePos >= 0 && Width > 0 && Height  > 0){//es handelt sich um ein Frame in einem Video
        mImagePaths.push_back(name+"&"+std::to_string(mRects.size()));
        mFramePos.push_back(FramePos);
        mRects.push_back(cv::Rect(X,Y,Width,Height));
        //std::cout<<name<<" "<<FramePos<<" "<<X<<" "<<Y<<" "<<Width<<" "<<Height<<std::endl;
    }else{// es handelt sich um ein Bild (jpg/png/usw)
        mImagePaths.push_back(name);
        mFramePos.push_back(-1);
    }
}
