#include "atentiontracer.h"

#include <iostream>
#include "FaceAnalyser.h"
#include "model/image.h"

AtentionTracer::AtentionTracer(Ui::MainWindow *parent)
{
    mTheWindow = parent;

    mWorldSize = cv::Size(312,208);
    mAtentSize = cv::Size(312,208);

    mWorldPose = cv::Vec6d(0,100,400,1,-0.5,0);
}

AtentionTracer::~AtentionTracer()
{

}

void AtentionTracer::reset(){
    mColores.clear();
    mCamPose.clear();
    mImgPose.clear();
}

// colore zwischen [0,1]
void AtentionTracer::newPosition(double colore, cv::Vec6d headPoseCam, cv::Vec6d headPoseImg){
    mColores.push_back(colore);
    mCamPose.push_back(headPoseCam);
    mImgPose.push_back(headPoseImg);
}

cv::Point AtentionTracer::calcArrowEndImage(cv::Vec6d headPose){
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(headPose[1],headPose[2],headPose[3]));
    cv::Vec3d p = R*cv::Vec3d(0,0,-mImageSize.width/5.0*headPose[0]);
    return cv::Point(cvRound(p[0]+headPose[4]),cvRound(p[1]+headPose[5]));
}

cv::Point AtentionTracer::calcWorlt2Image(cv::Vec3d point){
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(mWorldPose[3],mWorldPose[4],mWorldPose[5]));
    cv::Vec3d p = R*(point-cv::Vec3d(mWorldPose[0],mWorldPose[1],mWorldPose[2]));
    return cv::Point(mWorldSize.width*p[0]/p[2]+mWorldSize.width/2.0,
            mWorldSize.width*p[1]/p[2]+mWorldSize.height/2.0);
}

void AtentionTracer::setImageSize(int Width, int Height){
    mImageSize.width = Width;
    mImageSize.height = Height;
}

void AtentionTracer::print(){
    printImageOrientation();
    printWorld();
    printAttention();
    reset();
}

void AtentionTracer::printImageOrientation(){
    cv::Mat img(mImageSize, CV_8UC3, cv::Scalar(255,255,255));
    int thickness = (int)std::ceil(3.0* (double)mImageSize.width/ 640.0);
    for(int i = 0; i < mImgPose.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],0);
        cv::Vec6d pos = mImgPose[i];
        cv::arrowedLine(img, cv::Point(cvRound(pos[4]),cvRound(pos[5])), calcArrowEndImage(pos), color,thickness);
    }

    QImage img2 = Image::MatToQImage(img).scaled(mTheWindow->ImageBottomLeft_label->size(),Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomLeft_label->setPixmap(pix);
}

void AtentionTracer::printWorld(){
    cv::Mat img(mWorldSize, CV_8UC3, cv::Scalar(0,0,0));
    for(int i = 0; i < mCamPose.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mCamPose[i];
        cv::Point p1 = calcWorlt2Image(cv::Vec3d(pos[0],pos[1],pos[2]));
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pos[3],pos[4],pos[5]));
        cv::Point p2 = calcWorlt2Image(cv::Vec3d(pos[0],pos[1],pos[2])+(R*cv::Vec3d(0,0,-20)));
        cv::arrowedLine(img, p1,p2, color);
        std::cout<<"Welt: "<<pos<<std::endl;
    }

    QImage img2 = Image::MatToQImage(img);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomCenter_label->setPixmap(pix);
}

void AtentionTracer::printAttention(){
    cv::Mat img(mAtentSize, CV_8UC3, cv::Scalar(255,0,0));

    QImage img2 = Image::MatToQImage(img);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomRight_label->setPixmap(pix);
}
