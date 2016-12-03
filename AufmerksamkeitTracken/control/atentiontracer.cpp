#include "atentiontracer.h"

#include <iostream>
#include "FaceAnalyser.h"
#include "model/image.h"

AtentionTracer::AtentionTracer(Ui::MainWindow *parent)
{
    mTheWindow = parent;

    mWorldSize = cv::Size(312,208);
    mAtentSize = cv::Size(312,208);

    mWorldPose = cv::Vec6d(0,200,1200,1,-0.5,0);
    mAttentionPose = cv::Vec6d(0,0,1400,0,0,0);
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

cv::Point AtentionTracer::calcPose2Image(cv::Vec3d point, cv::Vec6d pose){
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pose[3],pose[4],pose[5]));
    cv::Vec3d p = R*(point-cv::Vec3d(pose[0],pose[1],pose[2]));
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
        cv::Point p1 = calcPose2Image(cv::Vec3d(pos[0],pos[1],pos[2]), mWorldPose);
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pos[3],pos[4],pos[5]));
        cv::Vec3d pose2(cv::Vec3d(pos[0],pos[1],pos[2])+(R*cv::Vec3d(0,0,-30)));
        cv::Point p2 = calcPose2Image(pose2, mWorldPose);
        cv::arrowedLine(img, p1,p2, color);
    }

    QImage img2 = Image::MatToQImage(img);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomCenter_label->setPixmap(pix);
}

void AtentionTracer::printAttention(){
    cv::Mat img(mAtentSize, CV_8UC3, cv::Scalar(0,0,0));
    for(int i = 0; i < mCamPose.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mCamPose[i];
        cv::Vec3d position(pos[0],pos[1],pos[2]);
        cv::Vec3d orientation(pos[3],pos[4],pos[5]);
        cv::Point p1 = calcPose2Image(position,mAttentionPose);
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(orientation);
        cv::Point p2 = calcPose2Image(position+(R*cv::Vec3d(0,0,-50)),mAttentionPose);
        cv::arrowedLine(img, p1,p2, color);
        printCirclePoints(img,cv::Vec3d(0,0,0),
                          cv::Vec3d(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i])),
                          position,
                          R*cv::Vec3d(0,0,-1));
    }

    QImage img2 = Image::MatToQImage(img);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomRight_label->setPixmap(pix);
}

void AtentionTracer::printCirclePoints(cv::Mat &img, cv::Vec3d center, cv::Vec3b color, cv::Vec3d position, cv::Vec3d orientation){
    cv::Point point= calcPose2Image(center, mAttentionPose);
    img.at<cv::Vec3b>(point) = cv::Vec3b(255,255,255);

    cv::Vec3d dif = position-center;
    double loc = dif[0]*orientation[0]+dif[1]*orientation[1]+dif[2]*orientation[2];
    double oc = dif[0]*dif[0]+dif[1]*dif[1]+dif[2]*dif[2];

    for(int i = 0; i < 260; i+=10){
        double sq = loc*loc-oc+i*i;
        if(sq >=0){
            cv::circle(img, point, cvRound(mAtentSize.width*i/sqrt(oc)), cv::Scalar(255,255,255));
            cv::circle(img, point, cvRound(mAtentSize.width*(i-10)/sqrt(oc)), cv::Scalar(255,255,255));
            double d1 = -loc+sqrt(sq);
            double d2 = -loc-sqrt(sq);
            cv::Point point1= calcPose2Image(position+d1*orientation, mAttentionPose);
            cv::Point point2= calcPose2Image(position+d2*orientation, mAttentionPose);
            img.at<cv::Vec3b>(point1) = color;
            img.at<cv::Vec3b>(point2) = color;
            std::cout<<"Treffer ["<<i-10<<", "<<i<<"]"<<position<<std::endl;
            break;
        }
    }
}

cv::Vec3d AtentionTracer::unitVector(cv::Vec3d vec){
    double n = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
    cv::Vec3d ret(vec[0]/n, vec[1]/n, vec[2]/n);
    std::cout<<vec<<"->"<<ret<<std::endl;
    return ret;
}
