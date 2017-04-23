#include "atentiontracer.h"

#include <iostream>
#include "FaceAnalyser.h"
#include "GazeEstimation.h"
#include "model/image.h"

AtentionTracer::AtentionTracer(Ui::MainWindow *parent, Camera *cam)
{
    mTheWindow = parent;
    mKamera = cam;

    mWorldSize = cv::Size(312,208);
    mAtentSize = cv::Size(312,208);

    mWorldPoseCam = cv::Vec3d(0,5500,5500);
    mWorldCamOri = cv::Matx33d(0,0,-1,
                               -1,0,0,
                               0,1,0);

    mAttentionCam = cv::Vec6d(0,0,1400,0,0,0);
}

AtentionTracer::~AtentionTracer()
{

}

void AtentionTracer::reset(){
    mColores.clear();
    mHeadWorld.clear();
    mHeadPoses.clear();
    mGazeDirections0.clear();
    mGazeDirections1.clear();
}

void AtentionTracer::showSolution(QString name, const LandmarkDetector::CLNF &model, double fx, double fy, double cx, double cy, double colore, bool write)
{
    // Gaze tracking, absolute gaze direction
    cv::Point3f gazeDirection0(0, 0, -1);
    cv::Point3f gazeDirection1(0, 0, -1);
    FaceAnalysis::EstimateGaze(model, gazeDirection0, fx, fy, cx, cy, true);
    FaceAnalysis::EstimateGaze(model, gazeDirection1, fx, fy, cx, cy, false);
    cv::Vec6d headPoseWorld = LandmarkDetector::GetCorrectedPoseCamera(model, fx, fy, cx, cy);//Distanz in Millimeter

    newPosition(colore,headPoseWorld,model.params_global,gazeDirection0,gazeDirection1);

    if(mWriteToFile && write){
        writeSolutionToFile(name,model.params_global,headPoseWorld,gazeDirection0,gazeDirection1);
    }
}

// colore zwischen [0,1]
void AtentionTracer::newPosition(double colore, cv::Vec6d HeadPoseWorld, cv::Vec6d HeadPose,cv::Point3f GazeDirection0, cv::Point3f GazeDirection1){
    mColores.push_back(colore);
    mHeadWorld.push_back(HeadPoseWorld);
    mHeadPoses.push_back(HeadPose);
    mGazeDirections0.push_back(GazeDirection0);
    mGazeDirections1.push_back(GazeDirection1);
}

void AtentionTracer::writeSolutionToFile(QString name, cv::Vec6d Model, cv::Vec6d HeadPoseWorld, cv::Point3f GazeDirection0, cv::Point3f GazeDirection1)
{
    cv::Point2d worldAngle, rotatAngle;
    cv::Point3d worlPoint, target;
    getOrienation(name,worldAngle,worlPoint, rotatAngle, target);

    std::ofstream myfile;
    myfile.open ("./data/BerechnungWinkel_Video.txt", std::ios::in | std::ios::app);
    myfile <<worlPoint*10.0<<target*10.0<<Model<<"|"
          <<HeadPoseWorld<<"|"<<calcAbweichung(HeadPoseWorld,target)
         <<calcAbweichung(cv::Vec3d(HeadPoseWorld[0],HeadPoseWorld[1],HeadPoseWorld[2]),GazeDirection0,target)
        <<calcAbweichung(cv::Vec3d(HeadPoseWorld[0],HeadPoseWorld[1],HeadPoseWorld[2]),GazeDirection1,target)<<std::endl;
    myfile.close();
}

cv::Vec6d AtentionTracer::calcAbweichung(cv::Vec6d Params,cv::Point3d Target)
{
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(Params[3],Params[4],Params[5]));
    cv::Vec3d Pos(Params[0],Params[1],Params[2]);

    cv::Vec3d ori = R*cv::Vec3d(0,0,-1);

    return calcAbweichung(Pos,ori,Target);
}

cv::Vec6d AtentionTracer::calcAbweichung(cv::Vec3d Start, cv::Point3f Orientierung, cv::Vec3d Target){
    return calcAbweichung(Start,cv::Vec3d(Orientierung.x,Orientierung.y,Orientierung.z),Target);
}

cv::Vec6d AtentionTracer::calcAbweichung(cv::Vec3d Start, cv::Vec3d Orientierung, cv::Vec3d Target)
{
    cv::Vec3d TargetRot = mKamera->rotateToCamera(Target);
    double q = (TargetRot[2]*10.0-Start[2])/Orientierung[2];
    cv::Vec3d solution = Start+Orientierung*q-TargetRot*10.0;
    return cv::Vec6d(solution[0],solution[1],solution[2],
            atan2(solution[0],Start[2]-TargetRot[2]),
            atan2(solution[1],Start[2]-TargetRot[2]),
            atan2(solution[2],Start[2]-TargetRot[2]));
}

void AtentionTracer::print(){
    printImageOrientation();
    printWorld();
    printAttention();
    reset();
}

void AtentionTracer::printImageOrientation(){
    cv::Mat img(mImageSize, CV_8UC3, cv::Scalar(255,255,255));
    int thickness = max(1,(int)std::ceil(3.0* (double)mImageSize.width/ 640.0));
    for(size_t i = 0; i < mHeadPoses.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mHeadPoses[i];
        cv::arrowedLine(img, cv::Point(cvRound(pos[4]),cvRound(pos[5])), calcArrowEndImage(pos), color,thickness);

        cv::arrowedLine(img, cv::Point(cvRound(pos[4]),cvRound(pos[5])),
                from3DTo2D(pos[4],pos[5],mGazeDirections0[i].x,mGazeDirections0[i].y,mImageSize.width,pos[0]),
                cv::Scalar(110.0, 220.0, 0.0),thickness);
        cv::arrowedLine(img, cv::Point(cvRound(pos[4]),cvRound(pos[5])),
                from3DTo2D(pos[4],pos[5],mGazeDirections1[i].x,mGazeDirections1[i].y,mImageSize.width,pos[0]),
                cv::Scalar(110.0, 220.0, 0.0),thickness);
    }

    if(!img.empty()){
    QImage img2 = Image::MatToQImage(img).scaled(mTheWindow->ImageBottomLeft_label->size(),Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomLeft_label->setPixmap(pix);
    }
}

void AtentionTracer::printWorld(){
    cv::Mat img(mWorldSize, CV_8UC3, cv::Scalar(0,0,0));
    cv::circle(img,calcPose2Image(cv::Vec3d(0,0,0),mWorldPoseCam),3,cv::Scalar(0,0,255),-1);
    for(size_t i = 0; i < mHeadWorld.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mHeadWorld[i];
        cv::Point p1 = calcPose2Image(cv::Vec3d(pos[0],pos[1],pos[2]), mWorldPoseCam);
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pos[3],pos[4],pos[5]));
        cv::Vec3d pose2(cv::Vec3d(pos[0],pos[1],pos[2])+(R*cv::Vec3d(0,0,-1000)));
        cv::Point p2 = calcPose2Image(pose2, mWorldPoseCam);
        cv::arrowedLine(img, p1,p2, color);
    }

    if(!img.empty()){
    QImage img2 = Image::MatToQImage(img).scaled(mTheWindow->ImageBottomCenter_label->size(),Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomCenter_label->setPixmap(pix);
    }
}

void AtentionTracer::printAttention(){
    cv::Mat img(mAtentSize, CV_8UC3, cv::Scalar(0,0,0));
    for(size_t i = 0; i < mHeadWorld.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mHeadWorld[i];
        cv::Vec3d position(pos[0],pos[1],pos[2]);
        cv::Vec3d orientation(pos[3],pos[4],pos[5]);
        cv::Point p1 = calcPose2Image(position,mAttentionCam);
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(orientation);
        cv::Point p2 = calcPose2Image(position+(R*cv::Vec3d(0,0,-50)),mAttentionCam);
        cv::arrowedLine(img, p1,p2, color);
        calcCirclePoints(img,cv::Vec3d(0,0,0),
                          cv::Vec3d(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i])),
                          position, R*cv::Vec3d(0,0,-1));
    }

    if(!img.empty()){
    QImage img2 = Image::MatToQImage(img).scaled(mTheWindow->ImageBottomRight_label->size(),Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    mTheWindow->ImageBottomRight_label->setPixmap(pix);
    }
}

cv::Point AtentionTracer::calcArrowEndImage(cv::Vec6d headPose){
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(headPose[1],headPose[2],headPose[3]));
    cv::Vec3d p = R*cv::Vec3d(0,0,-1);
    return from3DTo2D(headPose[4],headPose[5],p[0],p[1],mImageSize.width,headPose[0]);
}

cv::Point AtentionTracer::from3DTo2D(double X, double Y, double OriX, double OriY, int size, double scall)
{
    double s = getScall(size,scall);
    return cv::Point(cvRound(X+OriX*s),cvRound(Y+OriY*s));
}

cv::Point AtentionTracer::calcPose2Image(cv::Vec3d point, cv::Vec3d pose){
    cv::Vec3d p = mWorldCamOri*(pose-point);
    double fx = mWorldSize.width/2.0;
    return cv::Point(fx*p[0]/p[2]+fx, fx*p[1]/p[2]+mWorldSize.height/2.0);
}

cv::Point AtentionTracer::calcPose2Image(cv::Vec3d point, cv::Vec6d pose){
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pose[3],pose[4],pose[5]));
    cv::Vec3d p = R*(cv::Vec3d(pose[0],pose[1],pose[2])-point);
    return cv::Point(mWorldSize.width*p[0]/p[2]+mWorldSize.width/2.0,
            mWorldSize.width*p[1]/p[2]+mWorldSize.height/2.0);
}

void AtentionTracer::calcCirclePoints(cv::Mat &img, cv::Vec3d center, cv::Vec3b color, cv::Vec3d position, cv::Vec3d orientation){
    cv::Point point= calcPose2Image(center, mAttentionCam);
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
            cv::Point point1= calcPose2Image(position+d1*orientation, mAttentionCam);
            cv::Point point2= calcPose2Image(position+d2*orientation, mAttentionCam);
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

double AtentionTracer::getScall(int size, double scall)
{
    return max(size/10.0,min(50.0,size/5.0*scall));
}

void AtentionTracer::setWriteToFile(bool write)
{
    mWriteToFile = write;
}

void AtentionTracer::setImageSize(int Width, int Height){
    mImageSize.width = Width;
    mImageSize.height = Height;
}
