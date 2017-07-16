#include "atentiontracer.h"

#include <iostream>
#include "FaceAnalyser.h"
#include "GazeEstimation.h"
#include "model/image.h"
#include <cmath>

AtentionTracer::AtentionTracer(Ui::MainWindow *parent, Camera *cam, QString TargetFileName)
{
    mTheWindow = parent;
    mKamera = cam;

    mWorldPoseCam = cv::Vec3d(0,5500,5500);
    mWorldCamOri = cv::Matx33d(0,0,-1,
                               -1,0,0,
                               0,1,0);

    mAttentionCamPose = cv::Vec3d(0,0,-3500);
    mAttentiondCamOri = cv::Matx33d(1,0,0,
                                    0,1,0,
                                    0,0,1);
    loadFromFile(TargetFileName);

    mAttentionSumImage = cv::Mat(cv::Size(800,600), CV_8UC3, cv::Scalar(255,255,255));
    printAllTarget(mAttentionSumImage,mAttentionCamPose,mAttentiondCamOri,800,400,300,cv::Scalar(0,0,0));
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

void AtentionTracer::showSolution(QString name,int FrameNr, const LandmarkDetector::CLNF &model, double fx, double fy, double cx, double cy, double colore, bool write)
{
    // Gaze tracking, absolute gaze direction
    cv::Point3f gazeDirection0(0, 0, -1);
    cv::Point3f gazeDirection1(0, 0, -1);
    FaceAnalysis::EstimateGaze(model, gazeDirection0, fx, fy, cx, cy, true);
    FaceAnalysis::EstimateGaze(model, gazeDirection1, fx, fy, cx, cy, false);
    cv::Vec6d headPoseWorld = LandmarkDetector::GetPoseWorld(model, fx, fy, cx, cy);//Distanz in Millimeter

    if(mShowAtention){
        newPosition(colore,headPoseWorld,model.params_global,gazeDirection0,gazeDirection1);
    }

    if(mWriteToFile && write){
        cv::Vec4d box;
        getCLNFBox(model,box);
        writeSolutionToFile(name,FrameNr,model.params_global,headPoseWorld,box,gazeDirection0,gazeDirection1);
    }
}

// colore zwischen [0,1]
void AtentionTracer::newPosition(double colore, cv::Vec6d HeadPoseWorld, cv::Vec6d HeadPose,cv::Point3f GazeDirection0, cv::Point3f GazeDirection1){
    mColores.push_back(colore);
    mHeadWorld.push_back(HeadPoseWorld);
    mHeadPoses.push_back(HeadPose);

    if(mUseAVGEye){
        cv::Point3f Solution = calcVectorAVG(GazeDirection0,GazeDirection1);
        mGazeDirections0.push_back(Solution);
        mGazeDirections1.push_back(Solution);
    }else{
        mGazeDirections0.push_back(GazeDirection0);
        mGazeDirections1.push_back(GazeDirection1);
    }
}

void AtentionTracer::writeSolutionToFile(QString name,int FrameNr,
                                         cv::Vec6d Model, cv::Vec6d HeadPoseWorld, cv::Vec4d Box,
                                         cv::Point3f GazeDirection0, cv::Point3f GazeDirection1)
{
    cv::Point2d worldAngle, rotatAngle;
    cv::Point3d worlPoint, target;
    if(mUseTime){
        getOrienation(mKamera->getTimeSec()-mVideoTimeShift,worldAngle,worlPoint, rotatAngle, target);
    }else{
        getOrienation(name,worldAngle,worlPoint, rotatAngle, target);
    }

    std::ofstream myfile;
    myfile.open ("./data/Test_Versuch_1_video_resize.txt", std::ios::in | std::ios::app);
    myfile <<FrameNr<<" "<<worlPoint<<" "<<target<<" "<<Model<<" |"
          <<HeadPoseWorld<<" "<<GazeDirection0<<" "<<GazeDirection1<<" "<<Box<<"| "
         <<calcWinkel(worlPoint,target)
        <<" "<<calcAbweichung(HeadPoseWorld,target)
       <<" "<<calcAbweichung(cv::Vec3d(HeadPoseWorld[0],HeadPoseWorld[1],HeadPoseWorld[2]),GazeDirection0,target)
      <<" "<<calcAbweichung(cv::Vec3d(HeadPoseWorld[0],HeadPoseWorld[1],HeadPoseWorld[2]),GazeDirection1,target)
     <<" "<<calcAbweichung(cv::Vec3d(HeadPoseWorld[0],HeadPoseWorld[1],HeadPoseWorld[2]),calcVectorAVG(GazeDirection0,GazeDirection1),target)<<std::endl;
    myfile.close();
}

cv::Vec4d AtentionTracer::calcAbweichung(cv::Vec6d Params,cv::Point3d Target)
{
    cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(Params[3],Params[4],Params[5]));
    cv::Vec3d Pos(Params[0],Params[1],Params[2]);

    cv::Vec3d ori = R*cv::Vec3d(0,0,-1);

    return calcAbweichung(Pos,ori,Target);
}

cv::Vec4d AtentionTracer::calcAbweichung(cv::Vec3d Start, cv::Point3f Orientierung, cv::Vec3d Target){
    return calcAbweichung(Start,cv::Vec3d(Orientierung.x,Orientierung.y,Orientierung.z),Target);
}

cv::Vec3d AtentionTracer::calcWinkel(cv::Point3d B, cv::Point3d A)
{
    cv::Point3d p(B.x-A.x, B.y-A.y, B.z-A.z);
    cv::Vec3d tmp = calcWinkel(p);
    std::cout<<A<<B<<p<<tmp*180/M_PI<<std::endl;
    return tmp;
}

cv::Vec3d AtentionTracer::calcWinkel(cv::Point3d P)
{
    return cv::Vec3d(atan((P.x)/abs(P.z)),
                     atan((P.y)/abs(P.z)),
                     atan(P.y/sqrt(pow((P.x),2)+pow((P.z),2))));
}

cv::Vec4d AtentionTracer::calcAbweichung(cv::Vec3d Start, cv::Vec3d Orientierung, cv::Vec3d Target)
{
    cv::Vec3d ori = calcWinkel(Orientierung);
    cv::Vec4d retVec(ori[0],ori[1],ori[2],0.0);
    cv::Vec3d normale = mKamera->getRotationMatrix() * cv::Vec3d(0,0,1);
    cv::Vec3d contact;
    if(linePlaneIntersection(contact,Orientierung,Start,normale,cv::Vec3d(0,0,0))){
        cv::Vec3d TargetRot = mKamera->rotateToCamera(Target)*0.01;
        contact *= 0.01;
        double di = sqrt(contact[0]*contact[0]+contact[1]*contact[1]+contact[2]*contact[2])
                *sqrt(TargetRot[0]*TargetRot[0]+TargetRot[1]*TargetRot[1]+TargetRot[2]*TargetRot[2]);
        retVec[3] =  acos(TargetRot.dot(contact) / di);
    }else{
        retVec[3] =  -1.0;
    }
    return retVec;
}

void AtentionTracer::print(){
    if(mShowAtention){
        printImageOrientation();
        printWorld();
        printAttention();
    }
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
    cv::Size mWorldSize(mTheWindow->ImageBottomCenter_label->width(),mTheWindow->ImageBottomCenter_label->height());
    cv::Mat img(mWorldSize, CV_8UC3, cv::Scalar(0,0,0));
    double fx = mWorldSize.width/2.0;
    double cy = mWorldSize.height/2.0;
    cv::circle(img,calcPose2Image(cv::Vec3d(0,0,0),mWorldPoseCam,mWorldCamOri,fx,fx,cy),cvRound(fx/50),cv::Scalar(0,0,255),-1);
    printGrid(img,cv::Point3d(-3000,0,0),cv::Point3d(3000,0,11000),1000.0,mWorldPoseCam,mWorldCamOri,fx,fx,cy);
    for(size_t i = 0; i < mHeadWorld.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mHeadWorld[i];

        cv::Point p1 = calcPose2Image(cv::Vec3d(pos[0],pos[1],pos[2]), mWorldPoseCam, mWorldCamOri, fx,fx,cy);
        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pos[3],pos[4],pos[5]));
        cv::Vec3d pose2(cv::Vec3d(pos[0],pos[1],pos[2])+(R*cv::Vec3d(0,0,-1000)));
        cv::Point p2 = calcPose2Image(pose2, mWorldPoseCam, mWorldCamOri, fx,fx,cy);
        cv::arrowedLine(img, p1,p2, color);
    }

    if(!img.empty()){
        QPixmap pix = QPixmap::fromImage(Image::MatToQImage(img));
        mTheWindow->ImageBottomCenter_label->setPixmap(pix);
    }
}

void AtentionTracer::printAttention(){
    cv::Size mAtentSize(mTheWindow->ImageBottomRight_label->width(),mTheWindow->ImageBottomRight_label->height());
    cv::Mat img(mAtentSize, CV_8UC3, cv::Scalar(0,0,0));
    double fx = mAtentSize.width/2.0;
    double cy = mAtentSize.height/2.0;
    printGrid(img,cv::Point3d(-5000,-4000,0),cv::Point3d(5000,4000,0),1000.0,mAttentionCamPose,mAttentiondCamOri,fx,fx,cy);
    printTargets(img,mAttentionCamPose,mAttentiondCamOri,fx,fx,cy);
    cv::circle(img,calcPose2Image(cv::Vec3d(0,0,0),mAttentionCamPose,mAttentiondCamOri,fx,fx,cy),cvRound(fx/50),cv::Scalar(0,0,255),-1);
    cv::Vec3d normale = mKamera->getRotationMatrix() * cv::Vec3d(0,0,1);
    cv::Vec3b center(0,0,-mKamera->getTranslation()[3]);
    for(size_t i = 0; i < mHeadWorld.size(); i++){
        cv::Scalar color(255.0*(1.0-mColores[i]),255.0*mColores[i],255.0*(1.0-mColores[i]));
        cv::Vec6d pos = mHeadWorld[i];
        cv::Vec3d position(pos[0],pos[1],pos[2]);
        cv::Vec3d orientation(pos[3],pos[4],pos[5]);

        cv::Matx33d R = LandmarkDetector::Euler2RotationMatrix(orientation);
        cv::Vec3d ray = R*cv::Vec3d(0,0,-1);

        cv::Vec3d contact;
        if(linePlaneIntersection(contact,ray,position,normale,center)){
            cv::circle(img,calcPose2Image(contact,mAttentionCamPose,mAttentiondCamOri,fx,fx,cy),cvRound(fx/50),color,-1);
            if(mSaveVideoImage){
                cv::circle(mAttentionSumImage,calcPose2Image(contact,mAttentionCamPose,mAttentiondCamOri,800,400,300),3,color,-1);
            }
        }
    }
    if(!img.empty()){
        QPixmap pix = QPixmap::fromImage(Image::MatToQImage(img));
        mTheWindow->ImageBottomRight_label->setPixmap(pix);
    }
    if(mSaveVideoImage){
    cv::imwrite("Summe.png",mAttentionSumImage);
    }
}

void AtentionTracer::printTargets(cv::Mat &img, const cv::Vec3d &Pose, const cv::Matx33d Ori, double fx, double cx, double cy)
{
    if(mUseTime){
        cv::circle(img,calcPose2Image(mKamera->rotateToCamera(getTimeTarget(mKamera->getTimeSec()-mVideoTimeShift)),
                                      Pose,Ori,fx,cx,cy),
                   cvRound(fx/50),cv::Scalar(255,0,0),-1);
    }else{
        for(size_t i = 0; i < mPoints.size() ; i++){
            cv::circle(img,calcPose2Image(mKamera->rotateToCamera(mPoints[i]),Pose,Ori,fx,cx,cy),
                       cvRound(fx/50),cv::Scalar(255,0,0),-1);
        }
    }
}

void AtentionTracer::printAllTarget(cv::Mat &img, const cv::Vec3d &Pose, const cv::Matx33d Ori, double fx, double cx, double cy, cv::Scalar Colore)
{
    for(size_t i = 0; i+1 < mTimePoints.size(); i++){
        cv::line(img,calcPose2Image(mKamera->rotateToCamera(mTimePoints[i]),Pose,Ori,fx,cx,cy),
                     calcPose2Image(mKamera->rotateToCamera(mTimePoints[i+1]),Pose,Ori,fx,cx,cy),
                Colore,2);
    }
}

void AtentionTracer::printGrid(cv::Mat &img, cv::Point3d Point1, cv::Point3d Point2, double Step,const cv::Vec3d &Cam,
                               const cv::Matx33d &R, double fx, double cx, double cy)
{
    for(double x = Point1.x; x <= Point2.x; x+= Step){
        for(double y = Point1.y; y <= Point2.y; y += Step){
            for(double z = Point1.z; z <= Point2.z; z += Step){
                cv::circle(img,calcPose2Image(cv::Vec3d(x,y,z),Cam,R,fx,cx,cy),cvRound(fx/200),cv::Scalar(255,255,255),-1);
            }
        }
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

cv::Point AtentionTracer::calcPose2Image(cv::Vec3d point,const cv::Vec3d &pose, const cv::Matx33d &R, double fx, double cx, double cy){
    cv::Vec3d p = R*(point-pose);
    return cv::Point(fx*p[0]/p[2]+cx, fx*p[1]/p[2]+cy);
}

/*
 * Diese Funktion stammt aus:
 * http://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
 *
contact = the contact point on the plane, this is what i want calculated
ray = B - A, simply the line from A to B
rayOrigin = A, the origin of the line segement
normal = normal of the plane (normal of a triangle)
coord = a point on the plane (vertice of a triangle)
 */
bool AtentionTracer::linePlaneIntersection(cv::Vec3d& contact, cv::Vec3d ray, cv::Vec3d rayOrigin,
                           cv::Vec3d normal, cv::Vec3d coord) {
    // get d value
    float d = normal.dot(coord);

    if (normal.dot(ray) == 0) {
        return false; // No intersection, the line is parallel to the plane
    }

    // Compute the X value for the directed line ray intersecting the plane
    float x = (d - normal.dot(rayOrigin)) / normal.dot(ray);

    // output contact point
    contact = rayOrigin + ray*x; //Make sure your ray vector is normalized
    return true;
}

void AtentionTracer::getCLNFBox(const LandmarkDetector::CLNF &model, cv::Vec4d &Box)
{
    cv::Mat_<double> shape2D = model.detected_landmarks;

    int n = shape2D.rows/2;

    Box[0] = Box[2] = shape2D.at<double>(0);
    Box[1] = Box[3]= shape2D.at<double>(n);
    for(int i = 1; i < n; i++)// Beginnt bei 0 das Output-Format
    {
        double x = (shape2D.at<double>(i));
        double y = (shape2D.at<double>(i + n));
        Box[0] = min(Box[0],x);
        Box[1] = min(Box[1],y);
        Box[2] = max(Box[2] ,x);
        Box[3]= max(Box[3],y);
    }
    Box[2] -= Box[0];
    Box[3]-= Box[1];
}

double AtentionTracer::getScall(int size, double scall)
{
    return max(size/10.0,min(50.0,size/5.0*scall));
}

void AtentionTracer::setWriteToFile(bool write)
{
    mWriteToFile = write;
}

void AtentionTracer::setShowAtention(bool show)
{
    mShowAtention = show;
}

bool AtentionTracer::getUseTime()
{
    return mUseTime;
}

void AtentionTracer::setUseTime(bool t)
{
    mUseTime = t;
}

void AtentionTracer::setSaveVideoImage(bool v)
{
    mSaveVideoImage = v;
}

void AtentionTracer::setUseAVGEye(bool e)
{
    mUseAVGEye = e;
}

cv::Point3f AtentionTracer::calcVectorAVG(cv::Point3f A, cv::Point3f B)
{
    cv::Point3f Solution(A.x+B.x,
                         A.y+B.y,
                         A.z+B.z);
    double n = sqrt(Solution.x*Solution.x
                   +Solution.y*Solution.y
                   +Solution.z*Solution.z);
    if(n > 0){
        Solution.x = Solution.x/n;
        Solution.y = Solution.y/n;
        Solution.z = Solution.z/n;
    }
    return Solution;
}

void AtentionTracer::setImageSize(int Width, int Height){
    mImageSize.width = Width;
    mImageSize.height = Height;
}
