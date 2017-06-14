#ifndef ATENTIONTRACER_H
#define ATENTIONTRACER_H

#include <opencv2/opencv.hpp>

#include "ui_mainwindow.h"
#include "model/target.h"
#include "LandmarkCoreIncludes.h"

class AtentionTracer : public Target
{
private:
    Ui::MainWindow* mTheWindow;

    cv::Mat mAttentionSumImage;

    bool mWriteToFile = true;
    bool mShowAtention = true;
    bool mUseTime = true;

    std::vector<double> mColores;
    //Positives linkshändiges Koordinatensystem
    std::vector<cv::Vec6d> mHeadWorld;
    std::vector<cv::Vec6d> mHeadPoses;

    std::vector<cv::Point3f> mGazeDirections0;
    std::vector<cv::Point3f> mGazeDirections1;

    cv::Size mImageSize;

    // Position und Orientierung der World-Kamera
    cv::Vec3d mWorldPoseCam;
    cv::Matx33d mWorldCamOri;
    // Position und Orientierung der Attention-Kamera
    cv::Vec3d mAttentionCamPose;
    cv::Matx33d mAttentiondCamOri;

    cv::Point calcArrowEndImage(cv::Vec6d headPose);
    cv::Point from3DTo2D(double X, double Y, double OriX, double OriY, int size, double scall);
    cv::Point calcPose2Image(cv::Vec3d point, const cv::Vec3d &pose, const cv::Matx33d &R, double fx, double cx, double cy);

    double getScall(int size, double scall);

    void printImageOrientation();
    void printWorld();
    void printAttention();
    void printTargets(cv::Mat &img, const cv::Vec3d &Pose, const cv::Matx33d Ori, double fx, double cx, double cy);
    void printGrid(cv::Mat &img, cv::Point3d Point1, cv::Point3d Point2, double Step, const cv::Vec3d &Cam, const cv::Matx33d &R, double fx, double cx, double cy);

    void printAllTarget(cv::Mat &img, const cv::Vec3d &Pose, const cv::Matx33d Ori, double fx, double cx, double cy, cv::Scalar Colore);

    double calcAbweichung(cv::Vec6d Params, cv::Point3d Target);
    double calcAbweichung(cv::Vec3d Start, cv::Vec3d Orientierung, cv::Vec3d Target);
    double calcAbweichung(cv::Vec3d Start, cv::Point3f Orientierung, cv::Vec3d Target);

    void newPosition(double colore, cv::Vec6d HeadPoseWorld, cv::Vec6d HeadPose, cv::Point3f GazeDirection0, cv::Point3f GazeDirection1);
    void writeSolutionToFile(QString name, cv::Vec6d Model, cv::Vec6d HeadPoseWorld, cv::Point3f GazeDirection0, cv::Point3f GazeDirection1);
    bool linePlaneIntersection(cv::Vec3d &contact, cv::Vec3d ray, cv::Vec3d rayOrigin, cv::Vec3d normal, cv::Vec3d coord);
public:
    AtentionTracer(Ui::MainWindow *parent = 0, Camera *cam = 0, QString TargetFileName = "");
    ~AtentionTracer();
    void print();
    void reset();
    void setImageSize(int Width, int Height);
    void showSolution(QString name, const LandmarkDetector::CLNF &model,
                      double fx, double fy, double cx, double cy, double colore, bool write);
    void setWriteToFile(bool write);
    void setShowAtention(bool show);

    bool getUseTime();
    void setUseTime(bool t);

    double mVideoTimeShift = 0.0;
};

#endif // ATENTIONTRACER_H
