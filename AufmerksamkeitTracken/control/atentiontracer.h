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

    bool mWriteToFile = true;

    std::vector<double> mColores;
    //Positives linkshändiges Koordinatensystem
    std::vector<cv::Vec6d> mHeadWorld;
    std::vector<cv::Vec6d> mHeadPoses;

    std::vector<cv::Point3f> mGazeDirections0;
    std::vector<cv::Point3f> mGazeDirections1;

    cv::Size mImageSize;
    cv::Size mWorldSize;
    cv::Size mAtentSize;

    // Position und Orientierung der World-Kamera
    cv::Vec3d mWorldPoseCam;
    cv::Matx33d mWorldCamOri;
    // Position und Orientierung der Attention-Kamera
    cv::Vec6d mAttentionCam;

    cv::Point calcArrowEndImage(cv::Vec6d headPose);
    cv::Point from3DTo2D(double X, double Y, double OriX, double OriY, int size, double scall);
    cv::Point calcPose2Image(cv::Vec3d point, const cv::Vec3d &pose, const cv::Matx33d &R, double fx, double cx, double cy);
    cv::Point calcPose2Image(cv::Vec3d point, cv::Vec6d pose);

    cv::Vec3d unitVector(cv::Vec3d vec);

    double getScall(int size, double scall);

    void printImageOrientation();
    void printWorld();
    void printAttention();
    void printTargets(cv::Mat &img);
    void printGrid(cv::Mat &img, cv::Point3d Point1, cv::Point3d Point2, double Step, const cv::Vec3d &Cam, const cv::Matx33d &R, double fx, double cx, double cy);
    void calcCirclePoints(cv::Mat &img, cv::Vec3d center, cv::Vec3b color, cv::Vec3d position, cv::Vec3d orientation);

    cv::Vec6d calcAbweichung(cv::Vec6d Params, cv::Point3d Target);
    cv::Vec6d calcAbweichung(cv::Vec3d Start, cv::Vec3d Orientierung, cv::Vec3d Target);
    cv::Vec6d calcAbweichung(cv::Vec3d Start, cv::Point3f Orientierung, cv::Vec3d Target);

    void newPosition(double colore, cv::Vec6d HeadPoseWorld, cv::Vec6d HeadPose, cv::Point3f GazeDirection0, cv::Point3f GazeDirection1);
    void writeSolutionToFile(QString name, cv::Vec6d Model, cv::Vec6d HeadPoseWorld, cv::Point3f GazeDirection0, cv::Point3f GazeDirection1);
public:
    AtentionTracer(Ui::MainWindow *parent = 0,Camera *cam = 0);
    ~AtentionTracer();
    void print();
    void reset();
    void setImageSize(int Width, int Height);
    void showSolution(QString name, const LandmarkDetector::CLNF &model,
                      double fx, double fy, double cx, double cy, double colore, bool write);
    void setWriteToFile(bool write);
};

#endif // ATENTIONTRACER_H
