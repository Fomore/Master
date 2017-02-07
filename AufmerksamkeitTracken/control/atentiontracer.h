#ifndef ATENTIONTRACER_H
#define ATENTIONTRACER_H

#include <opencv2/opencv.hpp>

#include "ui_mainwindow.h"

class AtentionTracer
{
private:
    Ui::MainWindow* mTheWindow;

    std::vector<double> mColores;
    //Positives linkshändiges Koordinatensystem
    std::vector<cv::Vec6d> mCamPose;
    std::vector<cv::Vec6d> mImgPose;

    cv::Size mImageSize;
    cv::Size mWorldSize;
    cv::Size mAtentSize;

    // Position und Orientierung der World-Kamera
    cv::Vec6d mWorldPose;
    // Position und Orientierung der Attention-Kamera
    cv::Vec6d mAttentionPose;

    cv::Point calcArrowEndImage(cv::Vec6d headPose);
    cv::Point calcPose2Image(cv::Vec3d point, cv::Vec6d pose);

    cv::Vec3d unitVector(cv::Vec3d vec);

    void printImageOrientation();
    void printWorld();
    void printAttention();
    void printCirclePoints(cv::Mat &img, cv::Vec3d center, cv::Vec3b color, cv::Vec3d position, cv::Vec3d orientation);
public:
    AtentionTracer(Ui::MainWindow *parent = 0);
    ~AtentionTracer();
    void newPosition(double colore, cv::Vec6d headPoseCam, cv::Vec6d headPoseImg);
    void print();
    void reset();
    void setImageSize(int Width, int Height);
};

#endif // ATENTIONTRACER_H