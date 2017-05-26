#ifndef PRINTER_H
#define PRINTER_H

#include "LandmarkCoreIncludes.h"
#include <opencv2/opencv.hpp>
#include <QPainter>

class Printer
{
private:
    void printMatToQPainter(cv::Mat Img, QPainter &Paint, int Width, int Height, int Position);
    bool mDrawEyeLandmarks = false;
    bool mDrawLandmarks = true;
    bool mSaveImage = false;
public:
    Printer();
    void saveImage(std::string titel, cv::Mat img);
    void getEyeImageSize(double &X, double &Y, double &Width, double &Height, double maxX, double maxY, double sX, double sY, double sMaxX, double sMaxY);
    void print_CLNF(cv::Mat img, const LandmarkDetector::CLNF &model, double itens, double fx, double fy, double cx, double cy, double colore);
    void print_Orientation(cv::Mat img, const LandmarkDetector::CLNF &model, double colore);
    cv::Mat getEyeImage(const cv::Mat img, const LandmarkDetector::CLNF &model, int pos, int step);
    void getCLNFBox(const LandmarkDetector::CLNF &model, int pos, int step, double &X, double &Y, double &W, double &H);
    void printSmallImage(cv::Mat img, const LandmarkDetector::CLNF &model, QPainter &painterR, QPainter &painterL, std::string titel,
                         int sImageW, int sImageH, int pos);
    void printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, std::string titel, int sImageW, int sImageH);
    void setShowEye(bool show);
    void setSaveImage(bool save);
    void setDrawLandmarks(bool landmark);
    bool isSaveImage();
};

#endif // PRINTER_H
