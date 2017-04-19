#ifndef PRINTER_H
#define PRINTER_H

#include "LandmarkCoreIncludes.h"
#include <opencv2/opencv.hpp>
#include <QPainter>

class Printer
{
public:
    Printer();
    void saveImage(std::string titel, cv::Mat img);
    void getEyeImageSize(double &X, double &Y, double &Width, double &Height, double maxX, double maxY, double sX, double sY, double sMaxX, double sMaxY);
    void print_CLNF(cv::Mat img, LandmarkDetector::CLNF &model, double itens, double fx, double fy, double cx, double cy);
    void print_Orientation(cv::Mat img, LandmarkDetector::CLNF &model);
    cv::Mat getEyeImage(const cv::Mat img, LandmarkDetector::CLNF &model, int pos, int step, bool clacElse, float &quality);
    void getCLNFBox(LandmarkDetector::CLNF &model, int pos, int step, double &X, double &Y, double &W, double &H);
    void printSmallImage(cv::Mat img, LandmarkDetector::CLNF &model, QPainter &painterR, QPainter &painterL, bool print, std::string titel, bool drawLandmarks,
                         int sImageW, int sImageH, int pos);
    void printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, bool save, std::string titel, int sImageW, int sImageH);
};

#endif // PRINTER_H
