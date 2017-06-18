#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>
#include <QtGui>
#include <QDebug>

class Image
{
public:
    Image();
    ~Image();

    static QImage MatToQImage(const cv::Mat& mat);
    static void CLAHE(cv::Mat in, cv::Mat &out, double clip);
    static void Histogram(cv::Mat in, cv::Mat &out);
    static void convert_to_grayscale(const cv::Mat& in, cv::Mat& out);
    static void toGrayGleam(const cv::Mat &Image, cv::Mat &Out);
    static cv::Mat normalize(cv::Mat GrayImage, cv::Mat Image);
};

#endif // IMAGE_H
