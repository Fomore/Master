#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>
#include <QtGui>
#include <QDebug>

class Image
{
private:
    std::vector<std::string> mImagePaths;
    int ID;
public:
    Image();
    ~Image();
    cv::Mat get_Face_Image(cv::Mat image, int X, int Y, int Wight, int Height);
    static void convert_to_grayscale(const cv::Mat& in, cv::Mat& out);
    bool getNextImage(cv::Mat& out);
    bool getImage(cv::Mat& out);
    bool getScallImage(cv::Mat& out);
    static QImage MatToQImage(const cv::Mat& mat);
    static void saveImage(cv::Mat img, std::string name);
    int getID();
};

#endif // IMAGE_H
