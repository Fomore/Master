#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>
#include <QtGui>
#include <QDebug>

class Image
{
private:
    std::vector<cv::String> mImagePaths;
    int Image_ID;
public:
    Image();
    ~Image();
    cv::Mat get_Face_Image(cv::Mat image, int X, int Y, int Wight, int Height, double MinSize);
    static void convert_to_grayscale(const cv::Mat& in, cv::Mat& out);
    bool getNextImage(cv::Mat& out);
    bool getImage(cv::Mat& out);
    bool getScallImage(cv::Mat& out);
    static QImage MatToQImage(const cv::Mat& mat);
    static void saveImage(cv::Mat img, std::string name);
    int getImageID();
    void getFaceParameter(int Face_ID, int &X, int &Y, int &Width, int &Hight);
};

#endif // IMAGE_H
