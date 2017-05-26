#ifndef EINZELBILDER_H
#define EINZELBILDER_H

#include <opencv2/opencv.hpp>

class Einzelbilder
{
protected:
    std::vector<cv::String> mImagePaths;
    std::vector<cv::Rect> mRects;
    std::vector<int> mFramePos;

public:
    Einzelbilder();
    ~Einzelbilder();

    bool getImage(cv::Mat& Img, size_t &ImageID, std::string &name, int &FramePos, cv::Rect &Rect);
    void addImage(std::string name, int FramePos, int X, int Y, int Width, int Height);
};

#endif // EINZELBILDER_H
