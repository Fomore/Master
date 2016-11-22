#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>

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
};

#endif // IMAGE_H
