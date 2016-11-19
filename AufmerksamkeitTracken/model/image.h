#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>

class Image
{
public:
    Image();
    ~Image();
    cv::Mat get_Face_Image(cv::Mat image, int X, int Y, int Wight, int Height);
};

#endif // IMAGE_H

// /home/falko/Uni/Master/Film/Schulklasse_01.mp4
