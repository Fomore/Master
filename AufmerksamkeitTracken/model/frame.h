#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

class Frame
{
private:
    std::vector<cv::Rect> mBoxes;
    size_t mFrame;
public:
    Frame(size_t frame);
    size_t getSize();
    void addBox(int x, int y, int w, int h);
    void addBox(cv::Rect rec);
    void deleteBox(int p);
    size_t getFrame();
    cv::Rect getBox(int i);
};

#endif // FRAME_H
