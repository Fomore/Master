#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>
#include "model/box.h"

class Frame
{
private:
    std::vector<Box> mBoxes;
    size_t mFrame;

public:
    Frame(size_t frame);
    size_t getSize();
    void addBox(int x, int y, int w, int h);
    void addBox(int x, int y, int w, int h, std::string name, std::string event);
    void addBox(cv::Rect rec, std::string name, std::string event);
    void deleteBox(int p);
    size_t getFrame();
    cv::Rect getBox(int i);
    void printAll();
    void clearAll();
};

#endif // FRAME_H
