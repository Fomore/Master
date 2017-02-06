#ifndef FRAMEEVENTS_H
#define FRAMEEVENTS_H

#include <opencv2/opencv.hpp>
#include "model/frame.h"

class FrameEvents
{
private:
    std::vector<Frame> mFrames;

    int getFramePos(size_t frame);
    void addFrame(size_t frame);
public:
    FrameEvents();
    void loadXML(std::string path);
};

#endif // FRAMEEVENTS_H
