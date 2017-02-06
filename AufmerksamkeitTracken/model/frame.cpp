#include "frame.h"

Frame::Frame(size_t frame):
    mFrame(frame)
{
    mBoxes.clear();
}

size_t Frame::getSize()
{
    return mBoxes.size();
}

void Frame::addBox(int x, int y, int w, int h)
{
    addBox(cv::Rect(x,y,w,h));
}

void Frame::addBox(cv::Rect rec)
{
    mBoxes.push_back(cv::Rect(rec));
}

void Frame::deleteBox(int p)
{
    if(p >= 0 && p < mBoxes.size())
        mBoxes.erase(mBoxes.begin() + p);
}

size_t Frame::getFrame()
{
    return mFrame;
}

cv::Rect Frame::getBox(int i)
{
    if(i >= 0 && i < mBoxes.size())
        return mBoxes[i];
}
