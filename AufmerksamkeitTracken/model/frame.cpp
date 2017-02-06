#include "frame.h"
#include <iostream>

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

void Frame::printAll()
{
    for(size_t i = 0; i < mBoxes.size(); i++){
        std::cout<<i<<": "<<mBoxes[i]<<" ";
    }
    std::cout<<std::endl;
}
