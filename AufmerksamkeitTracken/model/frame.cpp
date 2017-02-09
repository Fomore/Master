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
    addBox(cv::Rect(x,y,w,h),"","");
}

void Frame::addBox(int x, int y, int w, int h, std::string name, std::string event)
{
    addBox(cv::Rect(x,y,w,h), name, event);
}

void Frame::addBox(cv::Rect rec, std::string name, std::string event)
{
    mBoxes.push_back(*(new Box(rec,name,event)));
}

void Frame::deleteBox(int p)
{
    if(p >= 0 && p < (int)mBoxes.size())
        mBoxes.erase(mBoxes.begin() + p);
}

size_t Frame::getFrame()
{
    return mFrame;
}

cv::Rect Frame::getBox(int i)
{
    if(i >= 0 && i < (int)mBoxes.size())
        return mBoxes[i].getRect();
}

void Frame::printAll()
{
    for(size_t i = 0; i < mBoxes.size(); i++){
        std::cout<<i<<": "<<mBoxes[i].getRect()<<" ";
    }
    std::cout<<std::endl;
}

void Frame::clearAll()
{
    mBoxes.clear();
}
