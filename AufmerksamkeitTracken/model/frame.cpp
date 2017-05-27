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
    addBox(cv::Rect(x,y,w,h),"","",0);
}

void Frame::addBox(int x, int y, int w, int h, std::string name, std::string event, int gaze)
{
    addBox(cv::Rect(x,y,w,h), name, event, gaze);
}

void Frame::addBox(int x, int y, int w, int h, std::string name, std::string event, double land[5][2], int gaze)
{
    addBox(cv::Rect(x,y,w,h), name, event, land, gaze);
}

void Frame::addBox(cv::Rect rec, std::string name, std::string event, int gaze)
{
    mBoxes.push_back(*(new Box(rec,name,event,gaze)));
}

void Frame::addBox(cv::Rect rec, std::string name, std::string event, double land[5][2], int gaze)
{
    mBoxes.push_back(*(new Box(rec,name,event,land, gaze)));
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

cv::Rect Frame::getBox(size_t i)
{
    if(i < mBoxes.size()){
        return mBoxes[i].getRect();
    }else{
        return cv::Rect(0,0,0,0);
    }
}

cv::Rect Frame::getBox(std::string name, int &gaze)
{
    for(size_t i = 0; i < mBoxes.size(); i++){
        if(mBoxes[i].getName() == name){
            gaze = mBoxes[i].getGaze();
            return mBoxes[i].getRect();
        }
    }
    gaze = 0;
    return cv::Rect(0,0,0,0);
}

void Frame::getLandmarks(size_t id, double land[5][2])
{
    if(mBoxes.size() > id){
        mBoxes[id].getLandmarks(land);
    }
}

bool Frame::isLandmark(size_t id)
{
    return mBoxes[id].isLandmark();
}

bool Frame::hasEventPart(std::string event, size_t start, size_t size, size_t &pos, int gaze)
{
    for(size_t i = 0; i < mBoxes.size(); i++){
        if(mBoxes[i].getEvent().compare(start,size,event) == 0
                || (gaze > 0 && mBoxes[i].getGaze() > gaze)){
            pos = i;
            return true;
        }
    }
    return false;
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

std::string Frame::getEvent(int i)
{
    if(i >= 0 && i < (int)mBoxes.size())
        return mBoxes[i].getEvent();
}

std::string Frame::getName(int i)
{
    if(i >= 0 && i < (int)mBoxes.size())
        return mBoxes[i].getName();
}

int Frame::getGaze(int i)
{
    if(i >= 0 && i < (int)mBoxes.size())
        return mBoxes[i].getGaze();
}

bool Frame::isGaze(size_t &pos)
{
    for(size_t i = 0; i < mBoxes.size(); i++){
        if(mBoxes[i].getGaze() > 0){
            pos = i;
            return true;
        }
    }
    return false;
}
