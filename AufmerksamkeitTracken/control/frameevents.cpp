#include "frameevents.h"

#include <iostream>

FrameEvents::FrameEvents()
{
    mFrames.clear();
}

int FrameEvents::getFramePos(size_t frame)
{
    size_t fmin = 0; size_t fmax = mFrames.size()-1;
    while (fmin <= fmax) {
        size_t pos = (fmin+fmax)/2;
        size_t f = mFrames[pos].getFrame();
        if(f == frame){
            return pos;
        }else if(f > frame){
            fmax = pos;
        }else if(f < frame){
            fmin = pos;
        }else if (fmin == fmax) {
            return pos;
        }
    }
    return 0;
}

void FrameEvents::addFrame(size_t frame)
{
    int pos = getFramePos(frame);
    mFrames.insert(mFrames.begin()+pos,*(new Frame(frame)));
}

void FrameEvents::loadXML(std::string path)
{
    std::cout<<"ToDo: Load XML"<<std::endl;
}
