#include "box.h"

Box::Box(cv::Rect rec, std::string name, std::string event)
{
    mRect.x = rec.x;
    mRect.y = rec.y;
    mRect.width = rec.width;
    mRect.height = rec.height;
    mName = name;
    mEvent = event;
}

std::string Box::getName()
{
    return mName;
}

std::string Box::getEvent()
{
    return mEvent;
}

cv::Rect Box::getRect()
{
    return mRect;
}
