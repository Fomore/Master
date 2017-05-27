#include "box.h"

void Box::setLandmarks(double land[5][2])
{
    for(int i = 0; i < 5; i++){
        mLandmarks[i][0] = land[i][0];
        mLandmarks[i][1] = land[i][1];
    }
    mIsLandmarksSet = true;
}

void Box::getLandmarks(double land[5][2])
{
    for(int i = 0; i < 5; i++){
        land[i][0] = mLandmarks[i][0];
        land[i][1] = mLandmarks[i][1];
    }
}

bool Box::isLandmark()
{
    return mIsLandmarksSet;
}

int Box::getGaze()
{
    return mGaze;
}

Box::Box(cv::Rect rec, std::string name, std::string event, int gaze)
{
    mRect.x = rec.x;
    mRect.y = rec.y;
    mRect.width = rec.width;
    mRect.height = rec.height;
    mName = name;
    mEvent = event;
    for(int i = 0; i < 5; i++){
        mLandmarks[i][0] = 0;
        mLandmarks[i][1] = 0;
    }
    mIsLandmarksSet = false;
    mGaze = gaze;
}

Box::Box(cv::Rect rec, std::string name, std::string event, double land[5][2], int gaze)
{
    mRect.x = rec.x;
    mRect.y = rec.y;
    mRect.width = rec.width;
    mRect.height = rec.height;
    mName = name;
    mEvent = event;
    setLandmarks(land);
    mGaze = gaze;
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
