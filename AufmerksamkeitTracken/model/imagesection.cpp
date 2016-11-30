#include "imagesection.h"

ImageSection::ImageSection()
{
    reset();
}

ImageSection::ImageSection(int x, int y, int w, int h){
    setSection(x,y,w,h);
    setSizeMinMax(H/2,H*3,W/2,W*3);
    avgW = w;
    avgH = h;
}

ImageSection::~ImageSection()
{

}

void ImageSection::reset(){
    X = Y = W = H = 0;
    parameterSet = false;
}

bool ImageSection::getSection(int &x, int &y, int &w, int &h){
    x = X;
    y = Y;
    w=W;
    h=H;
    return parameterSet;
}

void ImageSection::setParameterSet(bool set){
    parameterSet = set;
}

bool ImageSection::setSection(int x, int y, int w, int h){
    return setSection(x,y,w,h,true);
}

bool ImageSection::setSection(int x, int y, int w, int h, bool set){
    X=x;
    Y=y;
    W=w;
    H=h;
    parameterSet = set;
    calcAvg(w,h);
    return (minW < W && maxW > W && minH < H && maxH > H);
}

void ImageSection::setSizeMinMax(int miH, int mxH, int miW, int mxW){
    minH = miH;
    maxH = mxH;
    minW = miW;
    maxW = mxW;
}

void ImageSection::getSizeMinMax(int &miH, int &mxH, int &miW, int &mxW){
    miH = minH;
    miW = minW;
    mxW = maxW;
    mxH = maxH;
}

void ImageSection::calcAvg(double W, double H){
    avgH = 0.9*avgH+0.1*H;
    avgW = 0.9*avgW+0.1*W;
}

void ImageSection::getAvgSection(double &x, double &y, double &w, double &h){
    double cX = x+w/2.0;
    double cY = y+h/2.0;
    x = cX-avgW/2.0;
    y = cY-avgH/2.0;
    h = avgH;
    w = avgW;
}
