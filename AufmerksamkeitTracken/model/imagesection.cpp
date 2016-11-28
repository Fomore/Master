#include "imagesection.h"

ImageSection::ImageSection()
{
    reset();
}

ImageSection::ImageSection(int x, int y, int w, int h){
    setSection(x,y,w,h);
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

void ImageSection::setSection(int x, int y, int w, int h){
    setSection(x,y,w,h,true);
}

void ImageSection::setSection(int x, int y, int w, int h, bool set){
    X=x;
    Y=y;
    W=w;
    H=h;
    parameterSet = set;
}
