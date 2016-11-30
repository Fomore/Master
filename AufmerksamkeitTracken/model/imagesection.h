#ifndef IMAGESECTION_H
#define IMAGESECTION_H


class ImageSection
{
    int X,Y,H,W;
    bool parameterSet;
    int minX, minY, maxX, maxY, minH, maxH, minW, maxW;
    double avgW, avgH;
    void calcAvg(double W, double H);
public:
    ImageSection();
    ImageSection(int x, int y, int w, int h);
    ~ImageSection();
    bool getSection(int &x, int &y, int &w, int &h);
    bool setSection(int x, int y, int w, int h);
    bool setSection(int x, int y, int w, int h, bool set);
    void setSizeMinMax(int miH, int mxH, int miW, int mxW);
    void getSizeMinMax(int &miH, int &mxH, int &miW, int &mxW);
    void setParameterSet(bool set);
    void reset();
    void getAvgSection(double &x, double &y, double &w, double &h);
};

#endif // IMAGESECTION_H
