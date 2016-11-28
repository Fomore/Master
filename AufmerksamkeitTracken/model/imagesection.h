#ifndef IMAGESECTION_H
#define IMAGESECTION_H


class ImageSection
{
    int X,Y,H,W;
    bool parameterSet;
public:
    ImageSection();
    ImageSection(int x, int y, int w, int h);
    ~ImageSection();
    bool getSection(int &x, int &y, int &w, int &h);
    void setSection(int x, int y, int w, int h);
    void setSection(int x, int y, int w, int h, bool set);
    void setParameterSet(bool set);
    void reset();
};

#endif // IMAGESECTION_H
