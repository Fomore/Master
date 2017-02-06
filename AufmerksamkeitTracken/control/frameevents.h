#ifndef FRAMEEVENTS_H
#define FRAMEEVENTS_H

#include <QXmlStreamReader>
#include <QtXml>

#include <opencv2/opencv.hpp>
#include "model/frame.h"

class FrameEvents
{
private:
    std::vector<Frame> mFrames;

    int filnameToFrame(QString file);
    void boxAttributToValue(QXmlStreamAttributes att, int &height, int &left, int &top, int &width);

    int addFrame(size_t frame);
    void addBox(int id, int x, int y, int w, int h);

    void printAll();
public:
    FrameEvents();
    void loadXML(QString path);

    int getFramePos(size_t frame);

    size_t getBoxSizeInFrame(size_t frameID);

    cv::Rect getRect(size_t frameID,size_t boxID);
};

#endif // FRAMEEVENTS_H
