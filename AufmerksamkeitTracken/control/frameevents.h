#ifndef FRAMEEVENTS_H
#define FRAMEEVENTS_H

#include <QXmlStreamReader>
#include <QtXml>
#include <QString>

#include <opencv2/opencv.hpp>
#include "model/frame.h"

class FrameEvents
{
private:
    std::vector<Frame> mFrames;

    int filnameToFrame(QString file);
    void boxAttributToValue(QXmlStreamAttributes att, int &height, int &left, int &top, int &width);

    int addFrame(size_t frame);
    void addBox(int id, int x, int y, int w, int h, QString name, QString event);

    void printAll();

public:
    FrameEvents();
    void loadXML(QString path);

    int getFramePos(size_t frame);
    bool getNextFrame(size_t &frame);

    size_t getBoxSizeInFrame(size_t frameID);

    cv::Rect getRect(size_t frameID,size_t boxID);

    bool isNextFrame(size_t frame);
    bool isFrameUsed(size_t frame);

    bool getNextImageFrame(size_t &frame, cv::Rect &rec);

    void clearAll();
};

#endif // FRAMEEVENTS_H
