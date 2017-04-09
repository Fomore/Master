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
    std::vector<QString> mNames;

    int filnameToFrame(QString file);
    void boxAttributToValue(QXmlStreamAttributes att, int &height, int &left, int &top, int &width);

    int addFrame(size_t frame);
    void addBox(int id, int x, int y, int w, int h, QString name, QString event);
    void addBox(int id, int x, int y, int w, int h, QString name, QString event, double land[5][2]);

    void printAll();

    bool existName(QString name);
    bool isImageFrame(size_t frameID, std::string &name, size_t &pos);
public:
    FrameEvents();
    size_t loadXML(QString path);

    int getFramePos(size_t frame);
    bool getNextFrame(size_t &frame);
    bool getFrame(size_t &frame, size_t frameID);

    size_t getBoxSizeInFrame(size_t frameID);
    size_t getNameSize();

    cv::Rect getRect(size_t frameID,size_t boxID);
    cv::Rect getRectWithName(size_t frameID,size_t nameID);
    void getLandmarks(size_t frameID, size_t boxID, double land[5][2]);
    bool isLandmark(size_t frameID, size_t boxID);

    bool isNextFrame(size_t frame);
    bool isFrameUsed(size_t frame);

    bool getNextImageFrame(size_t &frame, cv::Rect &rec, std::string &name, int &id);
    bool isImageFrame(size_t frameID, std::string &ImageName, std::string ObjName);
    std::string getTitel(size_t frame);
    std::string getTitel(size_t frameID, size_t boxID);
    std::string getName(size_t NameID);

    void clearAll();
};

#endif // FRAMEEVENTS_H
