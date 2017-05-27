#ifndef EVENTHANDLER_H
#define EVENTHANDLER_H

#include <QXmlStreamReader>
#include <QtXml>
#include <QString>

#include <opencv2/opencv.hpp>
#include "model/frame.h"
#include "model/einzelbilder.h"

class EventHandler : public Einzelbilder
{
private:
    std::vector<Frame> mFrames;
    std::vector<QString> mNames;
    bool ignoreName = true;

    int filnameToFrame(QString file);
    void boxAttributToValue(QXmlStreamAttributes att, int &height, int &left, int &top, int &width);

    int addFrame(size_t frame);
    void addBox(int id, int x, int y, int w, int h, QString name, QString event, int gaze);
    void addBox(int id, int x, int y, int w, int h, QString name, QString event, double land[5][2], int gaze);

    void printAll();

    bool existName(QString name);
    bool isImageFrame(size_t frameID, std::string &name, size_t &pos, int gaze);
public:
    EventHandler();
    size_t loadXML(QString path, bool clear);

    int getFramePos(size_t frame);
    bool getNextFrame(size_t &frame);
    bool getFrame(size_t &frame, size_t frameID);

    size_t getBoxSizeInFrame(size_t frameID);
    size_t getNameSize();

    cv::Rect getRect(size_t frameID,size_t boxID);
    cv::Rect getRectWithName(size_t frameID, size_t nameID, int &gaze);
    void getLandmarks(size_t frameID, size_t boxID, double land[5][2]);
    bool isLandmark(size_t frameID, size_t boxID);

    bool isNextFrame(size_t frame);
    bool isFrameUsed(size_t frame);

    bool getNextImageFrame(size_t &frame, cv::Rect &rec, std::string &name, int &id, int gaze);
    bool isImageFrame(size_t frameID, std::string &ImageName, std::string ObjName, int gaze);
    std::string getTitel(size_t frame);
    std::string getTitel(size_t frameID, size_t boxID);
    std::string getName(size_t NameID);

    void clearAll();
};

#endif // EVENTHANDLER_H
