#include "frameevents.h"
#include <QFile>
#include <QDebug>

#include <iostream>

FrameEvents::FrameEvents()
{
    mFrames.clear();
}

int FrameEvents::getFramePos(size_t frame)
{
    size_t fmin = 0; size_t fmax = mFrames.size();
    size_t pos = (fmin+fmax)/2;

    while (fmin < fmax) {
        size_t f = mFrames[pos].getFrame();
        if(f == frame){
            return pos;
        }else if(f > frame){
            fmax = pos;
        }else if(f < frame){
            fmin = pos+1;
        }
        pos = (fmin+fmax)/2;
    }
    return pos;
}

bool FrameEvents::getNextFrame(size_t &frame)
{
    int pos = getFramePos(frame)+1;
    if(pos > 0 && mFrames.size() > pos){
        frame = mFrames[pos].getFrame();
        return true;
    }else{
        return false;
    }
}

bool FrameEvents::getNextFrame(size_t &frame, size_t &frameID)
{
    frameID++;
    if(frameID < mFrames.size()){
        frame = mFrames[frameID].getFrame();
        return true;
    }else{
        return false;
    }
}

size_t FrameEvents::getBoxSizeInFrame(size_t frameID)
{
    return mFrames[frameID].getSize();
}

cv::Rect FrameEvents::getRect(size_t frameID, size_t boxID)
{
    return mFrames[frameID].getBox(boxID);
}

void FrameEvents::getLandmarks(size_t frameID, size_t boxID, double land[5][2])
{
    mFrames[frameID].getLandmarks(boxID,land);
}

bool FrameEvents::isLandmark(size_t frameID, size_t boxID)
{
    return mFrames[frameID].isLandmark(boxID);
}

bool FrameEvents::isNextFrame(size_t frame)
{
    int pos = getFramePos(frame+1);
    return pos >= 0 && frame+1 == mFrames[pos].getFrame();
}

bool FrameEvents::isFrameUsed(size_t frame)
{
    int pos = getFramePos(frame);
    return pos >= 0 && frame == mFrames[pos].getFrame();
}

bool FrameEvents::getNextImageFrame(size_t &frame, cv::Rect &rec, std::string &name, int &id)
{
    for(int i = getFramePos(frame)+1;
        i <= (int)mFrames.size(); i++){
        size_t pos;
        if(mFrames[i].hasEventPart("Img",0,3,pos)){
            cv::Rect r = mFrames[i].getBox(pos);
            rec.x = r.x;
            rec.y = r.y;
            rec.width = r.width;
            rec.height = r.height;
            frame = mFrames[i].getFrame();
            name = mFrames[i].getEvent(pos)+"_"+mFrames[i].getName(pos);
            name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
            id = pos;
            return true;
        }
    }
    return false;
}

std::string FrameEvents::getTitel(size_t frame)
{
    int pos = getFramePos(frame);
    for(size_t i = 0; i < mFrames[pos].getSize(); i++){
        size_t pos;
        if(mFrames[i].hasEventPart("Img",0,3,pos)){
            return mFrames[i].getEvent(pos)+mFrames[i].getName(pos);
        }
    }
    return "NotFoun"+std::to_string(frame);
}

void FrameEvents::clearAll()
{
    for(size_t i = 0; i < mFrames.size(); i++){
        mFrames[i].clearAll();
    }
    mFrames.clear();
}

int FrameEvents::addFrame(size_t frame)
{
    if(mFrames.size() == 0){
        mFrames.push_back(*(new Frame(frame)));
        return 0;
    }else{
        int pos = getFramePos(frame);
        mFrames.insert(mFrames.begin()+pos,*(new Frame(frame)));
        return pos;
    }
}

void FrameEvents::addBox(int id, int x, int y, int w, int h, QString name, QString event)
{
    mFrames[id].addBox(x,y,w,h,name.toStdString(),event.toStdString());
}

void FrameEvents::addBox(int id, int x, int y, int w, int h, QString name, QString event, double land[5][2])
{
    mFrames[id].addBox(x,y,w,h,name.toStdString(),event.toStdString(), land);
}

void FrameEvents::printAll()
{
    for(size_t i = 0; i < mFrames.size(); i++){
        std::cout<<"["<<i<<"] "<<mFrames[i].getFrame()<<std::endl;
        mFrames[i].printAll();
    }
}

int FrameEvents::filnameToFrame(QString file)
{
    QStringList myStringList = file.split('-').last().split('.');
    if(myStringList.size() >=2){
        return myStringList[myStringList.size()-2].toInt();
    }
    return -1;
}

void FrameEvents::boxAttributToValue(QXmlStreamAttributes att, int &height, int &left, int &top, int &width)
{
    height = att.value("height").toInt();
    left = att.value("left").toInt();
    top = att.value("top").toInt();
    width = att.value("width").toInt();
}

void FrameEvents::loadXML(QString path)
{
    clearAll();
    std::cout<<"Load XML: "<<path.toStdString()<<std::endl;

    QXmlStreamReader xml;
    QFile xmlFile(path);
    if(xmlFile.open(QIODevice::ReadOnly)){
        xml.setDevice(&xmlFile);
        if (!xml.hasError() && xml.readNextStartElement() && xml.name() == "dataset"){
            while (xml.readNextStartElement()) {
                if (xml.name() == "images"){
                    while (xml.readNextStartElement()) {
                        if (xml.name() == "image"){
                            QString file = xml.attributes().value("file").toString();
                            int frame = -1;
                            if(!file.isNull())
                                frame = filnameToFrame(file);
                            int f_id = addFrame(frame);
                            while (xml.readNextStartElement()) {
                                if (xml.name() == "box"){
                                    int height, left, top, width;
                                    boxAttributToValue(xml.attributes(),height,left,top,width);
                                    QString name = "";
                                    QString event = "";
                                    bool land = false;
                                    double landmarks[5][2];
                                    while(xml.readNextStartElement()){
                                        if(xml.name() == "label"){
                                            xml.readNext();
                                            name = xml.text().toString();
                                            xml.skipCurrentElement();
                                        }else if(xml.name() == "event"){
                                            event = xml.attributes().value("name").toString();
                                            xml.skipCurrentElement();
                                        }else if(xml.name() == "landmarks"){
                                            land = true;
                                            while (xml.readNextStartElement()) {
                                                if(xml.name() == "point"){
                                                    int p = xml.attributes().value("idx").toInt();
                                                    double x = xml.attributes().value("x").toDouble();
                                                    double y = xml.attributes().value("y").toDouble();
                                                    landmarks[p-1][0]=x;
                                                    landmarks[p-1][1]=y;
                                                }
                                                xml.skipCurrentElement();
                                            }
                                        }else{
                                            xml.skipCurrentElement();
                                        }
                                    }
                                    if(land){
                                        addBox(f_id,left,top,width,height,name,event,landmarks);
                                    }else{
                                        addBox(f_id,left,top,width,height,name,event);
                                    }
                                }else{
                                    xml.skipCurrentElement();
                                }
                            }
                        }else{
                            xml.skipCurrentElement();
                        }
                    }
                }else{
                    xml.skipCurrentElement();
                }
            }
            std::cout<<"Fertig XML"<<std::endl;
        }else{
            std::cout<<"Fehler bei Name dataset: "<<xml.name().toString().toStdString()<<std::endl;
        }
    }else{
        qDebug()<< "Fehler in XML beim öffen von XML: "<<path;
    }
}
