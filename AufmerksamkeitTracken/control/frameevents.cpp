#include "frameevents.h"
#include <QFile>
#include <QDebug>
#include <QString>

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

void FrameEvents::addBox(int id, int x, int y, int w, int h)
{
    mFrames[id].addBox(x,y,w,h);
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
    std::cout<<"ToDo: Load XML: "<<path.toStdString()<<std::endl;

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
                                    addBox(f_id,left,top,width,height);
                                    xml.skipCurrentElement();
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
        }else{
            std::cout<<"Fehler bei Name dataset: "<<xml.name().toString().toStdString()<<std::endl;
        }
    }else{
        qDebug()<< "Fehler in XML beim öffen von XML: "<<path;
    }

    printAll();
}
