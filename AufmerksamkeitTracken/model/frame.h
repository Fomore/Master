#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>
#include "model/box.h"

class Frame
{
private:
    std::vector<Box> mBoxes;
    size_t mFrame;
    std::vector<std::string> mNames;

public:
    Frame(size_t frame);
    size_t getSize();
    void addBox(int x, int y, int w, int h);
    void addBox(int x, int y, int w, int h, std::string name, std::string event, int gaze);
    void addBox(int x, int y, int w, int h, std::string name, std::string event, double land[5][2], int gaze);
    void addBox(cv::Rect rec, std::string name, std::string event, int gaze);
    void addBox(cv::Rect rec, std::string name, std::string event, double land[5][2], int gaze);
    void deleteBox(int p);
    size_t getFrame();
    cv::Rect getBox(size_t i);
    cv::Rect getBox(std::string name, int &gaze);
    void getLandmarks(size_t id, double land[5][2]);
    bool isLandmark(size_t id);
    bool hasEventPart(std::string event, size_t start, size_t size, size_t &pos, int gaze);
    void printAll();
    void clearAll();
    std::string getEvent(int i);
    std::string getName(int i);
    int getGaze(int i);
    bool isGaze(size_t &pos);
};

#endif // FRAME_H
