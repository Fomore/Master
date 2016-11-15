#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include "model/camera.h"
#include "LandmarkCoreIncludes.h"

class FaceDetection
{
private:
    Camera* mKamera;
    void print_Eyes(const cv::Mat img,const LandmarkDetector::CLNF& clnf_model);
    void print_Eye(const cv::Mat img,const LandmarkDetector::CLNF& clnf_model, int pos, string name);

    void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections);

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> clnf_models;
    vector<bool> active_models;

public:
    FaceDetection();
    ~FaceDetection();
    void test();
};

#endif // FACEDETECTION_H
