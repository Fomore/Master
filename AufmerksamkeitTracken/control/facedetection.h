#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include "model/camera.h"
#include "model/image.h"
#include "model/imagesection.h"
#include "LandmarkCoreIncludes.h"
#include "src/algo.h"

#include "ui_mainwindow.h"

class FaceDetection
{
private:
    Ui::MainWindow* mTheWindow;
    void showImage(const cv::Mat image);

    Camera* mKamera;
    cv::Mat print_Eye(const cv::Mat img, int model, int pos, int step, bool clacElse);
    void print_FPS_Model(int fps, int model);
    void print_CLNF(cv::Mat img, int model, double itens, double fx, double fy, double cx, double cy);
    void getCLNFBox(int model, int pos, int step, double &X, double &Y, double &W, double &H);

    Image mImage;
    int Model_Init;
    int imgCount;

    void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections);

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> clnf_models;
    vector<bool> active_models;
    vector<ImageSection> mImageSections;

    int num_faces_max = 7;

    void shift_detected_landmarks(int model, double X, double Y, double w, double h, double W, double H);

public:
    FaceDetection(Ui::MainWindow *parent = 0);
    ~FaceDetection();
    void FaceTracking(std::string path);
    void FaceTrackingAutoSize(std::string path);
    void LearnModel();
};

#endif // FACEDETECTION_H
