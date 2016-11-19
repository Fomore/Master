#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include "model/camera.h"
#include "LandmarkCoreIncludes.h"
#include "src/algo.h"

#include "ui_mainwindow.h"

class FaceDetection
{
private:
    Ui::MainWindow* mTheWindow;
    void showImage(const cv::Mat image);
    void showEyeImage(const cv::Mat image, int number, bool right);
    QImage MatToQImage(const cv::Mat& mat);

    Camera* mKamera;
    void print_Eyes(const cv::Mat img,const LandmarkDetector::CLNF& clnf_model);
    void print_Eye(const cv::Mat img, const LandmarkDetector::CLNF& clnf_model, int pos, string name, bool right);
    void print_FPS_Model(int fps, int model);

//    ELSE mElSE;

    void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections);

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> clnf_models;
    vector<bool> active_models;

    int num_faces_max = 4;
public:
    FaceDetection(Ui::MainWindow *parent = 0);
    ~FaceDetection();
    void FaceTracking(std::string path);
};

#endif // FACEDETECTION_H
