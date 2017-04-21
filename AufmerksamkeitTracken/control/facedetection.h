#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include "model/camera.h"
#include "model/image.h"
#include "model/imagesection.h"
#include "model/target.h"

#include "LandmarkCoreIncludes.h"
#include "src/algo.h"
#include "control/atentiontracer.h"
#include "control/frameevents.h"
#include "control/printer.h"

#include "ui_mainwindow.h"

class FaceDetection
{
private:
    Ui::MainWindow* mTheWindow;
    void print_FPS_Model(int fps, int model);
    void showImage(const cv::Mat image, const QPixmap &pixmapL, const QPixmap &pixmapR);

    void CalcualteEyes(cv::Mat img, size_t CLNF_ID, int &used, double fx);

    Image mImage;
    int Model_Init;
    int imgCount;

    Target* mTarget;

    AtentionTracer *mAtentionTracer;
    FrameEvents *mFrameEvents;
    Camera* mKamera;
    Printer mPrinter;

    void NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections);

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> clnf_models;
    vector<bool> active_models;
    vector<ImageSection> mImageSections;

    void initCLNF();

    int num_faces_max = 2;

    bool mAutoSize = false;
    bool mUseBox = false;
    bool mUseImage = false;
    bool mCLAHE = true;
    bool mUseEye = true;

    bool getFrame(cv::Mat &img, size_t FrameID);
    bool getFrame(cv::Mat &Img, size_t &Frame, cv::Rect &Rec, std::string &Name, double fx, double fy, double cx, double cy, int x, int y);

public:
    FaceDetection(Ui::MainWindow *parent = 0, FrameEvents *frameEV = 0, Camera* cam = 0);
    ~FaceDetection();
    void FaceTracking();
    void FaceTrackingNewVersion();
    void FaceTrackingImage();

    void ShowFromeFile();

    void setMaxFaces(int i);
    int getMaxFaces();

    void setAutoSize(bool a);
    void setUseBox(bool b);
    void setLearn(bool l);
    void setCLAHE(bool c);
    void setUseEye(bool e);
    void setShowEyes(bool show);

    void setBoxScall(double s);
    void setBoxMinSize(int w, int h);
};

#endif // FACEDETECTION_H
