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

#include "ui_mainwindow.h"

class FaceDetection
{
private:
    Ui::MainWindow* mTheWindow;
    void showImage(const cv::Mat image);

    cv::Mat print_Eye(const cv::Mat img, int model, int pos, int step, bool clacElse, float &quality);
    void print_FPS_Model(int fps, int model);
    void print_CLNF(cv::Mat img, int model, double itens, double fx, double fy, double cx, double cy);
    void print_Orientation(cv::Mat img, int model);
    void print_SolutionToFile(QString name, int model, double fx, double fy, double cx, double cy);
    void getCLNFBox(int model, int pos, int step, double &X, double &Y, double &W, double &H);

    void printSmallImage(cv::Mat img, int model, QPainter &painterR, QPainter &painterL, bool print, string titel);
    void printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, bool save, string titel);
    void prinEyeCLNFImage(cv::Mat img, int model, string titel, bool save);

    void getImageSize(double &X, double &Y, double &Width, double &Height, double maxX, double maxY, double sX, double sY, double sMaxX, double sMaxY);

    void CalcualteEyes(cv::Mat img, size_t CLNF_ID, int &used);

    cv::Vec2d calcAngle(double X, double Y, double Z);
    cv::Vec2d calcAngle(cv::Vec6d Point);

    cv::Vec6d calcFaceAngle(cv::Vec6d Params_Global);
    cv::Point3d calcAbweichung(cv::Vec6d Params, cv::Point3d Target);

    Image mImage;
    int Model_Init;
    int imgCount;

    Target* mTarget;

    AtentionTracer *mAtentionTracer;
    FrameEvents *mFrameEvents;
    Camera* mKamera;

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
    bool mLearn = false;
    bool mCLAHE = true;
    bool mUseEye = true;

    bool getFrame(cv::Mat &img);
    bool getFrame(cv::Mat &img, size_t FrameID);
    bool getFrame(cv::Mat &Img, size_t &Frame, cv::Rect &Rec, std::string &Name, double fx, double fy, double cx, double cy, int x, int y);

public:
    FaceDetection(Ui::MainWindow *parent = 0, FrameEvents *frameEV = 0, Camera* cam = 0);
    ~FaceDetection();
    void FaceTracking();
    void FaceTrackingAutoSize();
    void LearnModel();

    void ShowFromeFile();

    void setMaxFaces(int i);
    int getMaxFaces();

    void setAutoSize(bool a);
    void setUseBox(bool b);
    void setLearn(bool l);
    void setCLAHE(bool c);
    void setUseEye(bool e);

};

#endif // FACEDETECTION_H
