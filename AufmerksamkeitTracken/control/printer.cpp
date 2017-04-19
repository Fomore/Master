#include "printer.h"

#include <QImage>

#include "src/algo.h"
#include "model/image.h"

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

Printer::Printer()
{
}

void Printer::saveImage(std::string titel, cv::Mat img)
{
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite(titel,img, compression_params);
}

void Printer::getEyeImageSize(double &X, double &Y, double &Width, double &Height, double maxX, double maxY, double sX, double sY, double sMaxX, double sMaxY){
    double fr_X = std::min(Width*sX, sMaxX);
    double fr_Y = std::min(Height*sY,sMaxY);
    X -= fr_X;
    Y -= fr_Y;
    Width += fr_X*2;
    Height += fr_Y*2;

    X = std::max(X,0.0);
    Y = std::max(Y,0.0);
    Width = std::min(Width,maxX-X);
    Height = std::min(Height, maxY-Y);

    X= cvRound(X);
    Y= cvRound(Y);
    Width= cvRound(Width);
    Height= cvRound(Height);
}

void Printer::print_CLNF(cv::Mat img, LandmarkDetector::CLNF &model, double itens, double fx, double fy, double cx, double cy){
    LandmarkDetector::Draw(img, model);

    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(2.0* ((double)img.cols) / 640.0);

    if (model.detection_success && model.eye_model)
    {
        // Gaze tracking, absolute gaze direction
        cv::Point3f gazeDirection0(0, 0, -1);
        cv::Point3f gazeDirection1(0, 0, -1);
        FaceAnalysis::EstimateGaze(model, gazeDirection0, fx, fy, cx, cy, true);
        FaceAnalysis::EstimateGaze(model, gazeDirection1, fx, fy, cx, cy, false);

        FaceAnalysis::DrawGaze(img, model, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
    }

    // Work out the pose of the head from the tracked model
    cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(model, fx, fy, cx, cy);

    // Draw it in reddish if uncertain, blueish if certain
    LandmarkDetector::DrawBox(img, pose_estimate, cv::Scalar((1-itens)*255.0,0, itens*255), thickness, fx, fy, cx, cy);

    // Stellt die Gesichtsorientierung dar
    print_Orientation(img,model);
}

//Hier wird die Kopforientierung dargestellt
void Printer::print_Orientation(cv::Mat img, LandmarkDetector::CLNF &model){
    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(1.2* ((double)img.cols) / 640.0);
    cv::Vec6d gparam = model.params_global;
    cv::Matx33d rot = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(gparam[1],gparam[2],gparam[3]));
    cv::Vec3d ln = rot*cv::Vec3d(0,0,(-(double)img.cols)/15.0);
    cv::Scalar colore(255, 255, 0);
    cv::arrowedLine(img, cv::Point(gparam[4],gparam[5]),cv::Point(gparam[4]+ln(0),gparam[5]+ln(1)), colore,thickness);
}

cv::Mat Printer::getEyeImage(const cv::Mat img, LandmarkDetector::CLNF &model, int pos, int step, bool clacElse, float &quality){
    double X,Y,Width,Height;
    getCLNFBox(model, pos, step, X,Y,Width,Height);

    getEyeImageSize(X,Y,Width,Height,img.cols, img.rows,0.35,0.4,30,30);

    cv::Mat img_Eye = img(cv::Rect(X,Y,Width,Height));
    if(Width > 8 && Height > 5 && step == 6 && clacElse){
        cv::RotatedRect ellipse = ELSE::run(img_Eye, quality);
        cv::ellipse(img_Eye, ellipse, cv::Scalar(0,255,0,255), 1,1 );
    }
    return img_Eye;
}

void Printer::getCLNFBox(LandmarkDetector::CLNF &model, int pos, int step, double &X, double &Y, double &W, double &H){
    cv::Mat_<double> shape2D = model.detected_landmarks;

    int n = shape2D.rows/2;

    X = cvRound(shape2D.at<double>(pos));
    Y = cvRound(shape2D.at<double>(pos + n));
    W = cvRound(shape2D.at<double>(pos));
    H = cvRound(shape2D.at<double>(pos + n));
    for(int i = pos+1; i < pos+step; ++i)// Beginnt bei 0 das Output-Format
    {
        double x = (shape2D.at<double>(i));
        double y = (shape2D.at<double>(i + n));
        X = min(X,x);
        Y = min(Y,y);
        W = max(W,x);
        H = max(H,y);
    }
    W = W-X;
    H = H-Y;
}

void Printer::printSmallImage(cv::Mat img, LandmarkDetector::CLNF &model, QPainter &painterR, QPainter &painterL, bool print, std::string titel, bool drawLandmarks, int sImageW, int sImageH, int pos){
    float quR, quL; // Qualität des Berechnung
    if(drawLandmarks){
        for(size_t i = 0; i < model.hierarchical_models.size(); ++i){
            if(model.hierarchical_models[i].pdm.NumberOfPoints() != model.hierarchical_mapping[i].size()
                    && model.hierarchical_models[i].detected_landmarks.rows == 56){
                int idx = model.patch_experts.GetViewIdx(model.params_global, 0);
                LandmarkDetector::Draw(img, model.hierarchical_models[i].detected_landmarks, model.patch_experts.visibilities[0][idx]);
            }
        }
    }

    cv::Mat R = getEyeImage(img,model,36,6, true,quR); //Left
    cv::Mat L = getEyeImage(img,model,42,6, true,quL); //Right

    if((L.cols > 16 && L.rows > 10) || (R.cols > 16 && R.rows > 10)){
        if((R.cols < 8 || R.rows < 5)){
            R = getEyeImage(img,model,0,27, false, quR);
        }else{
            L = getEyeImage(img,model,0,27, false, quL);
        }
    }
    if(print){
        saveImage(titel+".png",getEyeImage(img,model,36,12, false,quL));
    }
    if(L.data){
        QImage img = Image::MatToQImage(L);
        QImage img2 = img.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
        QPixmap pix = QPixmap::fromImage(img2);
        painterL.drawPixmap(0, sImageH*pos, pix);
    }
    if(R.data){
        QImage img = Image::MatToQImage(R);
        QImage img2 = img.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
        QPixmap pix = QPixmap::fromImage(img2);
        painterR.drawPixmap(0, sImageH*pos, pix);
    }
}

void Printer::printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, bool save, std::string titel, int sImageW, int sImageH)
{
    if(save){
        saveImage(titel+".png",img(rec));
    }

    QImage img1 = Image::MatToQImage(img(rec));
    QImage img2 = img1.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    paint.drawPixmap(0, sImageH*id, pix);
}
