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

void Printer::print_CLNF(cv::Mat img,const LandmarkDetector::CLNF &model, double itens, double fx, double fy, double cx, double cy, double colore){
    //LandmarkDetector::Draw(img, model);

    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(2.0* ((double)img.cols) / 640.0);

    if (model.detection_success && model.eye_model)
    {
        // Gaze tracking, absolute gaze direction
        cv::Point3f gazeDirection0(0, 0, -1);
        cv::Point3f gazeDirection1(0, 0, -1);
        FaceAnalysis::EstimateGaze(model, gazeDirection0, fx, fy, cx, cy, true);
        FaceAnalysis::EstimateGaze(model, gazeDirection1, fx, fy, cx, cy, false);

        if(mUseAVGEye){
            cv::Point3f Solution(gazeDirection0.x+gazeDirection1.x,
                                 gazeDirection0.y+gazeDirection1.y,
                                 gazeDirection0.z+gazeDirection1.z);
            double n = sqrt(Solution.x*Solution.x
                           +Solution.y*Solution.y
                           +Solution.z*Solution.z);
            if(n > 0){
                Solution.x = Solution.x/n;
                Solution.y = Solution.y/n;
                Solution.z = Solution.z/n;
            }
            FaceAnalysis::DrawGaze(img, model, Solution, Solution, fx, fy, cx, cy);
        }else{
            FaceAnalysis::DrawGaze(img, model, gazeDirection0, gazeDirection1, fx, fy, cx, cy);
        }
    }

    // Work out the pose of the head from the tracked model
    cv::Vec6d pose_estimate = LandmarkDetector::GetPoseWorld(model, fx, fy, cx, cy);

    if(mShowHeadBox){
        // Draw it in reddish if uncertain, blueish if certain
        LandmarkDetector::DrawBox(img, pose_estimate, cv::Scalar((1-itens)*255.0,0, itens*255), thickness, fx, fy, cx, cy);
    }

    // Stellt die Gesichtsorientierung dar
    print_Orientation(img,model, colore);
}

//Hier wird die Kopforientierung dargestellt
void Printer::print_Orientation(cv::Mat img, const LandmarkDetector::CLNF &model, double colore){
    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(1.2* ((double)img.cols) / 640.0);
    cv::Vec6d gparam = model.params_global;
    cv::Matx33d rot = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(gparam[1],gparam[2],gparam[3]));
    cv::Vec3d ln = rot*cv::Vec3d(0,0,(-(double)img.cols)/15.0);
    cv::Scalar Colore(255.0*(1.0-colore),255.0*colore,255.0*(1.0-colore));
    cv::arrowedLine(img, cv::Point(gparam[4],gparam[5]),cv::Point(gparam[4]+ln(0),gparam[5]+ln(1)), Colore,thickness);
}

cv::Mat Printer::getEyeImage(const cv::Mat img,const LandmarkDetector::CLNF &model, int pos, int step){
    double X,Y,Width,Height;
    getCLNFBox(model, pos, step, X,Y,Width,Height);

    getEyeImageSize(X,Y,Width,Height,img.cols, img.rows,0.35,0.4,30,30);

    cv::Mat img_Eye = img(cv::Rect(X,Y,Width,Height));
    return img_Eye;
}

void Printer::getCLNFBox(const LandmarkDetector::CLNF &model, int pos, int step, double &X, double &Y, double &W, double &H){
    cv::Mat_<double> shape2D = model.detected_landmarks;

    int n = shape2D.rows/2;

    X = cvRound(shape2D.at<double>(pos));
    Y = cvRound(shape2D.at<double>(pos + n));
    W = cvRound(shape2D.at<double>(pos));
    H = cvRound(shape2D.at<double>(pos + n));
    for(int i = std::max(0,pos-1); i < std::min(pos-1+step,n); ++i)// Beginnt bei 0 das Output-Format
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

void Printer::printSmallImage(cv::Mat img, const LandmarkDetector::CLNF &model, QPainter &painterR, QPainter &painterL,
                              std::string titel, int sImageW, int sImageH, int pos){
    cv::Mat R;
    cv::Mat L;
    if(mDrawEyeLandmarks){
        if(mDrawLandmarks){
            for(size_t i = 0; i < model.hierarchical_models.size(); ++i){
                if(model.hierarchical_models[i].pdm.NumberOfPoints() != model.hierarchical_mapping[i].size()
                        && model.hierarchical_models[i].detected_landmarks.rows == 56){
                    int idx = model.patch_experts.GetViewIdx(model.params_global, 0);
                    LandmarkDetector::Draw(img, model.hierarchical_models[i].detected_landmarks, model.patch_experts.visibilities[0][idx]);
                }
            }
        }
        L = getEyeImage(img,model,37,6); //Left
        R = getEyeImage(img,model,43,6); //Right
    }else{
        double X,Y,Width,Height;
        getCLNFBox(model, 1, 68, X,Y,Width,Height);
        //std::cout<<titel<<": "<<Width<<"/"<<Height<<" "<<model.params_global[0]<<std::endl;
        double f = min((double)sImageH/Height, (double)sImageW/Width);
        cv::resize(img(cv::Rect(X,Y,Width,Height)),R,cv::Size(0,0),f,f);

        if(mDrawLandmarks){
        LandmarkDetector::Draw(img,model);
        }else{
            print_Orientation(img,model,1);
        }

        cv::resize(img(cv::Rect(X,Y,Width,Height)),L,cv::Size(0,0),f,f);
    }

    if(L.data){
        printMatToQPainter(L,painterL,sImageW,sImageH,pos);
    }
    if(R.data){
        printMatToQPainter(R,painterR,sImageW,sImageH,pos);
    }

    if(mSaveImage){
        saveImage(titel,getEyeImage(img,model,37,12));
    }
}

void Printer::printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, std::string titel, int sImageW, int sImageH)
{
    if(mSaveImage){
        saveImage(titel+"_Small",img(rec));
    }
    printMatToQPainter(img(rec),paint,sImageW,sImageH,id);
}

void Printer::setShowEye(bool show)
{
    mDrawEyeLandmarks = show;
}

void Printer::setSaveImage(bool save)
{
    mSaveImage = save;
}

void Printer::setDrawLandmarks(bool landmark)
{
    mDrawLandmarks = landmark;
}

bool Printer::isSaveImage()
{
    return mSaveImage;
}

void Printer::setShowHeadBox(bool h)
{
    mShowHeadBox = h;
}

void Printer::setUseAVGEye(bool e)
{
    mUseAVGEye = e;
}

void Printer::printMatToQPainter(cv::Mat Img, QPainter &Paint, int Width, int Height, int Position)
{
    QImage img = Image::MatToQImage(Img);
    QImage img2 = img.scaled(Width,Height,Qt::KeepAspectRatio,Qt::SmoothTransformation);
    QPixmap pix = QPixmap::fromImage(img2);
    Paint.drawPixmap(0, Height*Position, pix);
}

void Printer::saveImage(std::string titel, cv::Mat img)
{
    std::cout<<"Speichen: "<<titel<<std::endl;
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite(titel+".png",img, compression_params);
}
