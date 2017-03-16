#include "facedetection.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <math.h>

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

using namespace std;

FaceDetection::FaceDetection(Ui::MainWindow *mWindow, FrameEvents *frameEV, Camera *cam)
{
    mTheWindow = mWindow;

    mAtentionTracer = new AtentionTracer(mWindow);
    mFrameEvents = frameEV;
    mKamera = cam;

    Model_Init = 0;
    imgCount = 0;

    initCLNF();
}

FaceDetection::~FaceDetection()
{

}

void FaceDetection::FaceTracking(){
    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    cv::Mat frame_col;
    mKamera->get_camera_params(fx,fy,cx,cy,x,y);
    mAtentionTracer->setImageSize(x,y);

    for(int frame_count = 0;getFrame(frame_col);frame_count++){
        // Reading the images

        cv::Mat_<uchar> grayscale_image;

        cv::Mat disp_image = frame_col.clone();

        Image::convert_to_grayscale(frame_col,grayscale_image);

        vector<cv::Rect_<double> > face_detections;

        bool all_models_active = true;
        for(unsigned int model = 0; model < clnf_models.size(); ++model){
            if(!active_models[model]){
                all_models_active = false;
            }
        }

        // Get the detections (every 8th frame and when there are free models available for tracking) //Nun wird jedes Frame (%1) Berechent
        if(frame_count % 3 == 0 && !all_models_active){
            if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                vector<double> confidences;
                LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[0].face_detector_HOG, confidences);
            }else{
                LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[0].face_detector_HAAR);
            }
        }

        // Keep only non overlapping detections (also convert to a concurrent vector
        NonOverlapingDetections(clnf_models, face_detections);

        vector<tbb::atomic<bool> > face_detections_used(face_detections.size());

        // Go through every model and update the tracking
        tbb::parallel_for(0, (int)clnf_models.size(), [&](int model){
            //for(unsigned int model = 0; model < clnf_models.size(); ++model){

            bool detection_success = false;

            // If the current model has failed more than 4 times in a row, remove it
            if(clnf_models[model].failures_in_a_row > 4){
                active_models[model] = false;
                clnf_models[model].Reset();
            }

            // If the model is inactive reactivate it with new detections
            if(!active_models[model]){
                for(size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind){
                    // if it was not taken by another tracker take it (if it is false swap it to true and enter detection, this makes it parallel safe)
                    if(face_detections_used[detection_ind].compare_and_swap(true, false) == false){
                        // Reinitialise the model
                        clnf_models[model].Reset();

                        // This ensures that a wider window is used for the initial landmark localisation
                        clnf_models[model].detection_success = false;

                        detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, face_detections[detection_ind], clnf_models[model], det_parameters[model]);

                        // This activates the model
                        active_models[model] = true;

                        // break out of the loop as the tracker has been reinitialised
                        break;
                    }
                }
            }else{
                // The actual facial landmark detection / tracking
                detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, clnf_models[model], det_parameters[model]);
            }
        });

        // Go through every model and visualise the results
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        for(size_t model = 0; model < clnf_models.size(); ++model){
            // Visualising the results
            // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
            double detection_certainty = clnf_models[model].detection_certainty; // Qualität der detection: -1 perfekt und 1 falsch

            double visualisation_boundary = -0.1;

            // Only draw if the reliability is reasonable, the value is slightly ad-hoc
            if(detection_certainty < visualisation_boundary){

                printSmallImage(frame_col,model,*painterR,*painterL, false,"");

                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;

                double itens = (detection_certainty + 1)/(visualisation_boundary +1);
                print_CLNF(disp_image,model,itens,fx,fy,cx,cy);

                mAtentionTracer->newPosition((double)model/num_faces_max,
                                             LandmarkDetector::GetCorrectedPoseCamera(clnf_models[model], fx, fy, cx, cy),
                                             clnf_models[model].params_global);
            }
        }
        painterL->end();
        painterR->end();

        // Work out the framerate
        if(frame_count % 10 == 0){
            t1 = cv::getTickCount();
            fps = 10.0 / (double(t1-t0)/cv::getTickFrequency());
            t0 = t1;
        }

        // Write out the framerate on the image before displaying it
        int num_active_models = 0;
        for( size_t active_model = 0; active_model < active_models.size(); active_model++){
            if(active_models[active_model]){
                num_active_models++;
            }
        }

        print_FPS_Model(cvRound(fps),num_active_models);
        showImage(disp_image);
        mTheWindow->Right_Label->setPixmap(*pixmapL);
        mTheWindow->Left_Label->setPixmap(*pixmapR);

        mAtentionTracer->print();

        if(cv::waitKey(30) >= 0)
            return;
    }
}

void FaceDetection::print_CLNF(cv::Mat img, int model, double itens, double fx, double fy, double cx, double cy){
//    LandmarkDetector::Draw(img, clnf_models[model]);

    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(2.0* ((double)img.cols) / 640.0);

    if (clnf_models[model].detection_success && clnf_models[model].eye_model)
    {
        // Gaze tracking, absolute gaze direction
        cv::Point3f gazeDirection0(0, 0, -1);
        cv::Point3f gazeDirection1(0, 0, -1);

//        cv::Mat(gazeDirection0)
        std::cout<<gazeDirection0<<" -> ";

        FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection0, fx, fy, cx, cy, true);
        FaceAnalysis::EstimateGaze(clnf_models[model], gazeDirection1, fx, fy, cx, cy, false);

        std::cout<<gazeDirection0<<gazeDirection1;

        FaceAnalysis::DrawGaze(img, clnf_models[model], gazeDirection0, gazeDirection1, fx, fy, cx, cy);
    }

    // Work out the pose of the head from the tracked model
    cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(clnf_models[model], fx, fy, cx, cy);

    // Draw it in reddish if uncertain, blueish if certain
//    LandmarkDetector::DrawBox(img, pose_estimate, cv::Scalar((1-itens)*255.0,0, itens*255), thickness, fx, fy, cx, cy);

    cv::Matx33d rot = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(pose_estimate[3], pose_estimate[4], pose_estimate[5]));
    std::cout<<"Ausrichtung: "<<rot<<std::endl;

    // Stellt die Gesichtsorientierung dar
    print_Orientation(img,model);
}
//Hier wird die Kopforientierung dargestellt
void FaceDetection::print_Orientation(cv::Mat img, int model){
    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(1.2* ((double)img.cols) / 640.0);
    cv::Vec6d gparam = clnf_models[model].params_global;
    cv::Matx33d rot = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(gparam[1],gparam[2],gparam[3]));
    cv::Vec3d ln = rot*cv::Vec3d(0,0,(-(double)img.cols)/15.0);
    cv::Scalar colore(255/num_faces_max*(num_faces_max-model),255/num_faces_max*model,0);
    cv::arrowedLine(img, cv::Point(gparam[4],gparam[5]),cv::Point(gparam[4]+ln(0),gparam[5]+ln(1)), colore,thickness);
}

// Dieser Teil ist aus OpenFace/FaceLandmarkVidMulti.cpp übernommen
void FaceDetection::NonOverlapingDetections(const vector<LandmarkDetector::CLNF>& clnf_models, vector<cv::Rect_<double> >& face_detections){

    // Go over the model and eliminate detections that are not informative (there already is a tracker there)
    for(size_t model = 0; model < clnf_models.size(); ++model)
    {

        // See if the detections intersect
        cv::Rect_<double> model_rect = clnf_models[model].GetBoundingBox();

        for(int detection = face_detections.size()-1; detection >=0; --detection)
        {
            double intersection_area = (model_rect & face_detections[detection]).area();
            double union_area = model_rect.area() + face_detections[detection].area() - 2 * intersection_area;

            // If the model is already tracking what we're detecting ignore the detection, this is determined by amount of overlap
            if( intersection_area/union_area > 0.5)
            {
                face_detections.erase(face_detections.begin() + detection);
            }
        }
    }
}

void FaceDetection::initCLNF()
{    vector<string> arguments;
     arguments.push_back(""); // Hat arguments keine Werte kann wes wegoptimiert werden und dadurch wirft die Initilaisierung unten Fehler

     /*
     -mloc - the location of landmark detection models
     -sigma -UpdateModelParameters
     -w_reg -
     -reg -
     -multi_view -
     -validate_detections -
     -n_iter -
     -gaze - indicate that gaze estimation should be performed
     -q - specifying to use quiet mode not visualizing output
     -wild - flag specifies when the images are more difficult, model considers extended search regions
     */
     // im build-Ordner muss das model von OpenFace sein
     LandmarkDetector::FaceModelParameters det_params(arguments); // Sollte Parameter beinhalten

      // Always track gaze in feature extraction
      det_params.track_gaze = true;

     det_params.use_face_template = true;
     // This is so that the model would not try re-initialising itself
     det_params.reinit_video_every = -1;

     det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;

     det_parameters.push_back(det_params);

     // Maximale Gesichter im Fil, Achtung: Schnitte und vieles Wechseln scherschlechtert die Qualität
     LandmarkDetector::CLNF clnf_model(det_parameters[0].model_location);
     clnf_model.face_detector_HAAR.load(det_parameters[0].face_detector_location);
     clnf_model.face_detector_location = det_parameters[0].face_detector_location;

     clnf_models.reserve(num_faces_max);

     clnf_models.push_back(clnf_model);
     active_models.push_back(false);

     for (int i = 1; i < num_faces_max; ++i)
     {
         clnf_models.push_back(clnf_model);
         active_models.push_back(false);
         det_parameters.push_back(det_params);
     }

}

void FaceDetection::shift_detected_landmarks(int model, cv::Rect rec, double width)
{
    int X = rec.x, Y = rec.y;
    double fx = (double)width/rec.width;
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        if(width > rec.width && rec.width > 0){
            double x = shape2D.at<double>(pos);
            double y = shape2D.at<double>(pos + n);
            shape2D.at<double>(pos) = X + x/fx;
            shape2D.at<double>(pos + n) = Y + y/fx;
        }else{
            shape2D.at<double>(pos) += X;
            shape2D.at<double>(pos + n) += Y;
        }
    }
    clnf_models[model].detected_landmarks = shape2D.clone();

    clnf_models[model].params_global[4] += X;
    clnf_models[model].params_global[5] += Y;

    for (size_t part = 0; part < clnf_models[model].hierarchical_models.size(); ++part){
        cv::Mat_<double> shape2D = clnf_models[model].hierarchical_models[part].detected_landmarks;

        int n = shape2D.rows/2;
        for(int pos = 0; pos < n; pos++){
            if(width > rec.width && rec.width > 0){
                double x = shape2D.at<double>(pos);
                double y = shape2D.at<double>(pos + n);
                shape2D.at<double>(pos) = X + x/fx;
                shape2D.at<double>(pos + n) = Y + y/fx;
            }else{
                shape2D.at<double>(pos) += X;
                shape2D.at<double>(pos + n) += Y;
            }
        }

        clnf_models[model].hierarchical_models[part].detected_landmarks = shape2D.clone();

        clnf_models[model].hierarchical_models[part].params_global[4] += X;
        clnf_models[model].hierarchical_models[part].params_global[5] += Y;
    }
}

void FaceDetection::showImage(const cv::Mat image){
    QImage img = Image::MatToQImage(image);
    QImage img2 = img.scaled(mTheWindow->Main_Label->size(),Qt::KeepAspectRatio);
    mTheWindow->Main_Label->setPixmap(QPixmap::fromImage(img2));
}

void FaceDetection::getCLNFBox(int model, int pos, int step, double &X, double &Y, double &W, double &H){
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

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

void FaceDetection::getImageSize(double &X, double &Y, double &Width, double &Height, double maxX, double maxY, double sX, double sY, double sMaxX, double sMaxY){
    double fr_X = min(Width*sX, sMaxX);
    double fr_Y = min(Height*sY,sMaxY);
    X -= fr_X;
    Y -= fr_Y;
    Width += fr_X*2;
    Height += fr_Y*2;

    X = max(X,0.0);
    Y = max(Y,0.0);
    Width = min(Width,maxX-X);
    Height = min(Height, maxY-Y);

    X= cvRound(X);
    Y= cvRound(Y);
    Width= cvRound(Width);
    Height= cvRound(Height);

}

void FaceDetection::TestEyeCalculation(cv::Mat img, LandmarkDetector::CLNF &clnf, size_t id)
{
    cv::Rect2d rec = clnf.GetBoundingBox();
    double sx = rec.width*0.3;
    double sy = rec.height*0.4;
    rec.x -= sx/2;
    rec.y -= sy/2;
    rec.width += sx;
    rec.height += sy;

    cv::Mat src = img.clone();
    cv::Mat result;

    LandmarkDetector::Draw(src,clnf);
    cv::vconcat(img(rec),src(rec),result);

    cv::Mat gray;
    src = img(rec).clone();
    Image::convert_to_grayscale(src, gray);
    float quality;
    cv::RotatedRect ellipse = ELSE::run(gray, quality);
    cv::ellipse(src, ellipse, cv::Scalar(0,255,0,255), 1,1 );
    cv::vconcat(result,src,result);

    /* Landmarks der Augen
     * Groß
          2
        1   3
      0       4
        7   5
          6
     * Klein
         25
       26  24
     27      23
       20  22
         21
     */

    src = img.clone();
    cv::Mat_<double> shape2D = clnf.detected_landmarks;
    int n = shape2D.rows/2;
    std::vector<cv::Point2f> pos, pos2;
    for( int i = 0; i < n; ++i){
        if( i < 8){
            pos.push_back(cv::Point2d(shape2D.at<double>(i),shape2D.at<double>(i+n)));
            shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((i-4)*M_PI/4.0)*ellipse.size.width/2*1.727;
            shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((i-4)*M_PI/4.0)*ellipse.size.height/2*1.727;
        }else if(i > 19){
            pos2.push_back(cv::Point2d(shape2D.at<double>(i),shape2D.at<double>(i+n)));
/*            std::cout<<"Alt:"<<shape2D.at<double>(i)<<"/"<<shape2D.at<double>(i + n)
                     <<" ["<<ellipse.center.x<<" "<<ellipse.center.y<<"] ["<<ellipse.size.width<<" "<<ellipse.size.height<<" "<<ellipse.angle
                     <<"] ["<<rec.x<<" "<<rec.y<<" - "<<rec.width<<" "<<rec.height<<"]"<<std::endl;*/
            shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((23-i)*M_PI/4.0)*ellipse.size.width/2*0.738;
            shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((23-i)*M_PI/4.0)*ellipse.size.height/2*0.738;
        }
    }

    cv::RotatedRect ret_ellipse = cv::fitEllipse(pos);
    cv::RotatedRect ret_ellipse2 = cv::fitEllipse(pos2);

    std::ofstream output;
    output.open("Auge.txt", ios::out | ios::app);
    output << "[ "<<ret_ellipse.center.x<<" "<<ret_ellipse.center.y<<" "<<ret_ellipse.size.width<<" "<<ret_ellipse.size.height<<" ]"
           << "[ "<<ret_ellipse2.center.x<<" "<<ret_ellipse2.center.y<<" "<<ret_ellipse2.size.width<<" "<<ret_ellipse2.size.height<<" ]"
           << " "<<clnf.model_likelihood
           << " [ "<<ellipse.center.x<<" "<<ellipse.center.y<<" "<<ellipse.size.width<<" "<<ellipse.size.height<<" ] "<<quality
           <<std::endl;
    output.close();

    clnf.detected_landmarks = shape2D.clone();
    LandmarkDetector::Draw(src,clnf);
//    cv::ellipse(src, ret_ellipse, cv::Scalar(0,255,255,255), 1,1 );
//    cv::ellipse(src, ret_ellipse2, cv::Scalar(0,255,0,255), 1,1 );
    cv::vconcat(result,src(rec),result);

    double f = max(100/rec.width,100/rec.height);
    cv::resize(result,result,cv::Size(f*rec.width,f*rec.height*4));
//    cv::imshow("Auge"+std::to_string(id),result);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    //    cv::imwrite("img/Auge"+std::to_string(id)+".png",result,compression_params);
}

void FaceDetection::CalcualteEye(cv::Mat img, size_t CLNF_ID, int &used)
{
    /*
     * used
     * 0: OpenFace
     * 1: ELSE
     * 2: ELSE & OpenFace
     */
    used = 0;
    int hir_id[2] = {-1,-1};

    for (size_t part = 0; part < clnf_models[CLNF_ID].hierarchical_models.size(); ++part){
        if(clnf_models[CLNF_ID].hierarchical_models[part].detected_landmarks.rows == 56){
            if(hir_id[0] == -1){
                hir_id[0] = part;
            }else{
                hir_id[1] = part;
            }
        }
    }

    for(size_t hir = 0; hir < 2; hir++){
        if(hir_id[hir] >= 0){
            cv::Mat_<double> shape2D = clnf_models[CLNF_ID].hierarchical_models[hir_id[hir]].detected_landmarks;
            int n = shape2D.rows/2;
            cv::Rect2d rec = clnf_models[CLNF_ID].hierarchical_models[hir_id[hir]].GetBoundingBox();
            double sx = rec.width*0.3;
            double sy = rec.height*0.4;
            rec.x -= sx/2;
            rec.y -= sy/2;
            rec.width += sx;
            rec.height += sy;

            std::vector<cv::Point2f> pos, pos2;
            for(size_t i = 20; i < 28; i++){
                pos.push_back(cv::Point2d(shape2D.at<double>(i),shape2D.at<double>(i+n)));
                pos2.push_back(cv::Point2d(shape2D.at<double>(i-20),shape2D.at<double>(i-20+n)));
            }
            cv::RotatedRect min_eye = cv::fitEllipse(pos);
            cv::RotatedRect max_eye = cv::fitEllipse(pos2);
            if((min_eye.size.width+min_eye.size.height)/2 > 4.5){
                cv::Mat gray;
                Image::convert_to_grayscale(img(rec), gray);
                float quality;
                cv::RotatedRect ellipse = ELSE::run(gray, quality);
                if(ellipse.size.width > 0 && ellipse.size.height >0
                        && sqrt(pow(max_eye.center.x - (rec.x + ellipse.center.x),2.0) + pow(max_eye.center.y - (rec.y + ellipse.center.y),2.0))
                        < (max_eye.size.width+max_eye.size.height)/5.0){
                    used += (int)pow(10,hir);
                    if((min_eye.size.width+min_eye.size.height)/2 < 6.0){
                        used += (int)pow(10,hir);
                    }
                    for( int i = 0; i < n; ++i){
                        if( i < 8){
                            if((min_eye.size.width+min_eye.size.height)/2 < 6.0){
                                double x = (shape2D.at<double>(i) + ellipse.center.x+rec.x + cos((i-4)*M_PI/4.0)*ellipse.size.width/2*1.727)/2;
                                double y = (shape2D.at<double>(i+n) + ellipse.center.y+rec.y + sin((i-4)*M_PI/4.0)*ellipse.size.height/2*1.727)/2;
                                shape2D.at<double>(i)  = x;
                                shape2D.at<double>(i+n)= y;
                            }else{
                                shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((i-4)*M_PI/4.0)*ellipse.size.width/2*1.727;
                                shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((i-4)*M_PI/4.0)*ellipse.size.height/2*1.727;
                            }
                        }else if(i > 19){
                            if((min_eye.size.width+min_eye.size.height)/2 < 6.0){
                                double x = (shape2D.at<double>(i) + ellipse.center.x+rec.x + cos((23-i)*M_PI/4.0)*ellipse.size.width/2*0.738)/2;
                                double y = (shape2D.at<double>(i+n) + ellipse.center.y+rec.y + sin((23-i)*M_PI/4.0)*ellipse.size.height/2*0.738)/2;
                                shape2D.at<double>(i)  = x;
                                shape2D.at<double>(i+n)= y;
                            }else{
                                shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((23-i)*M_PI/4.0)*ellipse.size.width/2*0.738;
                                shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((23-i)*M_PI/4.0)*ellipse.size.height/2*0.738;
                            }
                        }
                    }
                }
                clnf_models[CLNF_ID].hierarchical_models[hir_id[hir]].detected_landmarks = shape2D.clone();
            }
        }
    }
}

cv::Mat FaceDetection::print_Eye(const cv::Mat img, int model, int pos, int step, bool clacElse, float &quality){
    double X,Y,Width,Height;
    getCLNFBox(model, pos, step, X,Y,Width,Height);

    getImageSize(X,Y,Width,Height,img.cols, img.rows,0.35,0.4,30,30);

    cv::Mat img_Eye = img(cv::Rect(X,Y,Width,Height));
    if(Width > 8 && Height > 5 && step == 6 && clacElse){
        cv::Mat gray;
        Image::convert_to_grayscale(img_Eye, gray);

        cv::RotatedRect ellipse = ELSE::run(gray, quality);
        cv::ellipse(img_Eye, ellipse, cv::Scalar(0,255,0,255), 1,1 );
    }
    return img_Eye;
}

void FaceDetection::print_FPS_Model(int fps, int model){
    mTheWindow->FPS_Label->setText(QString::number(fps));
    mTheWindow->Model_Label->setText(QString::number(model));
}

void FaceDetection::LearnModel(){
    cv::Mat frame;
    cv::Rect rec;
    std::string name = "";

    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;
//    mKamera->get_camera_params(fx,fy,cx,cy,x,y);

    size_t frm=0;
    int BoxID = -1;
/*
    while(mImage.getNextImage(frame)){
        mKamera->setImageSize(frame.cols,frame.rows);
        mKamera->get_camera_params(fx,fy,cx,cy,x,y);

        rec = mFrameEvents->getRect(mImage.getImageID()-1,0);
        name = "img/"+ std::to_string(mImage.getImageID());

    */
    while(mFrameEvents->getNextImageFrame(frm,rec,name, BoxID)){
        mKamera->getFrame(frame,frm);
        name = "img/F_"+name;
        mKamera->setImageSize(frame.cols,frame.rows);
        mKamera->get_camera_params(fx,fy,cx,cy,x,y);

        cv::Mat frame_col = mImage.get_Face_Image(frame,rec,50);
        cv::Mat disp_image = frame.clone();

        std::cout<<name<<" : "<<frm<<" ["<<rec.x<<" "<<rec.y<<" "<<rec.width<<" "<<rec.height<<"] "<<frame_col.cols<<" "<<frame_col.rows<<std::endl;

        bool success = false;
        cv::Mat_<uchar> grayscale_image,grayIMG;
//        Image::convert_to_grayscale(frame,grayIMG);
        Image::convert_to_grayscale(frame_col,grayIMG);
        for(int clahe = 0; !success && clahe < 2 ; clahe++){
        if(mCLAHE && clahe > 0){
//            Image::CLAHE(grayIMG,grayscale_image,0.875);
            Image::Histogram(grayIMG,grayscale_image);
        }else{
            grayscale_image = grayIMG;
        }

        if(!mLearn || !active_models[Model_Init]){
            // Detect faces in an image
            vector<cv::Rect_<double> > face_detections;

            if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                vector<double> confidences;
                LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[0].face_detector_HOG, confidences);
            }else{
                LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[0].face_detector_HAAR);
            }

            for(size_t face=0; face < face_detections.size(); ++face){
                // if there are multiple detections go through them
                success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_models[Model_Init], det_parameters[Model_Init]);
                //success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, face_detections[face], clnf_models[Model_Init], det_parameters[Model_Init]);
            }
        }else{
            success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, clnf_models[Model_Init], det_parameters[Model_Init]);
        }
        }

        // perform landmark detection for every face detected
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        if(success){
            shift_detected_landmarks(Model_Init,rec,50);

            /*
            for (size_t part = 0; part < clnf_models[Model_Init].hierarchical_models.size(); ++part)
            {
                if(clnf_models[Model_Init].hierarchical_models[part].detected_landmarks.rows == 56){
//                    output << " "<<clnf_models[Model_Init].hierarchical_models[part].model_likelihood;
                    TestEyeCalculation(disp_image.clone(),clnf_models[Model_Init].hierarchical_models[part],mImage.getImageID()*10+part);
                }
            }*/
            int used;
            CalcualteEye(disp_image,Model_Init,used);
            name += "_"+std::to_string(used);

            prinEyeCLNFImage(disp_image,Model_Init,name, false);
            printSmallImage(disp_image.clone(),Model_Init,*painterR,*painterL, false, name);
            print_CLNF(disp_image,Model_Init,0.5,fx,fy,cx,cy);

            mAtentionTracer->newPosition((double)Model_Init/num_faces_max,
                                         LandmarkDetector::GetCorrectedPoseCamera(clnf_models[Model_Init], fx, fy, cx, cy),
                                         clnf_models[Model_Init].params_global);
            mAtentionTracer->print();

            //std::cout<<"Gesicht: "<<clnf_models[0].GetBoundingBox()<<std::endl;
            active_models[Model_Init] = true;

            vector<int> compression_params;
                compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression_params.push_back(9);
            cv::imwrite(name+"_Calc.png",disp_image(rec),compression_params);
        }

        painterL->end();
        painterR->end();

        imgCount++;
        showImage(disp_image);
        mTheWindow->Right_Label->setPixmap(*pixmapL);
        mTheWindow->Left_Label->setPixmap(*pixmapR);

        if(cv::waitKey(30) >= 0)
            return;
    }
    Model_Init= Model_Init%num_faces_max;
}

void FaceDetection::ShowFromeFile()
{
    cv::Mat frame;

    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;
    mKamera->get_camera_params(fx,fy,cx,cy,x,y);

    size_t frm = 0;
    size_t frameID = 0;
    std::string name = "";
    cv::Rect box;
    int i = -1;
    while(mFrameEvents->getNextImageFrame(frm,box,name, i)){
        name = "img/"+name;
//    while(mFrameEvents->getNextFrame(frm,frameID)){
        mKamera->getFrame(frame,frm);
        // perform landmark detection for every face detected
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

//        for(size_t i = 0; i < mFrameEvents->getBoxSizeInFrame(frameID); i++){
//            cv::Rect box = mFrameEvents->getRect(frameID,i);

            printSmallImage(frame,box,i,*painterL, false, name);

            frameID = mFrameEvents->getFramePos(frm);
            if(mFrameEvents->isLandmark(frameID,i)){
                double land[5][2];
                mFrameEvents->getLandmarks(frameID,i,land);
                for(int j = 0; j < 5; j++){
                cv::circle(frame, cv::Point2d(land[j][0],land[j][1]),std::min(5,std::max(1,(box.width+box.height)/70)),cv::Scalar(0,255,0),-1);
                }
            }
            printSmallImage(frame,box,i,*painterR, true, name);

            cv::rectangle(frame,box,cv::Scalar(0,255,0),3);
//        }

        mTheWindow->Right_Label->setPixmap(*pixmapR);
        mTheWindow->Left_Label->setPixmap(*pixmapL);
        showImage(frame);

        if(cv::waitKey(30) >= 0)
            return;
    }
}

void FaceDetection::setMaxFaces(int i)
{
    if(i > 0 && i != num_faces_max){
        num_faces_max = i;
        initCLNF();
    }
}

int FaceDetection::getMaxFaces()
{
    return num_faces_max;
}

void FaceDetection::setAutoSize(bool a)
{
    mAutoSize = a;
}

void FaceDetection::setUseBox(bool b)
{
    mUseBox = b;
}

void FaceDetection::setLearn(bool l)
{
    mLearn = l;
}

void FaceDetection::setCLAHE(bool c)
{
    mCLAHE = c;
}

void FaceDetection::shift_detected_landmarks_toWorld(int model, int worldX, int worldY, int worldW, int worldH, int imgW, int imgH){
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

    double centerWorldX = worldX+worldW/2.0;
    double centerWorldY = worldY+worldH/2.0;

    double centerImgX = imgW/2.0;
    double centerImgY = imgH/2.0;

    double f = (double)worldW/(double)imgW;

    //    std::cout<<"Model tW "<<model<<": "<<worldX<<"/"<<worldY<<" ["<<worldW<<", "<<worldH<<"] -> ["<<imgW<<", "<<imgH<<"] "<<f<<std::endl;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        double pX = shape2D.at<double>(pos);
        double pY = shape2D.at<double>(pos + n);

        shape2D.at<double>(pos) = ((pX-centerImgX)*f)+centerWorldX;
        shape2D.at<double>(pos + n) = ((pY-centerImgY)*f)+centerWorldY;
        //std::cout<<"["<<pX<<", "<<pY<<"] -> ["<<shape2D.at<double>(pos)<<", "<<shape2D.at<double>(pos+n)<<"]"<<std::endl;
    }
    clnf_models[model].detected_landmarks = shape2D.clone();

    clnf_models[model].params_global[4] = ((clnf_models[model].params_global[4]-centerImgX)*f)+centerWorldX;
    clnf_models[model].params_global[5] = ((clnf_models[model].params_global[5]-centerImgY)*f)+centerWorldY;
    //ToDo: Skallierung anpassen
}
void FaceDetection::shift_detected_landmarks_toImage(int model, int worldX, int worldY, int worldW, int worldH, int minSize){
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

    double centerWorldX = worldX+worldW/2.0;
    double centerWorldY = worldY+worldH/2.0;

    double f = 1;
    if(worldW < minSize){
        f = (double)minSize/(double)worldW;
    }

    double centerImgX = worldW*f/2.0;
    double centerImgY = worldH*f/2.0;

    //    std::cout<<"Model tI "<<model<<": "<<worldX<<"/"<<worldY<<" ["<<worldW<<", "<<worldH<<"] -> ["<<centerImgX*2<<", "<<centerImgY*2<<"] "<<f<<std::endl;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        double pX = shape2D.at<double>(pos);
        double pY = shape2D.at<double>(pos + n);

        shape2D.at<double>(pos) = ((pX-centerWorldX)*f)+centerImgX;
        shape2D.at<double>(pos + n) = ((pY-centerWorldY)*f)+centerImgY;
        //std::cout<<"["<<pX<<", "<<pY<<"] -> ["<<shape2D.at<double>(pos)<<", "<<shape2D.at<double>(pos+n)<<"]"<<std::endl;
    }
    clnf_models[model].detected_landmarks = shape2D.clone();

    clnf_models[model].params_global[4] = ((clnf_models[model].params_global[4]-centerWorldX)*f)+centerImgX;
    clnf_models[model].params_global[5] = ((clnf_models[model].params_global[5]-centerWorldY)*f)+centerImgY;
    //ToDo: Skallierung anpassen
}

bool FaceDetection::getFrame(cv::Mat &img)
{
    bool ret =  mKamera->getFrame(img);
    if(ret && mUseBox){
        size_t frame = mKamera->getFrameNr();
        if(mFrameEvents->isFrameUsed(frame)){
            return true;
        }else if(mFrameEvents->getNextFrame(frame)){
            mKamera->setFrame(frame);
            return mKamera->getFrame(img);
        }else{
            return false;
        }
    }
    return ret;
}

void FaceDetection::FaceTrackingAutoSize(){
    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;

    mImageSections.clear();
    for (int i = 0; i < num_faces_max; ++i){
        int x,y,w,h;
        mImage.getFaceParameter(i,x,y,w,h);
        mImageSections.push_back(ImageSection(x,y,w,h));
        active_models[i] = false;
    }

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    cv::Mat frame_colore;
    mKamera->get_camera_params(fx,fy,cx,cy,x,y);
    mAtentionTracer->setImageSize(x,y);

    double minSize = 200;

    for(int frame_count = 0;getFrame(frame_colore);frame_count++){
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        cv::Mat disp_image = frame_colore.clone();

        int num_active_models = 0;

        for(int model = 0; model < num_faces_max; model++){
            int x,y,w,h;
            mImageSections[model].getSection(x,y,w,h);
            // If the current model has failed more than 4 times in a row, remove it
            if(clnf_models[model].failures_in_a_row > 4)
            {
                active_models[model] = false;
                clnf_models[model].Reset();
                double nX = x;
                double nY = y;
                double nW = w;
                double nH = h;
                mImageSections[model].getAvgSection(nX,nY,nW,nH);
                x = cvRound(nX); y = cvRound(nY);
                w = cvRound(nW); h = cvRound(nH);
                //                std::cout<<"Korrektur Lost "<<frame_count<<": "<<model<<std::endl;
            }

            cv::Mat faceImageColore = mImage.get_Face_Image(frame_colore,x,y,w,h,minSize);
            cv::Mat_<uchar> faceImage;
            Image::convert_to_grayscale(faceImageColore,faceImage);

            bool detection_success;

            if(!active_models[model]){
                vector<cv::Rect_<double> > face_detections;
                if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                    vector<double> confidences;
                    LandmarkDetector::DetectFacesHOG(face_detections, faceImage, clnf_models[0].face_detector_HOG, confidences);
                }else{
                    LandmarkDetector::DetectFaces(face_detections, faceImage, clnf_models[0].face_detector_HAAR);
                }
                if(face_detections.size() == 1){
                    // Reinitialise the model
                    clnf_models[model].Reset();

                    // This ensures that a wider window is used for the initial landmark localisation
                    clnf_models[model].detection_success = false;

                    detection_success = LandmarkDetector::DetectLandmarksInVideo(faceImage, face_detections[0], clnf_models[model], det_parameters[model]);

                    // This activates the model
                    active_models[model] = true;
                }
            }else{
                detection_success = LandmarkDetector::DetectLandmarksInVideo(faceImage, clnf_models[model], det_parameters[model]);
            }
            double detection_certainty = clnf_models[model].detection_certainty; // Qualität der detection: -1 perfekt und 1 falsch
            double visualisation_boundary = -0.1;

            // Only draw if the reliability is reasonable, the value is slightly ad-hoc
            double itens = 0;
            if(detection_certainty < visualisation_boundary){
                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;
                itens = (detection_certainty + 1)/(visualisation_boundary +1);

                shift_detected_landmarks_toWorld(model,x,y,w,h,faceImage.cols, faceImage.rows);

                printSmallImage(frame_colore,model,*painterR,*painterL, false,"");
                //                print_CLNF(disp_image,model,itens,fx,fy,cx,cy);
                // Estimate head pose and eye gaze

                mAtentionTracer->newPosition((double)model/num_faces_max,
                                             LandmarkDetector::GetCorrectedPoseCamera(clnf_models[model], fx, fy, cx, cy),
                                             clnf_models[model].params_global);

                print_Orientation(disp_image,model);

                cv::Rect_<double> box = clnf_models[model].GetBoundingBox();

                getImageSize(box.x,box.y,box.width,box.height,frame_colore.cols, frame_colore.rows,0.3,0.4,40,60);

                if(!mImageSections[model].setSection(box.x,box.y,box.width,box.height)){//Wenn die Grenzen überschritten werden
                    active_models[model] = false; //Soll sichergestellt werden, dass nächstesmal ein Gesicht erkannt wird
                    mImageSections[model].getAvgSection(box.x,box.y,box.width,box.height);
                    //                    std::cout<<"Autoreset "<<frame_count<<std::endl;
                }
                shift_detected_landmarks_toImage(model,box.x,box.y,box.width,box.height,minSize);

                num_active_models++;
            }
            cv::rectangle(disp_image,cv::Rect(x,y,w,h),cv::Scalar((1-itens)*255.0,0,itens*255));
        }
        if(frame_count % 10 == 0)
        {
            t1 = cv::getTickCount();
            fps = 10.0 / (double(t1-t0)/cv::getTickFrequency());
            t0 = t1;
        }
        painterL->end();
        painterR->end();

        showImage(disp_image);
        mTheWindow->Right_Label->setPixmap(*pixmapL);
        mTheWindow->Left_Label->setPixmap(*pixmapR);

        print_FPS_Model(cvRound(fps),num_active_models);

        mAtentionTracer->print();

        if(cv::waitKey(30) >= 0) break;
    }
}

void FaceDetection::printSmallImage(cv::Mat img, int model, QPainter &painterR, QPainter &painterL, bool print, std::string titel){
    int sImageW = mTheWindow->Right_Label->size().width();
    int sImageH = mTheWindow->Right_Label->size().height()/num_faces_max;

    float quR, quL; // Qualität des Berechnung

    cv::Mat R = print_Eye(img,model,36,6, true,quR); //Left
    cv::Mat L = print_Eye(img,model,42,6, true,quL); //Right

    if((L.cols > 16 && L.rows > 10) || (R.cols > 16 && R.rows > 10)){
        if((R.cols < 8 || R.rows < 5)){
            R = print_Eye(img,model,0,27, false, quR);
        }else{
            L = print_Eye(img,model,0,27, false, quL);
        }
    }
    if(print){
        vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
//            cv::imwrite(titel+".png",print_Eye(img,model,0,27, false),compression_params);
            cv::imwrite(titel+".png",print_Eye(img,model,36,12, false,quL),compression_params);
    }
    if(L.data){
        QImage img = Image::MatToQImage(L);
        QImage img2 = img.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
        QPixmap pix = QPixmap::fromImage(img2);
        painterL.drawPixmap(0, sImageH*model, pix);
    }
    if(R.data){
        QImage img = Image::MatToQImage(R);
        QImage img2 = img.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
        QPixmap pix = QPixmap::fromImage(img2);
        painterR.drawPixmap(0, sImageH*model, pix);
    }
}

void FaceDetection::printSmallImage(cv::Mat img, cv::Rect rec, int id, QPainter &paint, bool save, std::string titel)
{
    if(save){
        vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
        cv::imwrite(titel+".png",img(rec),compression_params);
    }

    int sImageW = mTheWindow->Right_Label->size().width();
    int sImageH = mTheWindow->Right_Label->size().height()/num_faces_max;

    QImage img1 = Image::MatToQImage(img(rec));
    QImage img2 = img1.scaled(sImageW,sImageH,Qt::KeepAspectRatio);
    QPixmap pix = QPixmap::fromImage(img2);
    paint.drawPixmap(0, sImageH*id, pix);
}

void FaceDetection::prinEyeCLNFImage(cv::Mat img, int model, string titel, bool save)
{
    for(size_t i = 0; i < clnf_models[model].hierarchical_models.size(); ++i)
    {
        if(clnf_models[model].hierarchical_models[i].pdm.NumberOfPoints() != clnf_models[model].hierarchical_mapping[i].size()
                && clnf_models[model].hierarchical_models[i].detected_landmarks.rows == 56){
            int idx = clnf_models[model].patch_experts.GetViewIdx(clnf_models[model].params_global, 0);
            LandmarkDetector::Draw(img, clnf_models[model].hierarchical_models[i].detected_landmarks, clnf_models[model].patch_experts.visibilities[0][idx]);

        }
    }
    if(save){
    float quality;
    vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        cv::imwrite(titel+"E3.png",print_Eye(img,model,36,12, false,quality),compression_params);
    }
}
