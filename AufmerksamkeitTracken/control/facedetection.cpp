#include "facedetection.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <math.h>
#include <string>

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

using namespace std;

FaceDetection::FaceDetection(Ui::MainWindow *mWindow, Camera *cam)
{
    mTheWindow = mWindow;

    QString TFName = "/home/falko/Uni/Master/AufmerksamkeitTracken/data/Target_1.txt";
    mAtentionTracer = new AtentionTracer(mWindow,cam,TFName);
    mEventHandler = new EventHandler();
    mKamera = cam;

    Model_Init = 0;
    imgCount = 0;

    initCLNF();
}

FaceDetection::~FaceDetection()
{

}

void FaceDetection::FaceTracking(){
    if(mAtentionTracer->getUseTime()){
        mKamera->setFrame(mEventHandler->mVideoObservationStart);
        mAtentionTracer->mVideoTimeShift = mKamera->getTimeSec();
        mKamera->setFrame(mEventHandler->mVideoObservationStart-15);
    }

    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    cv::Mat frame_col;
    mKamera->get_camera_params(fx,fy,cx,cy,x,y);
    mAtentionTracer->setImageSize(x,y);

    std::cout<<"FaceTracking"<<std::endl;
    for(int FrameID = 0;getFrame(frame_col,FrameID);FrameID++){
        // Reading the images

        cv::Mat_<uchar> grayscale_image;

        cv::Mat disp_image = frame_col.clone();

        Image::convert_to_grayscale(frame_col,grayscale_image);
        //Image::toGrayGleam(frame_col,grayscale_image);

        vector<cv::Rect_<double> > face_detections;

        bool all_models_active = true;
        for(unsigned int model = 0; model < clnf_models.size(); ++model){
            if(!active_models[model]){
                all_models_active = false;
            }
        }

        // Get the detections (every 8th frame and when there are free models available for tracking) //Nun wird jedes Frame (%1) Berechent
        if(FrameID % 3 == 0 && !all_models_active){
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

        /*
         * bei thiago.avi
        bool manuelReset = mKamera->getFrameNr() == 784 || mKamera->getFrameNr() == 2137
                || mKamera->getFrameNr() == 2143;
        */
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
            double colore = (double)model/num_faces_max;

            // Only draw if the reliability is reasonable, the value is slightly ad-hoc
            if(detection_certainty < visualisation_boundary){

                mPrinter.printSmallImage(frame_col,clnf_models[model],*painterR,*painterL,"Frame_"+std::to_string(FrameID),
                                         mTheWindow->Right_Label->size().width(), mTheWindow->Right_Label->size().height()/num_faces_max, model);

                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;

                double itens = (detection_certainty + 1)/(visualisation_boundary +1);
                mPrinter.print_CLNF(disp_image,clnf_models[model],itens,fx,fy,cx,cy,colore);
                if(mAtentionTracer->getUseTime()){
                    mAtentionTracer->showSolution("",mKamera->getFrameNr(),clnf_models[model],fx,fy,cx,cy, colore, true);
                }else{
                mAtentionTracer->showSolution("",FrameID,clnf_models[model],fx,fy,cx,cy, colore, true);
                }
            }
        }
        painterL->end();
        painterR->end();

        // Work out the framerate
        if(FrameID % 10 == 0){
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
        showImage(disp_image, *pixmapL, *pixmapR);

        mAtentionTracer->print();

        if(cv::waitKey(30) >= 0)
            return;
    }
}

void FaceDetection::FaceTrackingNewVersion(){
    std::cout<<"AutoSize Tracking"<<std::endl;
    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;
    mKamera->get_camera_params(fx,fy,cx,cy,x,y);
    mAtentionTracer->setImageSize(x,y);

    for (int i = 0; i < num_faces_max; ++i){
        mBoxHandlers[i].setImageSize(x,y);
        active_models[i] = false;
        clnf_models[i].Reset();
    }
    if(mAtentionTracer->getUseTime()){
        mKamera->setFrame(mEventHandler->mVideoObservationStart);
        mAtentionTracer->mVideoTimeShift = mKamera->getTimeSec();
    }

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    size_t countFrame = 0, countFound = 0;

    cv::Mat frame_colore;

    for(size_t FrameID = 0;getFrame(frame_colore, FrameID);FrameID++){//mEventHandler->getFramePos(1350)
        //std::cout<<FrameID<<": "<<mKamera->getFrameNr()<<"["<<mKamera->getTimeSec()<<"]"<<std::endl;
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        cv::Mat disp_image = frame_colore.clone();

        int num_active_models = 0;

        //bool manuelReset = mKamera->getFrameNr() == 784 || mKamera->getFrameNr() == 2137 || mKamera->getFrameNr() == 2143 || mKamera->getFrameNr() == 2150;

        for(int model = 0; model < num_faces_max; model++){
            int gaze;
            countFrame++;
            mBoxHandlers[model].setNewRect(mKamera->correct_Rect(mEventHandler->getRectWithName(FrameID,model,gaze)));

            if(mBoxHandlers[model].existBox()){
                // If the current model has failed more than 4 times in a row, remove it
                if(clnf_models[model].failures_in_a_row > 4)
                {
                    active_models[model] = false;
                    clnf_models[model].Reset();
                    mBoxHandlers[model].setOldRect(cv::Rect2d(0,0,0,0));
                }
                cv::Mat faceImageColore;
                mBoxHandlers[model].getImage(frame_colore,faceImageColore);
                //cv::imshow("Gesicht "+std::to_string(model),faceImageColore);

                //cv::Mat faceImageColore = disp_image(cv::Rect(x,y,w,h));
                cv::Mat_<uchar> faceImage;
                Image::convert_to_grayscale(faceImageColore,faceImage);
                mBoxHandlers[model].toSection(clnf_models[model]);

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

                mBoxHandlers[model].toImage(clnf_models[model]);

                double colore = (double)model/num_faces_max;

                // Only draw if the reliability is reasonable, the value is slightly ad-hoc
                double itens = 0;
                if(detection_certainty < visualisation_boundary){
                    if(detection_certainty > 1)
                        detection_certainty = 1;
                    if(detection_certainty < -1)
                        detection_certainty = -1;
                    itens = (detection_certainty + 1)/(visualisation_boundary +1);

                    mBoxHandlers[model].setOldRect(clnf_models[model].GetBoundingBox());

                    int used;
                    CalcualteEyes(frame_colore.clone(),model,used,mBoxHandlers[model].getImageScall());
                    std::string name;

                    bool isImageFrame = mEventHandler->isImageFrame(FrameID,name,mEventHandler->getName(model),mGaze);
                    if(isImageFrame){
                        std::cout<<mKamera->getFrameNr()<<": "<<model<<" "<<name<<mBoxHandlers[model].getRect()<<std::endl;
                    }

                    mPrinter.printSmallImage(frame_colore,clnf_models[model],*painterR,*painterL, "img/Head_"+name,
                                             mTheWindow->Right_Label->size().width(), mTheWindow->Right_Label->size().height()/num_faces_max, model);

                    // Estimate head pose and eye gaze
                    if(mAtentionTracer->getUseTime()){
                        mAtentionTracer->showSolution(QString::number(mKamera->getTimeSec()),FrameID,
                                                      clnf_models[model],fx,fy,cx,cy, colore,
                                                      mKamera->getFrameNr() >= mEventHandler->mVideoObservationStart);

                    }else{
                        mAtentionTracer->showSolution(QString::fromStdString(name),FrameID,
                                                      clnf_models[model],fx,fy,cx,cy, colore,
                                                      isImageFrame || gaze > 1);
                    }

                    mPrinter.print_CLNF(disp_image,clnf_models[model],0.5,fx,fy,cx,cy, colore);

                    num_active_models++;
                    countFound++;
                }else{

                }

                if(gaze >1){
                    std::cout<<"Gaze: "<<FrameID<<" "<<model<<std::endl;
                }
                if(mShowImageBox){
                    cv::rectangle(disp_image,mBoxHandlers[model].getRect(),
                                  cv::Scalar(255.0*(1.0-colore),255.0*colore,255.0*(1.0-colore)),2);
                }
            }
        }
        if(FrameID % 10 == 0)
        {
            t1 = cv::getTickCount();
            fps = 10.0 / (double(t1-t0)/cv::getTickFrequency());
            t0 = t1;
        }
        painterL->end();
        painterR->end();

        showImage(disp_image, *pixmapL, *pixmapR);

        print_FPS_Model(cvRound(fps),num_active_models);

        mAtentionTracer->print();

        if(cv::waitKey(30) >= 0) break;
    }
    std::cout<<"Treffer: "<<countFound<<"/"<<countFrame<<std::endl;
}

void FaceDetection::FaceTrackingImage(){
    cv::Mat frame;
    cv::Rect rec;
    std::string name = "";

    // Initialisiierung
    double fx,fy,cx,cy;
    int x,y;

    size_t frm=0;
    std::cout<<"Bild berechung"<<std::endl;
    while(getFrame(frame,frm,rec,name,fx,fy,cx,cy,x,y)){
        mAtentionTracer->setImageSize(x,y);
        mBoxHandlers[0].setImageSize(x,y);

        cv::Mat disp_image = frame.clone();

        bool success = false;
        cv::Mat_<uchar> grayscale_image;
        //Image::convert_to_grayscale(frame_col,grayIMG);
        if(rec.width > 0 && rec.height > 0){
            mBoxHandlers[0].setNewRect(rec);
            cv::Mat part;
            mBoxHandlers[0].getImage(frame,part);
            Image::convert_to_grayscale(part,grayscale_image);
        }else{
            Image::convert_to_grayscale(frame,grayscale_image);
        }

        int end = 1;
        if(mCLAHE){
            end = 2;
        }
        for(int clahe = 0; !success && clahe < end ; clahe++){
            if(clahe > 0){
                // Image::CLAHE(grayIMG,grayscale_image,0.875);
                Image::Histogram(grayscale_image,grayscale_image);
            }
            clnf_models[Model_Init].Reset();

            // Detect faces in an image
            vector<cv::Rect_<double> > face_detections;

            if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                vector<double> confidences;
                LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[Model_Init].face_detector_HOG, confidences);
            }else{
                LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[Model_Init].face_detector_HAAR);
            }

            for(size_t face=0; face < face_detections.size(); ++face){
                // if there are multiple detections go through them
                success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_models[Model_Init], det_parameters[Model_Init]);
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
            if(rec.width > 0 && rec.height > 0){
                mBoxHandlers[0].toImage(clnf_models[Model_Init]);
            }
            int used;
            CalcualteEyes(frame.clone(),Model_Init,used,1.0);

            mAtentionTracer->showSolution(QString::fromStdString(name),frm,
                                          clnf_models[Model_Init],fx,fy,cx,cy, (double)Model_Init/num_faces_max, true);

            mPrinter.printSmallImage(disp_image.clone(),clnf_models[Model_Init],*painterR,*painterL, "img/Head_"+name,
                                     mTheWindow->Right_Label->size().width(), mTheWindow->Right_Label->size().height()/num_faces_max, Model_Init);


            double colore = (double)Model_Init/num_faces_max;
            mPrinter.print_CLNF(disp_image,clnf_models[Model_Init],0.5,fx,fy,cx,cy,colore);

            //std::cout<<"Gesicht: "<<clnf_models[0].GetBoundingBox()<<std::endl;
            active_models[Model_Init] = false;

            std::ofstream myfile;
            myfile.open ("./data/Test_Positionen_3_Label_Image.txt", std::ios::in | std::ios::app);
            myfile <<name<<std::endl;
            myfile.close();

        }
        if(mPrinter.isSaveImage()){
            if(disp_image.data){
                if(!success){
                    name += "_NotFound.png";
                }
                if(rec.width > 0 && rec.height > 0){
                    mPrinter.saveImage(name,disp_image(mBoxHandlers[0].getRect()));
                }else{
                    std::cout<<"Rechteck: "<<rec<<std::endl;
                }
            }else{
                std::cout<<"No Data "<<name<<std::endl;
            }
        }
        mAtentionTracer->print();

        painterL->end();
        painterR->end();

        imgCount++;
        showImage(disp_image, *pixmapL, *pixmapR);

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
    while(mEventHandler->getNextImageFrame(frm,box,name, i, mGaze)){
        name = "img/"+name;
        mKamera->getFrame(frame,frm);
        // perform landmark detection for every face detected
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        mPrinter.printSmallImage(frame,box,i,*painterL, name,
                                 mTheWindow->Right_Label->size().width(), mTheWindow->Right_Label->size().height()/num_faces_max);

        frameID = mEventHandler->getFramePos(frm);
        if(mEventHandler->isLandmark(frameID,i)){
            double land[5][2];
            mEventHandler->getLandmarks(frameID,i,land);
            for(int j = 0; j < 5; j++){
                cv::circle(frame, cv::Point2d(land[j][0],land[j][1]),std::min(5,std::max(1,(box.width+box.height)/70)),cv::Scalar(0,255,0),-1);
            }
        }
        mPrinter.printSmallImage(frame,box,i,*painterR, name,
                                 mTheWindow->Right_Label->size().width(), mTheWindow->Right_Label->size().height()/num_faces_max);

        if(mShowImageBox){
            cv::rectangle(frame,box,cv::Scalar(0,255,0),3);
        }

        showImage(frame, *pixmapL, *pixmapR);

        if(cv::waitKey(30) >= 0)
            return;
    }
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

void FaceDetection::CalcualteEyes(cv::Mat img, size_t CLNF_ID, int &used, double fx)
{
    /*
     * used
     * 0: OpenFace
     * 1: ELSE
     * 2: ELSE & OpenFace

     * Landmarks der Augen
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
    int hir_id[] = {-1,-1};
    int useCalc[] = {0,0};

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
            rec.x = max(0.0,rec.x - sx/2.0);
            rec.y = max(0.0,rec.y - sy/2.0);
            rec.width = min(rec.width + sx, img.cols-rec.x);
            rec.height = min(rec.height + sy, img.rows-rec.y);

            std::vector<cv::Point2f> pos, pos2;
            for(size_t i = 20; i < 28; i++){
                pos.push_back(cv::Point2d(shape2D.at<double>(i),shape2D.at<double>(i+n)));
                pos2.push_back(cv::Point2d(shape2D.at<double>(i-20),shape2D.at<double>(i-20+n)));
            }
            cv::RotatedRect min_eye = cv::fitEllipse(pos);
            cv::RotatedRect max_eye = cv::fitEllipse(pos2);
            if((min_eye.size.width+min_eye.size.height)/2 < 21/fx){
                cv::Mat gray;
                Image::toGrayGleam(img(rec), gray);
                float quality;
                cv::RotatedRect ellipse = ELSE::run(gray, quality);
                if(ellipse.size.width > 0 && ellipse.size.height >0
                        && sqrt(pow(max_eye.center.x - (rec.x + ellipse.center.x),2.0) + pow(max_eye.center.y - (rec.y + ellipse.center.y),2.0))
                        < (max_eye.size.width+max_eye.size.height)/5.0){

                    if((min_eye.size.width+min_eye.size.height)/2 > 15.0){
                        useCalc[hir] = 2;
                        for( int i = 0; i < n; ++i){
                            if(i < 8){
                                double x = (shape2D.at<double>(i) + ellipse.center.x+rec.x + cos((i-4)*M_PI/4.0)*ellipse.size.width/2*1.5)/2;
                                double y = (shape2D.at<double>(i+n) + ellipse.center.y+rec.y + sin((i-4)*M_PI/4.0)*ellipse.size.height/2*1.5)/2;
                                shape2D.at<double>(i)  = x;
                                shape2D.at<double>(i+n)= y;
                            }else if(i > 19){
                                double x = (shape2D.at<double>(i) + ellipse.center.x+rec.x + cos((23-i)*M_PI/4.0)*ellipse.size.width/2*0.7)/2;
                                double y = (shape2D.at<double>(i+n) + ellipse.center.y+rec.y + sin((23-i)*M_PI/4.0)*ellipse.size.height/2*0.7)/2;
                                shape2D.at<double>(i)  = x;
                                shape2D.at<double>(i+n)= y;
                            }
                        }
                    }else{
                        useCalc[hir] = 1;
                        for( int i = 0; i < n; ++i){
                            if( i < 8){
                                shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((i-4)*M_PI/4.0)*ellipse.size.width/2*1.5;
                                shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((i-4)*M_PI/4.0)*ellipse.size.height/2*1.5;
                            }else if(i > 19){
                                shape2D.at<double>(i)  = ellipse.center.x+rec.x + cos((23-i)*M_PI/4.0)*ellipse.size.width/2*0.7;
                                shape2D.at<double>(i+n)= ellipse.center.y+rec.y + sin((23-i)*M_PI/4.0)*ellipse.size.height/2*0.7;
                            }
                        }
                    }
                }
                clnf_models[CLNF_ID].hierarchical_models[hir_id[hir]].detected_landmarks = shape2D.clone();
            }
        }
    }
    used = useCalc[0]*10+useCalc[1];
}

void FaceDetection::initCLNF()
{
    clnf_models.clear();
    active_models.clear();
    det_parameters.clear();
    mBoxHandlers.clear();

    vector<string> arguments;
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
    mBoxHandlers.push_back(BoxHandler(0,0));

    for (int i = 1; i < num_faces_max; ++i)
    {
        clnf_models.push_back(clnf_model);
        active_models.push_back(false);
        det_parameters.push_back(det_params);
        mBoxHandlers.push_back(BoxHandler(0,0));
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
    for(size_t i = 0; i < mBoxHandlers.size(); i++){
        mBoxHandlers[i].setAutoSize(a);
    }
}

void FaceDetection::setBoxScall(double s){
    for(size_t i = 0; i < mBoxHandlers.size(); i++){
        mBoxHandlers[i].setBoxScall(s);
    }
}

void FaceDetection::setBoxMinSize(int w, int h)
{
    for(size_t i = 0; i < mBoxHandlers.size(); i++){
        mBoxHandlers[i].setBoxMinSize(w,h);
    }
}

void FaceDetection::setGaze(int gaze)
{
    mGaze = gaze;
}

int FaceDetection::getGaze()
{
    return mGaze;
}

void FaceDetection::setShowImageBox(bool b)
{
    mShowImageBox = b;
}

void FaceDetection::setShowHeadBox(bool h)
{
    mPrinter.setShowHeadBox(h);
}

void FaceDetection::setSaveVideoImage(bool v)
{
    mAtentionTracer->setSaveVideoImage(v);
}

size_t FaceDetection::loadXML(QString path, bool clear)
{
    return mEventHandler->loadXML(path,clear);
}

void FaceDetection::setUseBox(bool b)
{
    mUseBox = b;
}

void FaceDetection::setUseTime(bool t)
{
    mAtentionTracer->setUseTime(t);
}

void FaceDetection::setLearn(bool l)
{
    mUseImage = l;
}

void FaceDetection::setCLAHE(bool c)
{
    mCLAHE = c;
}

void FaceDetection::setUseEye(bool e)
{
    mAtentionTracer->setUseAVGEye(e);
    mPrinter.setUseAVGEye(e);
}

void FaceDetection::setShowEyes(bool show)
{
    mPrinter.setShowEye(show);
}

void FaceDetection::setSaveIamge(bool save)
{
    mPrinter.setSaveImage(save);
}

void FaceDetection::setWriteSolution(bool write)
{
    mAtentionTracer->setWriteToFile(write);
}

void FaceDetection::setShowAtention(bool show)
{
    mAtentionTracer->setShowAtention(show);
}

void FaceDetection::setShowLandmarks(bool land)
{
    mPrinter.setDrawLandmarks(land);
}

bool FaceDetection::getFrame(cv::Mat &img, size_t FrameID)
{
    size_t frameNr;
    if(!mUseBox && mKamera->getFrameNr() <= mEventHandler->mVideoObservationEnde){
        return mKamera->getFrame(img);
    }else if(mEventHandler->getFrame(frameNr,FrameID)){ //FrameID -> FarmeNummer
        if(mKamera->getFrameNr() +1 == frameNr){
            return mKamera->getFrame(img);
        }else{
            return mKamera->getFrame(img,frameNr);
        }
    }else{
        return false;
    }
}

bool FaceDetection::getFrame(cv::Mat &Img, size_t &Frame, cv::Rect &Rec, string &Name,
                             double &fx, double &fy, double &cx, double &cy, int &x, int &y)
{
    if(mUseImage){
        int FramePos = -1;
        if(mEventHandler->getImage(Img,Frame, Name,FramePos,Rec)){
            if(FramePos >= 0){
                mKamera->getFrame(Img,FramePos);
                mKamera->setImageSize(Img.cols,Img.rows);
                mKamera->get_camera_params(fx,fy,cx,cy,x,y);
            }else{
                fx = fy = ((500 * (Rec.width / 640.0)) + (500 * (Rec.height / 480.0)))/2.0;
                cx = Rec.width/2.0;
                cy = Rec.height/2.0;
            }
            Frame++;
            return Img.data;
        }else{
            Frame++;
            fx = fy = cx = cy = 0.0;
            x=y=0;
            Rec.x = Rec.y = Rec.width = Rec.height = 0;
            Name = "";
            return false;
        }
    }else{
        int BoxID = -1;
        if(mEventHandler->getNextImageFrame(Frame,Rec,Name, BoxID,mGaze)){
            mKamera->getFrame(Img,Frame);
            mKamera->setImageSize(Img.cols,Img.rows);
            mKamera->get_camera_params(fx,fy,cx,cy,x,y);
            return true;
        }else{
            return false;
        }
    }
}

void FaceDetection::showImage(const cv::Mat image, const QPixmap &pixmapL, const QPixmap &pixmapR){
    QImage img = Image::MatToQImage(image);
    QImage img2 = img.scaled(mTheWindow->Main_Label->size(),Qt::KeepAspectRatio);
    mTheWindow->Main_Label->setPixmap(QPixmap::fromImage(img2));

    mTheWindow->Right_Label->setPixmap(pixmapL);
    mTheWindow->Left_Label->setPixmap(pixmapR);
}

void FaceDetection::print_FPS_Model(int fps, int model){
    mTheWindow->FPS_Label->setText(QString::number(fps));
    mTheWindow->Model_Label->setText(QString::number(model));
}
