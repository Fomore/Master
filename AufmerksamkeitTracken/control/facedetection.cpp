#include "facedetection.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace std;

FaceDetection::FaceDetection(Ui::MainWindow *mWindow)
{
    mTheWindow = mWindow;

    mKamera = new Camera(-1);
    Model_Init = 0;
    imgCount = 0;

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

FaceDetection::~FaceDetection()
{

}

void FaceDetection::FaceTracking(std::string path){
    // Initialisiierung
    double fx,fy,cx,cy;
    mKamera->get_camera_params(fx,fy,cx,cy);

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    // Anwendung - Berechnung der Faces
    if(path.size() < 1){
        //        path = "/home/falko/Uni/Master/Film/Selbst_Webcam_01.mp4";
        //        path = "/home/falko/Uni/Master/Film/Schulklasse_01.mp4";
        //        path = "/home/falko/Uni/Master/Film/Chor_01.mp4";
        //        path = "/home/falko/Uni/Master/Film/Interview_640.mp4";
        path = "/home/falko/Uni/Master/Film/Interview_1280.mp4";
    }
    cv::VideoCapture video(path);
    if(!video.isOpened()){
        cout<<"Kein Video"<<std::endl;
        return;
    }
    cv::Mat frame_col;

    //    cv::namedWindow("tracking_result",1);
    /*
    cv::namedWindow("Normal",1);
    mImage.getImage(frame_col);
    imshow("Normal", frame_col);

    cv::namedWindow("grau",1);
    mImage.convert_to_grayscale(frame_col,frame_col);
    imshow("grau", frame_col);
    */
    for(int frame_count = 0;video.read(frame_col);frame_count++){
        //   for(int frame_count = 0;mImage.getScallImage(frame_col);frame_count++){
        //      for(int frame_count = 0;mImage.getImage(frame_col);frame_count++){//Hiermit bekommt am auf Chor-Face einen Wert nach einiger Zeit
        // Reading the images
        mKamera->correct_Image(frame_col);
        cv::Mat_<uchar> grayscale_image;

        cv::Mat disp_image = frame_col.clone();

        Image::convert_to_grayscale(frame_col,grayscale_image);

        vector<cv::Rect_<double> > face_detections;

        bool all_models_active = true;
        for(unsigned int model = 0; model < clnf_models.size(); ++model)
        {
            if(!active_models[model])
            {
                all_models_active = false;
            }

        }

        // Get the detections (every 8th frame and when there are free models available for tracking) //Nun wird jedes Frame (%1) Berechent
        if(frame_count % 1 == 0 && !all_models_active)
        {
            if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR)
            {
                vector<double> confidences;
                LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[0].face_detector_HOG, confidences);
            }
            else
            {
                LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[0].face_detector_HAAR);
            }
        }

        // Keep only non overlapping detections (also convert to a concurrent vector
        NonOverlapingDetections(clnf_models, face_detections);

        vector<tbb::atomic<bool> > face_detections_used(face_detections.size());

        // Go through every model and update the tracking
        tbb::parallel_for(0, (int)clnf_models.size(), [&](int model){
            //for(unsigned int model = 0; model < clnf_models.size(); ++model)
            //{

            bool detection_success = false;

            // If the current model has failed more than 4 times in a row, remove it
            if(clnf_models[model].failures_in_a_row > 4*8)
            {
                active_models[model] = false;
                clnf_models[model].Reset();
            }

            // If the model is inactive reactivate it with new detections
            if(!active_models[model])
            {

                for(size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind)
                {
                    // if it was not taken by another tracker take it (if it is false swap it to true and enter detection, this makes it parallel safe)
                    if(face_detections_used[detection_ind].compare_and_swap(true, false) == false)
                    {
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
            }
            else
            {
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

        int w = mTheWindow->Right_Label->size().width();
        int h = mTheWindow->Right_Label->size().height()/num_faces_max;

        for(size_t model = 0; model < clnf_models.size(); ++model)
        {
            // Visualising the results
            // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
            double detection_certainty = clnf_models[model].detection_certainty; // Qualität der detection: -1 perfekt und 1 falsch

            double visualisation_boundary = -0.1;

            // Only draw if the reliability is reasonable, the value is slightly ad-hoc
            if(detection_certainty < visualisation_boundary && !det_parameters[0].quiet_mode){

                cv::Mat L = print_Eye(frame_col,model,36,6); //Left
                cv::Mat R = print_Eye(frame_col,model,42,6); //Right
                if(!L.data || !R.data){
                    if(!R.data){
                        R = print_Eye(frame_col,model,0,27);
                    }else{
                        L = print_Eye(frame_col,model,0,27);
                    }
                }
                if(L.data){
                    QImage img = Image::MatToQImage(L);
                    QImage img2 = img.scaled(w,h,Qt::KeepAspectRatio);
                    QPixmap pix = QPixmap::fromImage(img2);
                    painterL->drawPixmap(0, h*model, pix);
                }
                if(R.data){
                    QImage img = Image::MatToQImage(R);
                    QImage img2 = img.scaled(w,h,Qt::KeepAspectRatio);
                    QPixmap pix = QPixmap::fromImage(img2);
                    painterR->drawPixmap(0, h*model, pix);
                }

                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;

                double itens = (detection_certainty + 1)/(visualisation_boundary +1);
                print_CLNF(disp_image,model,itens,fx,fy,cx,cy);
//                std::cout<<"Gesicht "<<model<<": "<<clnf_models[model].GetBoundingBox()<<std::endl;
            }
        }
        painterL->end();
        painterR->end();

        // Work out the framerate
        if(frame_count % 10 == 0)
        {
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

        if(!det_parameters[0].quiet_mode)
        {
            //            imshow("tracking_result", disp_image);
            print_FPS_Model(cvRound(fps),num_active_models);
            showImage(disp_image);
            mTheWindow->Right_Label->setPixmap(*pixmapL);
            mTheWindow->Left_Label->setPixmap(*pixmapR);
        }else{
            cout<<"Lol Quiet-Mode aktiv!"<<endl;
        }
        if(cv::waitKey(30) >= 0) break;
    }
}

void FaceDetection::print_CLNF(cv::Mat img, int model, double itens, double fx, double fy, double cx, double cy){
    LandmarkDetector::Draw(img, clnf_models[model]);

    // A rough heuristic for box around the face width
    int thickness = (int)std::ceil(2.0* ((double)img.cols) / 640.0);

    // Work out the pose of the head from the tracked model
    cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(clnf_models[model], fx, fy, cx, cy);

    // Draw it in reddish if uncertain, blueish if certain
    LandmarkDetector::DrawBox(img, pose_estimate, cv::Scalar((1-itens)*255.0,0, itens*255), thickness, fx, fy, cx, cy);
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

void FaceDetection::showImage(const cv::Mat image){
    QImage img = Image::MatToQImage(image);
    QImage img2 = img.scaled(mTheWindow->Main_Label->size().width(),mTheWindow->Main_Label->size().height(),Qt::KeepAspectRatio);
    mTheWindow->Main_Label->setPixmap(QPixmap::fromImage(img2));
}

cv::Mat FaceDetection::print_Eye(const cv::Mat img, int model, int pos, int step){
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

    int n = shape2D.rows/2;

    double X = cvRound(shape2D.at<double>(pos));
    double Y = cvRound(shape2D.at<double>(pos + n));
    double Width = cvRound(shape2D.at<double>(pos));
    double Height = cvRound(shape2D.at<double>(pos + n));
    for(int i = pos+1; i < pos+step; ++i)// Beginnt bei 0 das Output-Format
    {
        double x = (shape2D.at<double>(i));
        double y = (shape2D.at<double>(i + n));
        X = min(X,x);
        Y = min(Y,y);
        Width = max(Width,x);
        Height = max(Height,y);
    }
    // To Do: Grenzen Dynamische Abmessungen für Randwerte
    Width = Width-X;
    Height = Height-Y;

    double fr_X = Width*0.35;
    double fr_Y = Height*0.4;
    X -= fr_X;
    Y -= fr_Y;
    Width += fr_X*2;
    Height += fr_Y*2;

    X = max(X,0.0);
    Y = max(Y,0.0);
    Width = min(Width,img.cols-X);
    Height = min(Height, img.rows-Y);

    if(Width > 16 && Height > 10){
        cv::Mat img_Eye = img(cv::Rect(X,Y,Width,Height));
        if(step == 6){
            /*
        std::string name = "Eye_"+std::to_string(imgCount)+"_";
        if(right){
            name+="R";
        }else{
            name+="L";
        }
        Image::saveImage(img_Eye,name);
*/
            cv::Mat gray;
            Image::convert_to_grayscale(img_Eye, gray);

            cv::RotatedRect ellipse = ELSE::run(gray);
            cv::ellipse(img_Eye, ellipse, cv::Scalar(0,255,0,255), 1,1 );
        }
        return img_Eye;
    }
    return cv::Mat();
}

void FaceDetection::print_FPS_Model(int fps, int model){
    mTheWindow->FPS_Label->setText(QString::number(fps));
    mTheWindow->Model_Label->setText(QString::number(model));
}

void FaceDetection::LearnModel(){
    // Initialisiierung
    double fx,fy,cx,cy;
    mKamera->get_camera_params(fx,fy,cx,cy);
    cv::Mat frame_col,frame;

    if(mImage.getImage(frame)){
        // Reading the images
        mKamera->correct_Image(frame);
        int X,Y,W,H;
        if(Model_Init == 0){
            X=230; Y=65, W=48; H=63;
        }else if(Model_Init == 1){
            X=381; Y=95, W=36; H=48;
        }else if(Model_Init == 2){
            X=440; Y=90, W=35; H=45;
        }else if(Model_Init == 4){
            X=304; Y=117, W=21; H=37;
        }else if(Model_Init == 3){
            X=161; Y=72, W=35; H=50;
        }else if(Model_Init == 5){
            X=516; Y=92, W=41; H=56;
        }

        if(X != 0){
            frame_col = mImage.get_Face_Image(frame,X,Y,W,H);
        }else{
            frame_col = frame.clone();
        }

        cv::Mat_<uchar> grayscale_image;

        cv::Mat disp_image = frame_col.clone();

        Image::convert_to_grayscale(frame_col,grayscale_image);

        // Detect faces in an image
        vector<cv::Rect_<double> > face_detections;

        if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
            vector<double> confidences;
            LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[0].face_detector_HOG, confidences);
        }else{
            LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[0].face_detector_HAAR);
        }

        // perform landmark detection for every face detected
        QPixmap *pixmapL=new QPixmap(mTheWindow->Left_Label->size());
        pixmapL->fill(Qt::transparent);
        QPainter *painterL=new QPainter(pixmapL);

        QPixmap *pixmapR=new QPixmap(mTheWindow->Right_Label->size());
        pixmapR->fill(Qt::transparent);
        QPainter *painterR=new QPainter(pixmapR);

        int w = mTheWindow->Right_Label->size().width();
        int h = mTheWindow->Right_Label->size().height()/num_faces_max;

        for(size_t face=0; face < face_detections.size(); ++face)
        {
            Model_Init = (Model_Init+face)%num_faces_max;
            // if there are multiple detections go through them
            bool success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_models[0], det_parameters[0]);
            //            bool success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, face_detections[face], clnf_models[Model_Init], det_parameters[Model_Init]);

            if(success){
                cv::Mat L = print_Eye(frame_col,0,36,6); //Left
                cv::Mat R = print_Eye(frame_col,0,42,6); //Right
                if(!L.data || !R.data){
                    if(!R.data){
                        R = print_Eye(frame_col,0,0,27);
                    }else{
                        L = print_Eye(frame_col,0,0,27);
                    }
                }
                if(L.data){
                    QImage img = Image::MatToQImage(L);
                    QImage img2 = img.scaled(w,h,Qt::KeepAspectRatio);
                    QPixmap pix = QPixmap::fromImage(img2);
                    painterL->drawPixmap(0, h*face, pix);
                }
                if(R.data){
                    QImage img = Image::MatToQImage(R);
                    QImage img2 = img.scaled(w,h,Qt::KeepAspectRatio);
                    QPixmap pix = QPixmap::fromImage(img2);
                    painterR->drawPixmap(0, h*face, pix);
                }

                print_CLNF(disp_image,0,0.5,fx,fy,cx,cy);
                //                std::cout<<"Gesicht: "<<clnf_models[0].GetBoundingBox()<<std::endl;
                active_models[0] = true;
                if(X > 0)
                    shift_detected_landmarks(0,X,Y,W,H,frame_col.cols, frame_col.rows);
            }
        }
        painterL->end();
        painterR->end();

        Model_Init++;
        showImage(disp_image);
        mTheWindow->Right_Label->setPixmap(*pixmapL);
        mTheWindow->Left_Label->setPixmap(*pixmapR);
    }
}

void FaceDetection::shift_detected_landmarks(int model, double X, double Y,double w, double h, double W, double H){
    //    std::cout<<"Vorher\n"<<clnf_models[model].detected_landmarks<<std::endl;
    cv::Mat_<double> shape2D = clnf_models[model].detected_landmarks;

    double centerX = X+w/2;
    double centerY = Y+h/2;

    double fx = W/w;
    double fy = H/h;

    double centerW = W/2.0;
    double centerH = H/2.0;

    //    std::cout<<"Werte: "<<centerX<<"/"<<centerY<<" "<<centerW<<"/"<<centerH<<" "<<fx<<"/"<<fy<<std::endl;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        double pX = shape2D.at<double>(pos);
        double pY = shape2D.at<double>(pos + n);

        shape2D.at<double>(pos) = ((pX-centerW)/fx)+centerX;
        shape2D.at<double>(pos + n) = ((pY-centerH)/fy)+centerY;
        std::cout<<"["<<pX<<", "<<pY<<"] -> ["<<shape2D.at<double>(pos)<<", "<<shape2D.at<double>(pos+n)<<"]"<<std::endl;
    }
    clnf_models[model].detected_landmarks = shape2D.clone();
    //    std::cout<<"Danach\n"<<clnf_models[model].detected_landmarks<<std::endl;
}
