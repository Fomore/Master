#include "facedetection.h"

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

FaceDetection::FaceDetection(Ui::MainWindow *mWindow)
{
    mTheWindow = mWindow;

    mKamera = new Camera(2);

    vector<string> arguments;
    arguments.push_back(""); // Hat arguments keine Werte kann wes wegoptimiert werden und dadurch wirft die Initilaisierung unten Fehler

    /*
    -mloc - the location of landmark detection models
    -sigma -
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

    //    mKamera->correct_Image();

    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;

    // Anwendung - Berechnung der Faces
    if(path.size() < 1){
        path = "/home/falko/Uni/Master/Film/Selbst_Webcam_01.mp4";
    }
    cv::VideoCapture video(path);
    if(!video.isOpened()){
        cout<<"Kein Video"<<std::endl;
        return;
    }
    cv::Mat frame_col;

//    cv::namedWindow("tracking_result",1);

    for(int frame_count = 0;video.read(frame_col);frame_count++){
        if(frame_count == 0){
            mKamera->correct_Image_Init(frame_col.rows,frame_col.cols);
        }
        // Reading the images
        mKamera->correct_Image(frame_col);
        cv::Mat_<float> depth_image;
        cv::Mat_<uchar> grayscale_image;

        cv::Mat disp_image = frame_col.clone();

        if(frame_col.channels() == 3)
        {
            cv::cvtColor(frame_col, grayscale_image, CV_BGR2GRAY);
        }
        else
        {
            grayscale_image = frame_col.clone();
        }

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
            if(clnf_models[model].failures_in_a_row > 4)
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
                        detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, face_detections[detection_ind], clnf_models[model], det_parameters[model]);

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
                detection_success = LandmarkDetector::DetectLandmarksInVideo(grayscale_image, depth_image, clnf_models[model], det_parameters[model]);
            }
        });

        // Go through every model and visualise the results
        for(size_t model = 0; model < clnf_models.size(); ++model)
        {
            // Visualising the results
            // Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
            double detection_certainty = clnf_models[model].detection_certainty; // Qualität der detection: -1 perfekt und 1 falsch

            double visualisation_boundary = -0.1;

            // Only draw if the reliability is reasonable, the value is slightly ad-hoc
            if(detection_certainty < visualisation_boundary)
            {

                print_Eyes(disp_image, clnf_models[model]);

                LandmarkDetector::Draw(disp_image, clnf_models[model]);

                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;

                detection_certainty = (detection_certainty + 1)/(visualisation_boundary +1);

                // A rough heuristic for box around the face width
                int thickness = (int)std::ceil(2.0* ((double)frame_col.cols) / 640.0);

                // Work out the pose of the head from the tracked model
                cv::Vec6d pose_estimate = LandmarkDetector::GetCorrectedPoseWorld(clnf_models[model], fx, fy, cx, cy);

                // Draw it in reddish if uncertain, blueish if certain
                LandmarkDetector::DrawBox(disp_image, pose_estimate, cv::Scalar((1-detection_certainty)*255.0,0, detection_certainty*255), thickness, fx, fy, cx, cy);
            }
        }

        // Work out the framerate
        if(frame_count % 10 == 0)
        {
            t1 = cv::getTickCount();
            fps = 10.0 / (double(t1-t0)/cv::getTickFrequency());
            t0 = t1;
        }

        // Write out the framerate on the image before displaying it
        char fpsC[255];
        sprintf(fpsC, "%d", (int)fps);
        string fpsSt("FPS:");
        fpsSt += fpsC;
        cv::putText(disp_image, fpsSt, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1, CV_AA);

        int num_active_models = 0;

        for( size_t active_model = 0; active_model < active_models.size(); active_model++)
        {
            if(active_models[active_model])
            {
                num_active_models++;
            }
        }

        char active_m_C[255];
        sprintf(active_m_C, "%d", num_active_models);
        string active_models_st("Active models:");
        active_models_st += active_m_C;
        cv::putText(disp_image, active_models_st, cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 1, CV_AA);

        if(!det_parameters[0].quiet_mode)
        {
//            imshow("tracking_result", disp_image);
            showImage(disp_image);
        }else{
            cout<<"Lol Quiet-Mode aktiv!"<<endl;
        }
        if(cv::waitKey(30) >= 0) break;
    }
}

void FaceDetection::print_Eye(const cv::Mat img, const LandmarkDetector::CLNF &clnf_model, int pos, string name, bool right){
    cv::Mat_<double> shape2D = clnf_model.detected_landmarks;

    int n = shape2D.rows/2;

    double X = cvRound(shape2D.at<double>(pos));
    double Y = cvRound(shape2D.at<double>(pos + n));
    double Width = cvRound(shape2D.at<double>(pos));
    double Height = cvRound(shape2D.at<double>(pos + n));
    for(int i = pos+1; i < pos+6; ++i)// Beginnt bei 0 das Output-Format
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
    if(X >= 0 && Y >= 0 && Width > 0 && Height > 0 && X+Width < img.cols && Y+Height < img.rows){
        cv::Mat img_Eye = img(cv::Rect(X,Y,Width,Height));
//        cv::namedWindow(name,1);
//        imshow(name, img_Eye);
        showEyeImage(img_Eye,1,right);
    }
}

void FaceDetection::print_Eyes(const cv::Mat img, const LandmarkDetector::CLNF &clnf_model){
    print_Eye(img,clnf_model,36,"Left Eye",false);
    print_Eye(img,clnf_model,42,"Right Eye",true);
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

void FaceDetection::showImage(cv::Mat image){
    QImage img = MatToQImage(image);
    QImage img2 = img.scaled(mTheWindow->Main_Label->size().width(),mTheWindow->Main_Label->size().height(),Qt::KeepAspectRatio);
    mTheWindow->Main_Label->setPixmap(QPixmap::fromImage(img2));
}

void FaceDetection::showEyeImage(cv::Mat image, int number, bool right){
    QImage img = MatToQImage(image);
    int h,w;
    if(right){
        h = mTheWindow->Right_Label->size().height();
        w = mTheWindow->Right_Label->size().width();
    }else{
        h = mTheWindow->Left_Label->size().height();
        w = mTheWindow->Left_Label->size().width();
    }
    QImage img2 = img.scaled(w,h/num_faces_max,Qt::KeepAspectRatio);
    if(right){
        mTheWindow->Right_Label->setPixmap(QPixmap::fromImage(img2));
    }else{
        mTheWindow->Left_Label->setPixmap(QPixmap::fromImage(img2));
    }
}

// Diese Methode stammt von http://www.qtcentre.org/threads/56482-efficient-way-to-display-opencv-image-into-Qt
QImage FaceDetection::MatToQImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    else if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
//        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}
