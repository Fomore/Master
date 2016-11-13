#include "camera.h"

#include <iostream>

Camera::Camera(){
    Camera(2);
}
Camera::Camera(std::string path){
    camera_calibration(path);
}

Camera::Camera(int id)
{
    if(id == 1){ //Für 4k Action Cam
        cameraMatrix = (cv::Mat_<double>(3,3) << 5606.085310668697, 0, 1919.69788797893,
                        0, 5645.910633922549, 1077.745088495707,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -0.8104565014370436, -2.781890571732347,
                      0.005975670615393422, -0.02735745506102933, 14.40716902307102);
    }else if(id == 2){ //Webcam
        cameraMatrix = (cv::Mat_<double>(3,3) << 1687.931264381175, 0, 220.916339131956,
                        0, 1725.537614772272, 328.0023471819056,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -0.2337548449359694, 6.124272192973521, -0.02745182623396643, 0.04883917214658949, -78.60351534016021 );
    }else if(id == 3){ //1280P der 4k Cam
        cameraMatrix = (cv::Mat_<double>(3,3) << 0,0,0,0,0,0,0,0,0 );
        distCoeffs = (cv::Mat_<double>(1,5) << 0,0,0,0,0 );
    }
    else{//Default Parameter
        cameraMatrix = (cv::Mat_<double>(3,3) << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);

    }
}

Camera::~Camera()
{

}

void Camera::camera_calibration(std::string path){
    // Bei h und w aufpassen, das ist ein nervender Fehler bei falscher Wahl
    int h = 5;
    int w = 5;
    cv::Size patternsize(h,w);
    std::vector<cv::Point3f> realPoints;
    for(int i = 0; i < h*w; i++){
        realPoints.push_back(cv::Point3f((i/h)*0.031,(i%h)*0.031,0));
    }
    std::vector<std::vector<cv::Point3f> > p;
    std::vector<std::vector<cv::Point2f> > m;


    bool set = true;


    cv::Size s(0,0);

    std::cout<<"Lade Video "<<std::endl;
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161113_19_22_03_Pro.mp4");
    if(video.isOpened()){
        cv::Mat frame_col;
        //        cv::namedWindow("True Image Colo",1);
        //        cv::namedWindow("False Image Colo",1);

        while (video.read(frame_col)) {
            if(set){
                set = false;
                s.width = frame_col.cols;
                s.height= frame_col.rows;
            }

            cv::Mat gray;
            cvtColor(frame_col, gray, CV_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            corners.clear();
            bool patternfound = cv::findChessboardCorners(frame_col, patternsize, corners,
                                                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
            if(patternfound){
                cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.04));

                p.push_back(realPoints);
                m.push_back(corners);

                //                drawChessboardCorners(frame_col, cv::Size(h,w), corners, patternfound);
                //                imshow("True Image Colo", frame_col);
            }
            else{
                //                imshow("False Image Colo", frame_col);
            }

            if(cv::waitKey(30) >= 0) break;
        }
        //        cv::destroyWindow("True Image Colo");
        //        cv::destroyWindow("False Image Colo");
    }else{
        std::cout<<"Datei nicht gefunden"<<std::endl;
    }


    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< p.size()<< " Bildern" << std::endl;
    cv::calibrateCamera(p, m, s, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
}

void Camera::correct_Image(){
    //    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/20161112_125821A.mp4");//4K
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161113_15_30_03_Pro.mp4");//Webcam
//        cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161113_19_22_03_Pro.mp4");//1080P
    cv::Size imageSize;
    if(video.isOpened()){
        cv::Mat frame_col, map1, map2;
        cv::namedWindow("Image Raw",1);
        cv::namedWindow("Image Correct",1);

        bool init = true;

        while (video.read(frame_col)) {
            if(init){
                imageSize.height = frame_col.rows;
                imageSize.width = frame_col.cols;
                init = false;
                cv::initUndistortRectifyMap(
                            cameraMatrix, distCoeffs, cv::Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                            CV_16SC2, map1, map2);
            }
            cv::Mat view;

            //            cv::undistort(frame_col, view, cameraMatrix, distCoeffs);//Korrektur mit beschneiden
            cv::remap(frame_col, view, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung

            cv::Size size(384*2.5, 216*2.5);
            resize(view,view,size);
            resize(frame_col,frame_col,size);
            imshow("Image Raw", frame_col);
            imshow("Image Correct", view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}
