#include "camera.h"

#include <iostream>

Camera::Camera()
{
//    camera_calibration();
    //Für 4k Action Cam
    cameraMatrix = (cv::Mat_<double>(3,3) << 5606.085310668697, 0, 1919.69788797893,
                    0, 5645.910633922549, 1077.745088495707,
                    0, 0, 1);
    distCoeffs = (cv::Mat_<double>(1,5) << -0.8104565014370436, -2.781890571732347, 0.005975670615393422, -0.02735745506102933, 14.40716902307102);

    correct_Image();
}

Camera::~Camera()
{

}

void Camera::camera_calibration(){
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

    std::vector<cv::VideoCapture> videos;
    // 4k Action Cam
    //    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/20161112_125821A.mp4"));

    // Webcam
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161112_21_54_26_Pro.mp4"));
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_01_06_Pro.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_22_12_35_Pro.mp4"));

    // 1080p der 4k Action Cam
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161112_22_00_54_Pro.mp4"));
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_04_14_Pro.mp4"));
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_05_14_Pro.mp4"));
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_06_38_Pro.mp4"));
    //        videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_08_22_Pro.mp4"));

    bool set = true;


    cv::Size s(0,0);

    for(int i = 0; i < videos.size(); i++){
        std::cout<<"Lade Video "<<i<<std::endl;
        cv::VideoCapture video = videos.at(i);
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
            std::cout<<"Datei "<<i<<" nicht gefunden"<<std::endl;
        }
    }

    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< p.size()<< " Bildern" << std::endl;
    cv::calibrateCamera(p, m, s, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
}

void Camera::correct_Image(){
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/20161112_125821A.mp4");
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
            imshow("Image Raw", view);
            imshow("Image Correct", frame_col);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}
