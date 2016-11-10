#include "camera.h"

#include <iostream>

Camera::Camera()
{
    camera_calibration();
}

Camera::~Camera()
{

}

void Camera::camera_calibration(){
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
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_01_06_Pro.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_22_12_35_Pro.mp4"));

    //    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_04_14_Pro.mp4"));
    //    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_05_14_Pro.mp4"));
    //    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_06_38_Pro.mp4"));
    //    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_08_22_Pro.mp4"));

    bool set = true;


    cv::Size s(0,0);

    for(int i = 0; i < videos.size(); i++){
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
                    cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.04));

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
    }

    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;

    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< p.size()<< " Bildern" << std::endl;
    cv::calibrateCamera(p, m, s, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
}
