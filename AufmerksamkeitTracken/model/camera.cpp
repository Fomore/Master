#include "camera.h"

#include <iostream>
#include <math.h>

Camera::Camera(){
    init = true;
    camera_calibration("");
    correct_Image();
}

Camera::Camera(int id)
{
    init = true;
    if(id == 1){ //Webcam
        cameraMatrix = (cv::Mat_<double>(3,3) << 2288.8872146151, 0, 326.4366918263556,
                        0, 2186.84347316115, 240.4137245062925,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0.5488223009312883, 0.8600516903057833, -0.07985604709274093, 0.02173550476776541, -1048.693669653505);
    }else if(id == 2){ //1280P der 4k Actioncam als Webcam
        cameraMatrix = (cv::Mat_<double>(3,3) << 1907.35363928477, 0, 633.8982380360976,
                        0, 1645.567312479385, 366.9137086428425,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -0.4328529813449852, -0.5421577205524081,
                      -0.01820603681764989, -0.02865158102593597, 4.266311993981554 );
    }else{//Default Parameter
        cameraMatrix = (cv::Mat_<double>(3,3) << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    }
    //        correct_Image();
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

    std::vector<cv::VideoCapture> videos;

    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_0.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_2.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_3.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_4.mp4"));

    //    cv::namedWindow("True Image Colo",1);
    //    cv::namedWindow("Fase Image Colo",1);

    for(int i = 0; i < videos.size(); i++){ // Könnte parallel werden
        std::cout<<"Lade Video "<<i<<std::endl;
        cv::VideoCapture video = videos.at(i);
        if(video.isOpened()){
            cv::Mat frame_col;
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
                                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);
                if(patternfound){
                    cornerSubPix(gray, corners, cv::Size(3,3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.04));
                    m.push_back(corners);
                    //                    drawChessboardCorners(frame_col, cv::Size(h,w), corners, patternfound);
                    //                    imshow("True Image Colo", frame_col);
                }
                //                if(cv::waitKey(30) >= 0) break;
            }
            //                    cv::destroyWindow("True Image Colo");
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }

    std::cout << "Berechnung hat "<< m.size()<< " Bilder ergeben" << std::endl;

    std::vector<std::vector<cv::Point2f> > m2 = get_perfect_Points(m,s,120);
    for(int i = 0; i < m2.size(); i++){
        p.push_back(realPoints);
    }

    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< m2.size()<< " Bildern" << std::endl;
    cv::calibrateCamera(p, m2, s, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
    correct_Image_Init(s.height,s.width);
}

std::vector<std::vector<cv::Point2f> > Camera::get_perfect_Points(std::vector<std::vector<cv::Point2f> > points, const cv::Size dim, int maxImages){
    std::vector<cv::Point3d> your_point;
    for(int i = 0; i < points.size(); i++){
        cv::Mat point = cv::Mat(points.at(i)).reshape(1).t();
        double mean_X = cv::mean(point.row(0))[0];
        double mean_Y = cv::mean(point.row(1))[0];
        double radius = 0;
        for(int y = 0; y < point.rows; y++){
            radius += sqrt((point.at<float>(0,y)-mean_X)*(point.at<float>(0,y)-mean_X)
                           +(point.at<float>(1,y)-mean_Y)*(point.at<float>(1,y)-mean_Y));
        }
        your_point.push_back(cv::Point3d(mean_X,mean_Y,radius/25.0));
    }
    cv::Mat your_mat = cv::Mat(your_point).reshape(1).t();

    double q = (double)dim.width/(double)dim.height;
    double b = sqrt((double)maxImages/q);
    double a = sqrt((double)maxImages*q);

    double X = dim.width/a;
    double Y = dim.height/b;

    std::vector<int> center[(int)a+1][(int)b+1];
    for(int i = 0; i < your_mat.cols; i++){
        int x = your_mat.at<double>(0,i)/X;
        int y = your_mat.at<double>(1,i)/Y;
        center[x][y].push_back(i);
    }

    std::vector<int> valueUse;
    for(int x = 0; x<=(int)a; x++){
        for(int y= 0; y<=(int)b; y++){
            if(center[x][y].size() > 0){
                double centerX, centerY;
                if(x == 0){ centerX = 0;}
                else if(x == (int)a-1){ centerX = dim.width;}
                else{centerX = (x+0.5)*X;}

                if(y == 0){ centerY = 0;}
                else if(y == (int)b-1){ centerY = dim.height;}
                else{centerY = (y+0.5)*Y;}

                int value = center[x][y].at(0);
                double minDistanz = sqrt(pow(your_mat.at<double>(0,value)-centerX,2)+pow(your_mat.at<double>(1,value)-centerY,2));
                for(int i = 1; i < center[x][y].size(); i++){
                    int v = center[x][y].at(i);
                    double dis = sqrt(pow(your_mat.at<double>(0,v)-centerX,2)+pow(your_mat.at<double>(1,v)-centerY,2));
                    if(dis < minDistanz){
                        value = v;
                        minDistanz = dis;
                    }
                }
                double maxRadius = your_mat.at<double>(2,value);
                minDistanz += sqrt(pow(X/20,2)+pow(Y/20,2));
                for(int i = 0; i < center[x][y].size(); i++){
                    int v = center[x][y].at(i);
                    double dis = sqrt(pow(your_mat.at<double>(0,v)-centerX,2)+pow(your_mat.at<double>(1,v)-centerY,2));
                    if(dis < minDistanz && your_mat.at<double>(2,v) > maxRadius){
                        value = v;
                        maxRadius = your_mat.at<double>(2,v);
                    }
                }
                valueUse.push_back(value);
            }
        }
    }
    std::vector<std::vector<cv::Point2f> > ret;
    for(int i = 0; i < valueUse.size(); i++){
        int v = valueUse.at(i);
        ret.push_back(points.at(v));
    }
    return ret;
}

void Camera::correct_Image(){
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_1280_1.mp4");
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

            //            cv::Size size(384*2.5, 216*2.5);
            //            resize(view,view,size);
            //            resize(frame_col,frame_col,size);
            imshow("Image Raw", frame_col);
            imshow("Image Correct", view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}

void Camera::correct_Image(cv::Mat frame){
    if(init){
        cv::Size imageSize;
        imageSize.height = frame.rows;
        imageSize.width = frame.cols;
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                                    CV_16SC2, map1, map2);
        init = false;
    }
    //            cv::undistort(frame,frame, cameraMatrix, distCoeffs);//Korrektur mit beschneiden
    cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung
}

void Camera::correct_Image_Init(int height, int width){
    cv::Size imageSize;
    imageSize.height = height;
    imageSize.width = width;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                                CV_16SC2, map1, map2);
    init = false;
}

// Gibt die Werte der Kamera aus
void Camera::get_camera_params(double &fx, double &fy, double &cx, double &cy){
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);
}
