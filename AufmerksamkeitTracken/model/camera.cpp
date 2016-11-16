#include "camera.h"

#include <iostream>
#include <math.h>

Camera::Camera(){
    camera_calibration("");
    init = true;
}

Camera::Camera(int id)
{
    init = true;
    if(id == 1){ //Webcam
        cameraMatrix = (cv::Mat_<double>(3,3) << 1687.931264381175, 0, 220.916339131956,
                        0, 1725.537614772272, 328.0023471819056,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -0.2337548449359694, 6.124272192973521,
                      -0.02745182623396643, 0.04883917214658949, -78.60351534016021 );
    }else if(id == 2){ //1280P der 4k Actioncam
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
    //    correct_Image();
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

    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161110_13_01_06_Pro.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161112_21_54_26_Pro.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/WIN_20161113_15_30_03_Pro.mp4"));

    for(int i = 0; i < videos.size(); i++){ // Könnte parallel werden
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
                //            if(cv::waitKey(30) >= 0) break;
            }
            //        cv::destroyWindow("True Image Colo");
            //        cv::destroyWindow("False Image Colo");
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }

    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< p.size()<< " Bildern" << std::endl;
    get_perfect_Points(m,s);
    cv::calibrateCamera(p, m, s, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
}

void Camera::get_perfect_Points(std::vector<std::vector<cv::Point2f> > points, const cv::Size dim){
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

    double minX, maxX, meanX;
    meanX = cv::mean(your_mat.row(0))[0];
    cv::minMaxLoc(your_mat.row(0), &minX, &maxX);

    std::cout<<"Ergebnis X: "<<minX<<" "<<meanX<<" "<<maxX<<std::endl;

    double minY, maxY, meanY;
    meanY = cv::mean(your_mat.row(1))[0];
    cv::minMaxLoc(your_mat.row(1), &minY, &maxY);

    std::cout<<"Ergebnis Y: "<<minY<<" "<<meanY<<" "<<maxY<<std::endl;

    double minR, maxR, meanR;
    meanR = cv::mean(your_mat.row(2))[0];
    cv::minMaxLoc(your_mat.row(2), &minR, &maxR);

    std::cout<<"Ergebnis R: "<<minR<<" "<<meanR<<" "<<maxR<<std::endl;

    double q = (double)dim.width/(double)dim.height;
    double b = sqrt(120.0/q);
    double a = sqrt(120.0*q);

    double X = dim.width/a;
    double Y = dim.height/b;

    std::cout<<dim<<" "<<q<<" "<<a<<" "<<(int)a<<" "<<b<<" - "<<X<<" "<<Y<<std::endl;

    std::vector<int> center[(int)a+1][(int)b+1];
    for(int i = 0; i < your_mat.cols; i++){
        int x = your_mat.at<double>(0,i)/X;
        int y = your_mat.at<double>(1,i)/Y;
        center[x][y].push_back(i);
    }
    std::vector<cv::Point2d> feld_dim;
    feld_dim.clear();
    for(int x = 0; x<=(int)a; x++){
        for(int y= 0; y<=(int)b; y++){
            if(center[x][y].size() > 0){
                double mi, mx;
                mi = mx = your_mat.at<double>(2,center[x][y].at(0));
                for(int i = 1; i < center[x][y].size(); i++){
                    double r = your_mat.at<double>(2,center[x][y].at(i));
                    mi = std::min(mi, r);
                    mx = std::max(mx, r);
                }
                std::cout<<x<<"/"<<y<<" : "<<mi<<" "<<mx<<" "<<center[x][y].size()<<std::endl;
                feld_dim.push_back(cv::Point2d(mi, mx));
            }
        }
    }
//    std::cout<<"Grenzen: "<<feld_dim<<std::endl;

    // Sortieren im Index;
    cv::Mat out;
    cv::sortIdx(your_mat, out, CV_SORT_EVERY_ROW+CV_SORT_ASCENDING);
}

void Camera::correct_Image(){
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_1280.mp4");
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
