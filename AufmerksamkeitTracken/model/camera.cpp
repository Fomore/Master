#include "camera.h"

#include <iostream>
#include <math.h>

#include "model/image.h"

Camera::Camera(){
    init = true;
    ID = 0;
    camera_calibration("");
    correct_Image();
}

Camera::Camera(int id)
{
    init = true;
    setCameraParameter(6);
//    getCorrectImageSize(5,3);
    correct_Image();
}

Camera::~Camera()
{

}

void Camera::setCameraParameter(int id){
    ID = id;
    if(id == 1){ //Webcam
        cameraMatrix = (cv::Mat_<double>(3,3) << 2288.8872146151, 0, 326.4366918263556,
                        0, 2186.84347316115, 240.4137245062925,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0.5488223009312883, 0.8600516903057833, -0.07985604709274093, 0.02173550476776541, -1048.693669653505);
    }else if(id == 2){ //1280P der 4k Actioncam (als Webcam)
        cameraMatrix = (cv::Mat_<double>(3,3) << 4505.771917224674, 0, 627.2704519691812,
                        0, 2986.23823820304, 365.9469872012109,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << -5.683648805753482, 69.69903660169872, -0.1033539021069702, -0.0165845448486779, -487.6393497545911);
    }else if(id == 3){
        // 1940P der 4K Actioncam (1080P einstellung)
        cameraMatrix = (cv::Mat_<double>(3,3) << 13343.09623288915, 0, 966.1848227467876,
                        0, 7594.338338846001, 535.2483346076116,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -9.599320311558495, 153.5177497796487, -0.02105648553599707, -0.03765560152948207, 4.241928717530834);
    }else if(id == 4){
        //2688P der 4k Actioncam (2.7K Einstellung)
        cameraMatrix = (cv::Mat_<double>(3,3) << 15373.97717428267, 0, 1321.996093444815,
                        0, 17764.65120151666, 735.3988503456478,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -18.60440739952032, 523.0811532248169, 0.02032995403105285, -0.04626945929212306, 8.68919963553518);
    }else if(id == 5){
        // 3840P der 4K Actioncam
        cameraMatrix = (cv::Mat_<double>(3,3) << 7409.28638524711, 0, 1868.435847081091,
                        0, 7512.705802013185, 977.0636190423108,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -2.006696653082546, 14.50478814130672, 0.01196899857324854, -0.0326620616728269, -56.30904541044546);
    }else{//Default Parameter
        cameraMatrix = (cv::Mat_<double>(3,3) << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    }
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

    double avg_Quality = 0.0;
    int del=0;

    std::vector<cv::VideoCapture> videos;

    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1920_0.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1920_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_1920_2.mp4"));

    //    cv::namedWindow("True Image Colo",1);
    //    cv::namedWindow("False Image Colo",1);
    for(int i = 0; i < videos.size(); i++){ // Könnte parallel werden
        std::cout<<"Lade Video "<<i<<" - "<<m.size()<<" / "<<del<<std::endl;
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
                equalizeHist(gray, gray);

                std::vector<cv::Point2f> corners;
                corners.clear();
                bool patternfound = cv::findChessboardCorners(gray, patternsize, corners,
                                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);
                if(patternfound){
                    bool run = true;
                    int search_size = std::max(3.0,sqrt(pow(corners.at(0).x - corners.at(h*w-1).x,2)
                                                        +pow(corners.at(0).y - corners.at(h*w-1).y,2))/(sqrt(h*h+w*w)*5));
                    for(int j = 0; j < corners.size(); j++){
                        int X = corners.at(j).x-search_size;
                        int Y = corners.at(j).y-search_size;

                        cv::Mat img_cur = gray(cv::Rect(X,Y,search_size*2,search_size*2));
                        cv::Mat img_out;

                        cornerHarris(img_cur, img_out, 3, 3, 0.04, cv::BORDER_DEFAULT);
                        double minVal, maxVal;
                        cv::Point minLoc, maxLoc;

                        cv::minMaxLoc(img_out, &minVal, &maxVal, &minLoc, &maxLoc );

                        avg_Quality += maxVal;

                        if(maxVal < 0.0006){// Schwellenwert wie unscharf eine Ecke sein darf, bei 1280-> 0.0006
                            run = false;
                            del++;
                        }
                    }
                    /*
                    drawChessboardCorners(frame_col, cv::Size(h,w), corners, patternfound);
                    if(gray.cols > 1000 || gray.rows > 600){
                        double fx = 1000.0/gray.cols;
                        double fy = 600.0/gray.rows;
                        double fxy = std::min(fx,fy);
                        resize(gray, gray, cv::Size(), fxy, fxy, CV_INTER_LINEAR);
                    }
*/
                    if(run){
                        cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.04));
                        m.push_back(corners);
                        //                        imshow("True Image Colo", gray);
                        //                    }else{
                        //                        imshow("False Image Colo", gray);
                    }
                }
                //                if(cv::waitKey(30) >= 0) break;
            }
            //                    cv::destroyWindow("True Image Colo");
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }

    std::cout << "Berechnung hat "<< m.size()<< " Bilder ergeben mit Qualität "<<avg_Quality/(m.size()*25)<<" entfernt:"<<del<< std::endl;
    std::vector<std::vector<cv::Point2f> > m2;
    if(m.size() > 150){
        m2 = get_perfect_Points(m,s,200);
    }else{
        m2 = m;
    }
    for(int i = 0; i < m2.size(); i++){
        p.push_back(realPoints);
    }

    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< m2.size()<< " Bildern" << std::endl;
    if(m2.size() > 1){
        cv::Mat imgCal = cv::Mat::zeros(s.height,s.width, CV_8UC3 );
        int lineType = 8;
        int npt[] = { 4 };
        int colStep = 255/m2.size();

        for(int i = 0; i < m2.size(); i++){
            cv::Point rook_points[1][4];
            rook_points[0][0] = m2.at(i).at(20);
            rook_points[0][1] = m2.at(i).at(0);
            rook_points[0][2] = m2.at(i).at(4);
            rook_points[0][3] = m2.at(i).at(24);

            const cv::Point* ppt[1] = { rook_points[0] };

            cv::fillPoly( imgCal, ppt, npt, 1, cv::Scalar( 255-(i*colStep), 0, 255-((m2.size()-1-i)*colStep)), lineType );
        }
        cv::namedWindow("Kalibrierung",1);
        //        cv::imwrite("Verteilung",imgCal);
        DisplayImage(imgCal);
        imshow("Kalibrierung", imgCal);
        cv::calibrateCamera(p, m2, s, cameraMatrix, distCoeffs, rvecs, tvecs);
        std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;
        correct_Image_Init(s.height,s.width);
    }else{
        std::cout<<"Keine Kalibrierungsbilder gefunden"<<std::endl;
    }
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

void Camera::DisplayImage(cv::Mat &img){
    if(img.cols > 1000 || img.rows > 600){
        double fx = 1000.0/img.cols;
        double fy = 600.0/img.rows;
        double fxy = std::min(fx,fy);
        resize(img, img, cv::Size(), fxy, fxy, CV_INTER_LINEAR);
    }
}

void Camera::correct_Image(){
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_1920_0.mp4");
    if(video.isOpened()){
        cv::namedWindow("Image Raw",1);
        cv::namedWindow("Image Correct",1);

        init = true;
        cv::Mat frame_col;
        while (video.read(frame_col)) {
            cv::Mat view = frame_col.clone();
            correct_Image(view);

            DisplayImage(frame_col);
            DisplayImage(view);

            imshow("Image Raw", frame_col);
            imshow("Image Correct", view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}

void Camera::correct_Image(cv::Mat frame){
    bool tmp = init;
    if(tmp)
        Image::saveImage(frame,"Orginal");
    if(ID >= 0 ){
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
    if(tmp)
        Image::saveImage(frame,"Korrekt");
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

cv::Size Camera::getCorrectImageSize(int Width, int Height){
    std::vector<cv::Point> points, corpoints;

    for(int i = 0; i < std::max(Width,Height); i++){
        if(i < Width){
            points.push_back(cv::Point(i,0));
            points.push_back(cv::Point(i,Height-1));
        }
        if( i < Height){
            points.push_back(cv::Point(0,i));
            points.push_back(cv::Point(Width-1,i));
        }
    }
    cv::Mat a = cv::Mat(points);
    cv::Mat b = cv::Mat(corpoints);
    cv::undistortPoints(a, b, cameraMatrix, distCoeffs);
    std::cout<<a.t()<<" | <"<<b.t()<<std::endl;
    return cv::Size(Width,Height);
}
