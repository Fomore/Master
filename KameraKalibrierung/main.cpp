#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <tbb/parallel_for.h>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace std;

std::vector<std::vector<cv::Point2f> > get_perfect_Points(std::vector<std::vector<cv::Point2f> > points, const cv::Size dim, int maxImages){
    std::cout<<"Perfect: "<<points.size()<<std::endl;
    std::vector<cv::Point3d> your_point;
    for(size_t i = 0; i < points.size(); i++){
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
                for(size_t i = 1; i < center[x][y].size(); i++){
                    int v = center[x][y].at(i);
                    double dis = sqrt(pow(your_mat.at<double>(0,v)-centerX,2)+pow(your_mat.at<double>(1,v)-centerY,2));
                    if(dis < minDistanz){
                        value = v;
                        minDistanz = dis;
                    }
                }
                double maxRadius = your_mat.at<double>(2,value);
                minDistanz += sqrt(pow(X/20,2)+pow(Y/20,2));
                for(size_t i = 0; i < center[x][y].size(); i++){
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
    for(size_t i = 0; i < valueUse.size(); i++){
        int v = valueUse.at(i);
        ret.push_back(points.at(v));
    }
    return ret;
}

void DisplayImage(string titel, cv::Mat &img){
    if(img.cols > 1000 || img.rows > 600){
        double fx = 1000.0/img.cols;
        double fy = 600.0/img.rows;
        double fxy = std::min(fx,fy);
        resize(img, img, cv::Size(), fxy, fxy, CV_INTER_LINEAR);
    }else if(img.cols < 50 || img.rows < 30){
        double fx = 50.0/img.cols;
        double fy = 30.0/img.rows;
        double fxy = std::max(fx,fy);
        resize(img, img, cv::Size(), fxy, fxy, CV_INTER_LINEAR);
    }
    cv::imshow(titel, img);
}

void correct_Image(cv::Mat cameraMatrix, cv::Mat distCoeffs){
    //    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_1.mp4");
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_0.mp4");
    if(video.isOpened()){
        cv::namedWindow("Image Raw",1);
        cv::namedWindow("Image Correct",1);

        cv::Mat frame_col;
        /*
        cv::Mat map1, map2;
        cv::Size imageSize(frame.cols,frame.rows);
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                                    CV_16SC2, map1, map2);
        */
        while (video.read(frame_col)) {
            cv::Mat view = frame_col.clone();

            cv::undistort(frame_col, view, cameraMatrix, distCoeffs);//Korrektur mit beschneiden
            //cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung

            DisplayImage("Image Raw", frame_col);
            DisplayImage("Image Correct",view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}

void loadFromFile(std::vector<cv::Point3f> &WorldPoints, std::vector<cv::Point2f> &ImagePoints, cv::Mat &img){
    std::cout<<"Load Data"<<std::endl;
    std::ifstream file("/home/falko/Uni/Master/Dateien/Positions_Data5.txt");
    std::cout<<"Data: "<<file.is_open()<<std::endl;
    std::string str;
    size_t count = 0;
    while (std::getline(file, str)){
        std::vector<std::string> v;
        boost::split(v, str, boost::is_any_of(" ") );

        int x,y;
        if(count >= 125){
            x = atoi(v[2].c_str());
            y = atoi(v[3].c_str());
            cv::circle(img,cv::Point2i(x,y),4,
                       cv::Scalar(count%125*2,count%125*2,255-count%125*2,255));
        }else{
            x = atoi(v[2].c_str());
            y = atoi(v[3].c_str());
            cv::circle(img,cv::Point2i(x,y),4,
                       cv::Scalar(count%125*2,count%125*2,255-count%125*2,255));
        }

        //cv::putText(img,std::to_string(count),cv::Point2i(x,y),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255));

        ImagePoints.push_back(cv::Point2f(x,y));
        //WorldPoints.push_back(cv::Point3f((atoi(v[1].c_str())-4)*100,atoi(v[0].c_str())*100,0));
        WorldPoints.push_back(cv::Point3f((4-atoi(v[1].c_str()))*100,atoi(v[0].c_str())*100,0));

        count++;
    }
    file.close();
    for(size_t i = 0; i < WorldPoints.size(); i++){
        std::cout<<WorldPoints[i]<<ImagePoints[i]<<std::endl;
    }
    std::cout<<"Load ende"<<std::endl;
}

void SVD(std::vector<cv::Point3f> WorldPoints, std::vector<cv::Point2f> ImagePoints, double fx, double fy, double cx, double cy){
    double in[ImagePoints.size()][8];
    for(size_t i = 0; i < ImagePoints.size(); i++){
        double x = (ImagePoints[i].x-cx)/fx;
        double y = (ImagePoints[i].y-cy)/fy;

        in[i][0] = (x*WorldPoints[i].x);
        in[i][1] = (x*WorldPoints[i].y);
        in[i][2] = (x*WorldPoints[i].z);
        in[i][3] = (x);
        in[i][4] = (-y*WorldPoints[i].x);
        in[i][5] = (-y*WorldPoints[i].y);
        in[i][6] = (-y*WorldPoints[i].z);
        in[i][7] = (-y);

        std::cout<<i<<": ";
        for(size_t j = 0; j < 8; j++){
            std::cout<<in[i][j]<<" ";
        }
        std::cout<<std::endl;
    }
    //cv::SVD matrix(cv::Mat(8, WorldPoints.size(), CV_64F, in));
    cv::Mat A(WorldPoints.size(),8, CV_64F, in);
    cv::Mat w, u, vt;
    std::cout<<"Compute"<<std::endl;
    cv::SVD::compute(A, w, u, vt);
    std::cout<<A<<std::endl<<w<<std::endl<<u<<std::endl<<vt<<std::endl<<u*vt<<std::endl;

    std::cout<<"Bla: "<<w.cols<<" "<<w.rows<<std::endl;
    cv::Mat wi = cv::Mat::zeros(WorldPoints.size(),8, CV_64F);
    for(size_t i = 0; i < 8; i++){
        std::cout<<w.at<double>(0,i)<<std::endl;
        if(w.at<double>(i) != 0.0){
            wi.at<double>(i,i) = 1/w.at<double>(0,i);
        }else{
            wi.at<double>(i,i) = 0.0;
        }
    }
    std::cout<<"Test: "<<wi<<std::endl;

    std::ofstream myfile;
    myfile.open ("Matrix.txt", std::ios::in | std::ios::app);
    myfile <<A<<std::endl<<w<<std::endl<<u<<std::endl<<vt<<std::endl<<u*vt<<std::endl;//<<vt.t()*wi*u.t()<<std::endl;
    myfile.close();
    std::cout<<"Ende"<<std::endl;
}

int main()
{
    cv::Mat Testbild = cv::imread("/home/falko/Bilder/Bildschirmfoto von 20170408_124949A.mp4.png", -1);

    std::vector<cv::Point3f> WorldPoints;
    std::vector<cv::Point2f> ImagePoints;
    loadFromFile(WorldPoints, ImagePoints,Testbild);

    SVD(WorldPoints,ImagePoints,6245.985171734248, 6593.452572771233, 1340.851224182062, 755.2799615988556);

    std::vector<std::vector<cv::Point3f>> World;
    World.push_back(WorldPoints);
    std::vector<std::vector<cv::Point2f>> Image;
    Image.push_back(ImagePoints);

    cv::Mat cameraMatrix ;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs,tvecs;
    cv::Size imageSize(2688,1520);
    cv::calibrateCamera(World, Image, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout<<"cameraMatrix: "<<cameraMatrix<<std::endl<<"distCoeffs "<<distCoeffs<<std::endl;
    for(size_t i = 0; i < rvecs.size(); i++){
        std::cout<<rvecs[i]<<std::endl;
    }
    for(size_t i = 0; i < tvecs.size(); i++){
        std::cout<<tvecs[i]<<std::endl;
    }
    cv::Mat R;
    cv::Mat_<double> T(3,1);
    T.at<double>(0,0) = tvecs[0].at<double>(0,0);
    T.at<double>(1,0) = tvecs[0].at<double>(1,0);
    T.at<double>(2,0) = tvecs[0].at<double>(2,0);
    cv::Rodrigues(rvecs[0],R);

    cv::Mat_<double> R2(3,3);
    R2.at<double>(0,0) = 1;
    R2.at<double>(0,1) = 0;
    R2.at<double>(0,2) = 0;
    R2.at<double>(1,0) = 0;
    R2.at<double>(1,1) = 0;
    R2.at<double>(1,2) = 1;
    R2.at<double>(2,0) = 0;
    R2.at<double>(2,1) = 1;
    R2.at<double>(2,2) = 0;

    std::cout<<R<<std::endl<<R2<<std::endl<<R*R2<<std::endl<<R2*R<<std::endl;
    R2.at<double>(1,2) = -1;
    R2.at<double>(2,1) = 1;
    std::cout<<R*R2<<std::endl<<R2*R<<std::endl;
    R2.at<double>(1,2) = 1;
    R2.at<double>(2,1) = -1;
    std::cout<<R*R2<<std::endl<<R2*R<<std::endl;

    std::vector<cv::Point3f> PointTestIn;
    std::vector<cv::Point2f> PointTestOut;

    for(int i = -3; i <= 3; i++){
        for(int j = 2; j < 11; j++){
            PointTestIn.push_back(cv::Point3f(i*100.0, j*100.0, 0.0));
        }
    }
    cv::projectPoints(PointTestIn,rvecs[0],tvecs[0],cameraMatrix,distCoeffs,PointTestOut);

    for(size_t i = 0; i < PointTestOut.size(); i++){
        //std::cout<<PointTestIn[i]<<PointTestOut[i]<<std::endl;
        cv::putText(Testbild, std::to_string(i), PointTestOut[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0,255));
        cv::circle(Testbild,PointTestOut[i],4,cv::Scalar(0,255,0,255),-1);
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("Position_0.png",Testbild,compression_params);

    /*

    int h = 5;
    int w = 5;
    cv::Size patternsize(h,w);
    std::vector<cv::Point3f> realPoints;
    for(int i = 0; i < h*w; i++){
        realPoints.push_back(cv::Point3f((i/h)*0.031,(i%h)*0.031,0));
    }

    std::vector<std::vector<cv::Point2f> > m;

    bool set = true;
    cv::Size s(0,0);

    double avg_Quality = 0.0;
    double avg_Count = 0;
    double Quali_max = 0;
    int del=0;

//    std::vector<cv::VideoCapture> videos;
    std::vector<std::string> videos;

//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_0.mp4"));
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_0.mp4");
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_1.mp4"));
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_2.mp4"));
    /*
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_4.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_2.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_3.mp4"));
*/
    /*
    for(size_t i = 0; i < videos.size(); i++){ // Könnte parallel werden
        std::cout<<"Lade Video "<<i<<std::endl;
        cv::VideoCapture video(videos[i]);
        if(video.isOpened()){
            std::vector<cv::Mat> img;
            img.clear();
            cv::Mat frame_col;
            while (video.read(frame_col)) {
                img.push_back(cv::Mat(frame_col));
                if(set){
                    set = false;
                    s.width = frame_col.cols;
                    s.height= frame_col.rows;
                }
            }
            tbb::parallel_for(0, (int)img.size(), [&](int i){
//            for(size_t i = 0; i < img.size(); i++){
                cv::Mat gray;
                cvtColor(img[i], gray, CV_BGR2GRAY);
                equalizeHist(gray, gray);

                std::vector<cv::Point2f> corners;
                corners.clear();
                bool patternfound = cv::findChessboardCorners(gray, patternsize, corners,
                                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);
                if(patternfound){
                    bool run = true;
                    int search_size = std::max(3.0,sqrt(pow(corners.at(0).x - corners.at(h*w-1).x,2)
                                                        +pow(corners.at(0).y - corners.at(h*w-1).y,2))/(sqrt(h*h+w*w)*5));
                    for(size_t j = 0; j < corners.size(); j++){
                        int X = corners.at(j).x-search_size;
                        int Y = corners.at(j).y-search_size;

                        cv::Mat img_cur = gray(cv::Rect(X,Y,search_size*2,search_size*2));
                        cv::Mat img_out;

                        cornerHarris(img_cur, img_out, 3, 3, 0.04, cv::BORDER_DEFAULT);
                        double minVal, maxVal;
                        cv::Point minLoc, maxLoc;

                        cv::minMaxLoc(img_out, &minVal, &maxVal, &minLoc, &maxLoc );

                        avg_Quality += maxVal;
                        avg_Count++;
                        Quali_max = std::max(Quali_max,maxVal);

                        if(maxVal < 0.00002){// Schwellenwert wie unscharf eine Ecke sein darf, bei 1280-> 0.0006, Webcam: 0.00004
                            run = false;
                        }
                    }
                    if(run){
                        cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.04));
                        m.push_back(corners);
                    }else{
                        del++;
                    }
                }
            });
            std::cout<<"Gefunden: "<<m.size()<<" / "<<del<<std::endl;
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }

    std::cout << "Berechnung hat "<< m.size()<<"/"<<avg_Count<< " Bilder ergeben Qualität "<<avg_Quality/avg_Count<<" max "<<Quali_max<<" entfernt:"<<del<< std::endl;
    std::vector<std::vector<cv::Point2f> > m2;
    if(m.size() > 100){
        std::cout<<"If"<<std::endl;
        m2 = get_perfect_Points(m,s,300);
    }else{
        m2 = m;
    }
    std::cout<<"Nach perfect"<<std::endl;
    std::vector<std::vector<cv::Point3f> > p;
    for(size_t i = 0; i < m2.size(); i++){
        p.push_back(realPoints);
    }

//    std::vector<cv::Mat> rvecs,tvecs;
    std::cout << "Berechnung läuft auf "<< m2.size()<< " Bildern" << std::endl;
    if(m2.size() > 1){
        cv::Mat imgCal = cv::Mat::zeros(s.height,s.width, CV_8UC3 );
        int lineType = 8;
        int npt[] = { 4 };
        int colStep = 255/m2.size();

        for(size_t i = 0; i < m2.size(); i++){
            cv::Point rook_points[1][4];
            rook_points[0][0] = m2.at(i).at(20);
            rook_points[0][1] = m2.at(i).at(0);
            rook_points[0][2] = m2.at(i).at(4);
            rook_points[0][3] = m2.at(i).at(24);

            const cv::Point* ppt[1] = { rook_points[0] };

            cv::fillPoly( imgCal, ppt, npt, 1, cv::Scalar( 255-(i*colStep), 0, 255-((m2.size()-1-i)*colStep)), lineType );
        }
        cv::namedWindow("Kalibrierung",1);
        DisplayImage("Kalibrierung",imgCal);

//        cv::Mat cameraMatrix ;
//        cv::Mat distCoeffs;

        cv::calibrateCamera(p, m2, s, cameraMatrix, distCoeffs, rvecs, tvecs);
        std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;

        correct_Image(cameraMatrix,distCoeffs);

    }else{
        std::cout<<"Keine Kalibrierungsbilder gefunden"<<std::endl;
    }
    */
    return 0;
}

