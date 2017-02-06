#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

std::vector<std::vector<cv::Point2f> > get_perfect_Points(std::vector<std::vector<cv::Point2f> > points, const cv::Size dim, int maxImages){
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
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_1.mp4");
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

int main()
{
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
    double avg_Count = 0;
    double Quali_max = 0;
    int del=0;

    std::vector<cv::VideoCapture> videos;

    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_3.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_5.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_6.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_2.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_4.mp4"));

    for(size_t i = 0; i < videos.size(); i++){ // Könnte parallel werden
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

                        if(maxVal < 0.01){// Schwellenwert wie unscharf eine Ecke sein darf, bei 1280-> 0.0006, Webcam: 0.00004
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
            }
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }

    std::cout << "Berechnung hat "<< m.size()<< " Bilder ergeben Qualität "<<avg_Quality/avg_Count<<" max "<<Quali_max<<" entfernt:"<<del<< std::endl;
    std::vector<std::vector<cv::Point2f> > m2;
    if(m.size() > 100){
        m2 = get_perfect_Points(m,s,120);
    }else{
        m2 = m;
    }
    for(size_t i = 0; i < m2.size(); i++){
        p.push_back(realPoints);
    }

    std::vector<cv::Mat> rvecs,tvecs;
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

        cv::Mat cameraMatrix ;
        cv::Mat distCoeffs;

        cv::calibrateCamera(p, m2, s, cameraMatrix, distCoeffs, rvecs, tvecs);
        std::cout <<"Neue Kalibrierung:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;

    }else{
        std::cout<<"Keine Kalibrierungsbilder gefunden"<<std::endl;
    }
    return 0;
}

