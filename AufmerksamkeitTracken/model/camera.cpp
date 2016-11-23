#include "camera.h"

#include <iostream>
#include <math.h>

Camera::Camera(){
    init = true;
    ID = 0;
    camera_calibration("");
    correct_Image();
}

Camera::Camera(int id)
{
    init = true;
    setCameraParameter(id);
//    correct_Image();
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
    }else if(id = 3){
        // 1920P der 4K Actioncam (1080P einstellung), naja, eher fasch
        cameraMatrix = (cv::Mat_<double>(3,3) << 2663.074079845223, 0, 947.41241748433,
                        0, 2632.404666246407, 505.9801667119421,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -0.7289272231084237, 1.215115916326713, -0.006172816529476588, 0.009414413544519131, -3.664316355386734);
     }else if(id == 4){//2688P der 4k Actioncam (2.7K Einstellung)
        cameraMatrix = (cv::Mat_<double>(3,3) << 15373.97717428267, 0, 1321.996093444815,
                        0, 17764.65120151666, 735.3988503456478,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -18.60440739952032, 523.0811532248169, 0.02032995403105285, -0.04626945929212306, 8.68919963553518);
    }else if(id == 5){ // 3840P der 4K Actioncam (Rechte Ecken vollkomen daneben, aber sonst gut)
        cameraMatrix = (cv::Mat_<double>(3,3) <<  5651.959129794478, 0, 1833.779346065389,
                        0, 5613.841123206106, 962.5658045838203,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.484996965134229, 5.748404337479958, 0.008656338023441646, -0.01457077780605927, -12.09940654367966);
    }else if(id = 6){
        // 3840P der 4K Actioncam (Im Randbereich krümmung des Inhaltes)
        cameraMatrix = (cv::Mat_<double>(3,3) << 8814.572769059419, 0, 1875.385785348307,
                        0, 8871.4437844653, 971.4901818251557,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.915891109401613, 20.49816269684929, 0.07322823783978712, -0.03882332526060985, -142.3394617487301);
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

    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_2688_0.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_2688_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_2688_2.mp4"));

    /*
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_3840_1.mp4"));
    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_3840_2.mp4"));
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_3840_3.mp4"));
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_3840_4.mp4"));
//    videos.push_back(cv::VideoCapture("/home/falko/Uni/Master/KalibirierungDaten/Action_3840_5.mp4"));
*/

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
    cv::VideoCapture video("/home/falko/Uni/Master/KalibirierungDaten/Action_2688_1.mp4");
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
