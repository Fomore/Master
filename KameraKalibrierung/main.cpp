#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <tbb/parallel_for.h>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

void correct_Image(cv::Matx33d K, cv::Vec4d D, std::string path){
    cv::VideoCapture video(path);
    if(video.isOpened()){
        cv::Mat frame_col;
        cv::namedWindow("Image Raw",1);
        cv::namedWindow("Image Correct",1);
        while (video.read(frame_col)) {
            cv::Mat view;

            cv::fisheye::undistortImage(frame_col, view, K, D,K);

            DisplayImage("Image Raw", frame_col);
            DisplayImage("Image Correct",view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}
void correct_Image(cv::Mat cameraMatrix, cv::Mat distCoeffs, std::string path){
    cv::VideoCapture video(path);
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

            cv::undistort(frame_col, view, cameraMatrix, distCoeffs);
            //cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung

            DisplayImage("Image Raw", frame_col);
            DisplayImage("Image Correct",view);

            if(cv::waitKey(30) >= 0) break;
        }
    }
}

void loadFromFile(std::vector<cv::Point3f> &WorldPoints, std::vector<cv::Point2f> &ImagePoints, cv::Point3f CamPos, double Scall,
                  cv::Mat &img, std::string path){
    std::cout<<"Load Data:"<<path<<std::endl;
    std::ifstream file(path);
    std::string str;
    size_t count = 0;
    while (std::getline(file, str)){
        if(count >= 124){
            break;
        }
        std::vector<std::string> v;
        boost::split(v, str, boost::is_any_of(" ") );

        int xI = atoi(v[2].c_str());
        int yI = atoi(v[3].c_str());

        int xW = atoi(v[1].c_str());
        int zW = atoi(v[0].c_str());

        //cv::circle(img,cv::Point2i(xI,yI),4,cv::Scalar(xW*36,23*zW,xW*zW*3));
        cv::putText(img,std::to_string(xW)+"|"+std::to_string(zW)+":"+std::to_string(count),cv::Point2i(xI,yI),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255,255));

        ImagePoints.push_back(cv::Point2f(xI,yI));
        //WorldPoints.push_back(cv::Point3f((4-atoi(v[1].c_str()))*Scall+CamPos.x, 0+CamPos.y, atoi(v[0].c_str())*Scall+CamPos.z));
        WorldPoints.push_back(cv::Point3f((xW-4)*Scall+CamPos.x, 0+CamPos.y, zW*Scall+CamPos.z));

        count++;
    }
    file.close();
    for(size_t i = 0; i < WorldPoints.size(); i++){
        std::cout<<WorldPoints[i]<<ImagePoints[i]<<std::endl;
    }
    std::cout<<"Load ende"<<std::endl;
}

cv::Matx33d SVD(std::vector<cv::Point3f> WorldPoints, std::vector<cv::Point2f> ImagePoints, double cx, double cy,
         cv::Mat Image){
    double in[ImagePoints.size()][6];
    std::cout<<"Punkte"<<std::endl;
    for(size_t i = 0; i < ImagePoints.size(); i++){
        double x = ImagePoints[i].x-cx;
        double y = ImagePoints[i].y-cy;

        std::cout<<WorldPoints[i]<<std::endl;

        in[i][0] = x*WorldPoints[i].x;
        in[i][1] = x*WorldPoints[i].y;
        in[i][2] = x*WorldPoints[i].z;
        in[i][3] =-y*WorldPoints[i].x;
        in[i][4] =-y*WorldPoints[i].y;
        in[i][5] =-y*WorldPoints[i].z;
    }
    //cv::SVD matrix(cv::Mat(8, WorldPoints.size(), CV_64F, in));
    cv::Mat A(WorldPoints.size(),6, CV_64F, in);
    cv::Mat w, u, vt;
    std::cout<<"Compute"<<std::endl;
    cv::SVDecomp(A, w, u, vt, cv::SVD::FULL_UV);

    double min = w.at<double>(0,0);
    size_t lastI = 5;
    for(size_t i = 1; i < 6; i++){
        if(min > w.at<double>(0,i) && w.at<double>(0,i) >= 0.00001){
            lastI = i;
            min = w.at<double>(0,i);
        }
    }

    cv::Mat line = vt.row(lastI);
    double l1 = sqrt(line.at<double>(0,0)*line.at<double>(0,0)
                     +line.at<double>(0,1)*line.at<double>(0,1)
                     +line.at<double>(0,2)*line.at<double>(0,2));
    double l2 = sqrt(line.at<double>(0,3)*line.at<double>(0,3)
                     +line.at<double>(0,4)*line.at<double>(0,4)
                     +line.at<double>(0,5)*line.at<double>(0,5));

    cv::Mat R = cv::Mat::zeros(3,3, CV_64F);
    R.at<double>(1,0) = line.at<double>(0,0)/l1;
    R.at<double>(1,1) = line.at<double>(0,1)/l1;
    R.at<double>(1,2) = line.at<double>(0,2)/l1;

    R.at<double>(0,0) = line.at<double>(0,3)/l2;
    R.at<double>(0,1) = line.at<double>(0,4)/l2;
    R.at<double>(0,2) = line.at<double>(0,5)/l2;

    R.at<double>(2,0) = R.at<double>(0,1)*R.at<double>(1,2)-R.at<double>(0,2)*R.at<double>(1,1);
    R.at<double>(2,1) = R.at<double>(0,2)*R.at<double>(1,0)-R.at<double>(0,0)*R.at<double>(1,2);
    R.at<double>(2,2) = R.at<double>(0,0)*R.at<double>(1,1)-R.at<double>(0,1)*R.at<double>(1,0);

    cv::Mat Rw, Ru, Rvt;
    cv::SVDecomp(R, Rw, Ru, Rvt, cv::SVD::FULL_UV);
    cv::Matx33d R2 = cv::Matx33d(Ru)*cv::Matx33d(Rvt);

    bool posTerm = R2.val[0]*WorldPoints[0].x + R2.val[1]*WorldPoints[0].y+R2.val[2]*WorldPoints[0].z >= 0;
    bool posX = ImagePoints[0].x-cx >= 0;

    if(posTerm != posX){
        R.at<double>(1,0) = -line.at<double>(0,0)/l1;
        R.at<double>(1,1) = -line.at<double>(0,1)/l1;
        R.at<double>(1,2) = -line.at<double>(0,2)/l1;

        R.at<double>(0,0) = -line.at<double>(0,3)/l2;
        R.at<double>(0,1) = -line.at<double>(0,4)/l2;
        R.at<double>(0,2) = -line.at<double>(0,5)/l2;

        R.at<double>(2,0) = R.at<double>(0,1)*R.at<double>(1,2)-R.at<double>(0,2)*R.at<double>(1,1);
        R.at<double>(2,1) = R.at<double>(0,2)*R.at<double>(1,0)-R.at<double>(0,0)*R.at<double>(1,2);
        R.at<double>(2,2) = R.at<double>(0,0)*R.at<double>(1,1)-R.at<double>(0,1)*R.at<double>(1,0);

        cv::SVDecomp(R, Rw, Ru, Rvt, cv::SVD::FULL_UV);
        R2 = cv::Matx33d(Ru)*cv::Matx33d(Rvt);
    }

    double fx = (ImagePoints[0].x-cx)*(R2.val[6]*WorldPoints[0].x + R2.val[7]*WorldPoints[0].y + R2.val[8]*WorldPoints[0].z)/
            (R2.val[0]*WorldPoints[0].x + R2.val[1]*WorldPoints[0].y + R2.val[2]*WorldPoints[0].z);
    double fy = fx/(l2/l1);

    std::cout<<"Orthogonal: "<<R2<<std::endl<<fx<<" "<<fy<<std::endl;

    for(size_t i = 0; i < WorldPoints.size(); i++){
        cv::circle(Image,ImagePoints[i],4,cv::Scalar(255,0,0,255));
        cv::Vec3d pos = R2*cv::Vec3d(WorldPoints[i].x, WorldPoints[i].y, WorldPoints[i].z);
        cv::Point2d point(pos[0]/pos[2]*fx+cx,pos[1]/pos[2]*fy+cy);
        //std::cout<<pos<<point<<std::endl;
        cv::circle(Image,point,4,cv::Scalar(0,0,255,255),-1);
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("Position_1.png",Image,compression_params);

    std::ofstream myfile;
    myfile.open ("Matrix.txt", std::ios::in | std::ios::app);
    myfile <<R2<<fx<<" "<<fy<<std::endl;
    myfile.close();
    std::cout<<"Ende"<<std::endl;

    return R2;
}

void print3D(std::vector<cv::Point3f> WorldPoints, cv::Matx33d Rotation){
    // Create a window
    cv::viz::Viz3d myWindow("Coordinate Frame");
    cv::viz::Viz3d myWindowRef("Coordinate Frame 2");

    // Add coordinate axes
    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    myWindowRef.showWidget("Coordinate Widget 2", cv::viz::WCoordinateSystem());


    cv::Point3d cam_pos(0.0,0.0,0.0), cam_focal_point(0.0,0.0,500.0), cam_y_dir(0.0,0.0,0.0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    myWindow.setViewerPose(cam_pose);

    cam_focal_point.y = 200.0;
    cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    myWindowRef.setViewerPose(cam_pose);

    for(size_t i = 0; i < WorldPoints.size(); i++){
        cv::Vec3d Pos = cv::Vec3d(WorldPoints[i].x, WorldPoints[i].y, WorldPoints[i].z);
        cv::Vec3d Position = Rotation*Pos;

        std::cout<<Pos<<Position<<std::endl;

        // Construct a Point widget
        cv::viz::WSphere sphere_widget(Position,5);
        sphere_widget.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
        myWindow.showWidget("Sphere Widget"+to_string(i), sphere_widget);

        // Construct a referenz Point widget
        cv::viz::WSphere sphere_widget2(Pos,5);
        sphere_widget2.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
        myWindowRef.showWidget("Sphere Widget"+to_string(i), sphere_widget2);
    }

    while(!myWindow.wasStopped() && !myWindowRef.wasStopped())
    {
        myWindow.spinOnce(1, true);
        myWindowRef.spinOnce(1, true);
    }
}

void loadFromFile(std::vector<std::vector<cv::Point2f> > &Landmarks, std::string path){
    std::ifstream file(path);
    std::string str;
    while (std::getline(file, str)){
        std::vector<cv::Point2f> points;
        std::vector<std::string> v;
        boost::split(v, str, boost::is_any_of("]["));
        for(size_t i = 1; i < v.size(); i += 2){
            std::vector<std::string> pos;
            boost::split(pos, v[i], boost::is_any_of(", "));
            if(pos.size() == 3){
                points.push_back(cv::Point2f(std::stof(pos[0]), std::stof(pos[2])));
            }
        }
        if(points.size() == 25){
            Landmarks.push_back(points);
        }
    }
}

int main()
{
    /*
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::Mat Testbild = cv::imread("/home/falko/Bilder/Bildschirmfoto von Test_Positionen_1.mp4.png", -1);
    cv::Point3f translation(0, 206, 31);

    //cv::Mat Testbild = cv::imread("/home/falko/Bilder/Bildschirmfoto von 20170408_124949A.mp4.png", -1);
    //cv::Point3f translation(0, 148+40, 0);

    std::vector<cv::Point3f> WorldPoints;
    std::vector<cv::Point2f> ImagePoints;
    //loadFromFile(WorldPoints, ImagePoints,translation,100,Testbild,"/home/falko/Uni/Master/Dateien/Positions_Data5.txt");
    loadFromFile(WorldPoints, ImagePoints,translation,100,Testbild,"/home/falko/Uni/Master/Dateien/Positions_Data.txt");

    cv::imwrite("Position.png",Testbild,compression_params);

    cv::Matx33d rotation = SVD(WorldPoints,ImagePoints, 1344, 756, Testbild.clone());

    print3D(WorldPoints,rotation);

    */

    //-----------------------------------------

    /*
    cv::Matx33d K1(855.6169740620294, 0, 1343.5,
                  0, 855.6169740620294, 759.5,
                  0, 0, 1);
    cv::Vec4d D1(0, 0, 0, 0);

    correct_Image(K1,D1);
    */

    int h = 5;
    int w = 5;
    cv::Size patternsize(h,w);
    std::vector<cv::Point3f> realPoints;
    for(int i = 0; i < h*w; i++){
        realPoints.push_back(cv::Point3f((i/h)*0.031,(i%h)*0.031,0));
    }

    std::vector<std::vector<cv::Point2f> > m;

    std::vector<std::string> videos;
    /*
    cv::Size s(640,480);
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_0.mp4");
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_1.mp4");
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_2.mp4");
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_3.mp4");
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_4.mp4");
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Webcam_640_5.mp4");
    */

    cv::Size s(2688,1520);
    videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_1.mp4");
    //videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_2.mp4");
    //videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_3.mp4");
    //videos.push_back("/home/falko/Uni/Master/KalibirierungDaten/Action_Box_4.mp4");

    std::ofstream myfile;
    loadFromFile(m,"FoundLandmarkSchach_Action_Box.txt");
    /*
    myfile.open ("./FoundLandmarkSchach.txt", std::ios::in | std::ios::app);

    for(size_t i = 0; i < videos.size(); i++){ // Könnte parallel werden
        std::cout<<"Lade Video "<<i<<std::endl;
        cv::VideoCapture video(videos[i]);
        if(video.isOpened()){
            cv::Mat frame_col[4];
            size_t frameCount = 0;
            size_t frameFound = 0;
            double videoFPS = video.get(CV_CAP_PROP_FPS);
            double maxVideoFrame = video.get(cv::CAP_PROP_FRAME_COUNT);
            size_t step = (size_t)min(videoFPS*10.0,maxVideoFrame/100.0);
            while (video.read(frame_col[frameCount%4])) {
                frameCount++;
                if(frameCount % step == 0){
                    std::cout<<"Berechnung:"<<frameCount<<"/"<<(size_t)maxVideoFrame<<" "<<cvRound((double)frameCount/maxVideoFrame*100)<<std::endl;
                }
                if(frameCount%4 == 0 || frameCount >= (size_t)maxVideoFrame){
                    std::vector<cv::Point2f> corners[4];
                    bool patternfound[4];
                    tbb::parallel_for(0, 4, [&](int i){
                    //for(int i = 0; i < 4; i++){
                        if(!frame_col[i].empty()){
                            corners[i].clear();
                            patternfound[i] = cv::findChessboardCorners(frame_col[i], patternsize, corners[i],
                                                                        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);
                        }
                    });
                    for(int i = 0; i< 4; i++){
                        if(patternfound[i]){
                            frameFound++;
                            m.push_back(corners[i]);
                            for(size_t pos = 0; pos < corners[i].size(); pos++){
                                myfile <<corners[i][pos];
                            }
                            myfile<<std::endl;
                        }
                        frame_col[i].release();
                    }
                }
            }
            std::cout<<"Gefunden: "<<frameFound<<" / "<<(size_t)maxVideoFrame<<std::endl;
        }else{
            std::cout<<"Datei nicht gefunden"<<std::endl;
        }
    }
    myfile.close();    
    */

    std::cout << "Berechnung hat "<< m.size()<<" Bilder ergeben"<<std::endl;
    std::vector<std::vector<cv::Point2f> > m2;
    if(m.size() > 100){
        m2 = get_perfect_Points(m,s,300);
    }else{
        m2 = m;
    }

    std::vector<std::vector<cv::Point3f> > p;
    for(size_t i = 0; i < m2.size(); i++){
        p.push_back(realPoints);
    }

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

        std::cout<<"Darstellung der Landmarks"<<s<<std::endl;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        cv::imwrite("Kalibrierung.png",imgCal);

        std::cout<<"Kalibrierung: p="<<p.size()<<" ,m2="<<m2.size()<<" "<<s<<std::endl;

        cv::Mat cameraMatrix ;
        cv::Mat distCoeffs;
        std::vector<cv::Mat> rvecs,tvecs;
        cv::calibrateCamera(p, m2, s, cameraMatrix, distCoeffs, rvecs, tvecs);
        std::cout <<"Kalibrierung Normal:" << std::endl << cameraMatrix << std::endl << distCoeffs << std::endl;

        cv::Matx33d K;
        cv::Vec4d D;
        std::vector<cv::Vec3d> rvec;
        std::vector<cv::Vec3d> tvec;
        cv::fisheye::calibrate(p, m2, s, K, D, rvec, tvec);
        std::cout <<"Kalibrierung Fischauge:" << std::endl << K << std::endl << D << std::endl;
        /*
        [855.6169740620294, 0, 1343.5;
         0, 855.6169740620294, 759.5;
         0, 0, 1]
        [0, 0, 0, 0]
*/
        myfile.open ("./Normal_Werte.txt", std::ios::in | std::ios::app);
        myfile <<cameraMatrix<<std::endl<<distCoeffs<<std::endl;
        myfile.close();

        myfile.open ("./Fischauge_Werte.txt", std::ios::in | std::ios::app);
        myfile <<K<<std::endl<<D<<std::endl;
        myfile.close();

        correct_Image(cameraMatrix,distCoeffs,videos[0]);
        correct_Image(K,D,videos[0]);

    }else{
        std::cout<<"Keine Kalibrierungsbilder gefunden"<<std::endl;
    }

    return 0;
}

