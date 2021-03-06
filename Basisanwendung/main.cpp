#include <QCoreApplication>

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <ctime>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <QStringList>
#include "LandmarkCoreIncludes.h"

void calcDiffernez(std::string name,cv::Mat in , cv::Mat out, std::vector<int> compression_params){
    cv::Mat ret;
    cv::absdiff(in,out,ret);
    std::cout<<name<<": "<<cv::sum(ret)<<" "<<cv::sum(cv::sum(ret))<<std::endl;

    cv::imwrite(name+"_differenz.png",ret, compression_params);
}

void convert_to_grayscale(const cv::Mat& in, cv::Mat& out)
{
    if(in.channels() == 3)
    {
        // Make sure it's in a correct format
        if(in.depth() != CV_8U)
        {
            if(in.depth() == CV_16U)
            {
                cv::Mat tmp = in / 256;
                tmp.convertTo(tmp, CV_8U);
                cv::cvtColor(tmp, out, CV_BGR2GRAY);
            }
        }
        else
        {
            cv::cvtColor(in, out, CV_BGR2GRAY);
        }
    }
    else if(in.channels() == 4)
    {
        cv::cvtColor(in, out, CV_BGRA2GRAY);
    }
    else
    {
        if(in.depth() == CV_16U)
        {
            cv::Mat tmp = in / 256;
            out = tmp.clone();
        }
        else if(in.depth() != CV_8U)
        {
            in.convertTo(out, CV_8U);
        }
        else
        {
            out = in.clone();
        }
    }
}

void getCLNFBox(const LandmarkDetector::CLNF &model, double &X, double &Y, double &W, double &H){
    cv::Mat_<double> shape2D = model.detected_landmarks;

    int n = shape2D.rows/2;

    X = cvRound(shape2D.at<double>(0));
    Y = cvRound(shape2D.at<double>(n));
    W = cvRound(shape2D.at<double>(0));
    H = cvRound(shape2D.at<double>(n));
    for(size_t i = 0; i < n; ++i)// Beginnt bei 0 das Output-Format
    {
        double x = (shape2D.at<double>(i));
        double y = (shape2D.at<double>(i + n));
        X = min(X,x);
        Y = min(Y,y);
        W = max(W,x);
        H = max(H,y);
    }
    W = W-X;
    H = H-Y;
}

cv::Mat getMean(const vector<cv::Mat>& images)
{
    std::cout<<"Berechnung Mean: "<<images.size()<<std::endl;

    if (images.empty()) return cv::Mat();

    // Create a 0 initialized image to use as accumulator
    cv::Mat m(images[0].rows, images[0].cols, CV_64FC3);

    // Use a temp image to hold the conversion of each input image to CV_64FC3
    // This will be allocated just the first time, since all your images have
    // the same size.
    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat temp;
        // Convert the input images to CV_64FC3 ...
        images[i].convertTo(temp, CV_64FC3);
        m += temp;
    }
    m.convertTo(m, CV_8UC3, 1.0 / images.size());
    return m;
}

void calcHistogramm(const vector<cv::Mat>& images, cv::Mat Ref){
    int blau[257];
    int green[257];
    int red[257];
    for(int i = 0; i < 257; i++){
        blau[i] = 0;
        green[i] = 0;
        red[i] = 0;
    }
    std::cout<<"Histogramm "<< images.size() <<std::endl;

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    //tbb::parallel_for(0, (int)images.size(), [&](int i){
    bool save = true;
    for(size_t i = 0; i < images.size(); i++){
        cv::Mat Img;
        cv::absdiff(images[i],Ref,Img);
        for(int x = 0; x < Img.cols; x++){
            for(int y = 0; y < Img.rows; y++){
                cv::Vec3b pix = Img.at<cv::Vec3b>(y,x);
                blau[pix[0]]++;
                red[pix[1]]++;
                green[pix[2]]++;
            }
        }
    }
    for(int i = 0; i < 257; i++){
        std::cout<<blau[i]<<" "<<red[i]<<" "<<green[i]<<std::endl;
    }
}

void setNois8UC3(cv::Mat& Img, int Chace, int Step){
    int shift = Step/2;
    for(int x = 0; x < Img.cols; x++){
        for(int y = 0; y < Img.rows; y++){
            cv::Vec3b nois = Img.at<cv::Vec3b>(x,y);
            for(int i = 0; i < 3; i++){
                if(rand() %100 < Chace){
                    nois[i] = max(0,min(nois[i]+rand() %Step - shift,255));
                }
            }
            Img.at<cv::Vec3b>(x,y) = nois;
        }
    }
}
cv::Mat setNois8UC1(cv::Mat &Img, int Chace, int Step){
    int shift = Step/2;
    for(int x = 0; x < Img.rows; x++){
        for(int y = 0; y < Img.cols; y++){
            if(rand() %100 < Chace){
                Img.at<uchar>(x,y) = max(0,min(Img.at<uchar>(x,y)+rand() %Step - shift,255));
            }
        }
    }
}
void setNois8UC3(cv::Mat &Img, cv::Mat NoisImg){
    int Ny = rand() % (NoisImg.rows - Img.rows);
    int Nx = rand() & (NoisImg.cols - Img.cols);
    for(int x = 0; x < Img.cols; x++){
        for(int y = 0; y < Img.rows; y++){
            cv::Vec3b imgPix = Img.at<cv::Vec3b>(x,y);
            cv::Vec3b noisPix = NoisImg.at<cv::Vec3b>(Nx+x,Ny+y);
            for(int i = 0; i < 3; i++){
                imgPix[i] = min(imgPix[i]+noisPix[i],255);
            }
            Img.at<cv::Vec3b>(x,y) = imgPix;
        }
    }
}

void calcNois(){
    cv::VideoCapture video;
    video.open("/home/falko/Uni/Master/Film/Rauschen.mp4");
    cv::Mat Frame;
    video.set(CV_CAP_PROP_POS_FRAMES,20.0);
    double max = video.get(CV_CAP_PROP_FRAME_COUNT);

    std::cout<<"Nois Bestimmung:"<<std::endl;
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    bool save = true;
    vector<cv::Mat> images;
    while (video.read(Frame) && video.get(CV_CAP_PROP_POS_FRAMES) < max - 50.0) {
        if(Frame.data){
            if(save){
                cv::imwrite("Video_One.png",Frame, compression_params);
                save = false;
            }
            images.push_back(Frame);
        }
    }

    //cv::Mat MeanImage = getMean(images);
    cv::Mat MeanImage = cv::imread("/home/falko/Uni/Master/Bilder/Video_avg.png", -1);

    calcHistogramm(images, MeanImage);

    cv::imwrite("Video_avg.png",MeanImage, compression_params);
    std::cout<<"Ende"<<std::endl;
}

void getCorrectMatrix(std::string name, cv::Vec3d &Pos, cv::Vec3d &Rot){
    name.replace(name.size()-7,7,"pose.txt");
    //std::cout<<name<<std::endl;

    int x, y;
    double rotation [9];
    ifstream in(name);

    if (!in) {
        cout << "Cannot open file.\n";
        Pos = cv::Vec3d(0,0,0);
        Rot = cv::Vec3d(0,0,0);
    }

    for (x = 0; x < 3; x++) {
        for (y = 0; y < 3; y++) {
            in >> rotation[x*3+y];
        }
    }

    cv::Matx33d corrected_rotation(rotation);
    //std::cout<<corrected_rotation<<std::endl;
    Rot = LandmarkDetector::RotationMatrix2Euler(corrected_rotation);

    double distance [3];
    for(int i = 0; i < 3; i++){
        in >> distance[i];
    }
    Pos = cv::Vec3d(distance);

    in.close();
}

void toSection(LandmarkDetector::CLNF &clnf, double BoxX, double BoxY, double Scall)
{
    cv::Mat_<double> shape2D = clnf.detected_landmarks;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        if(Scall != 1.0){
            double x = shape2D.at<double>(pos);
            double y = shape2D.at<double>(pos + n);
            shape2D.at<double>(pos) = BoxX + x/Scall;
            shape2D.at<double>(pos + n) = BoxY + y/Scall;
        }else{
            shape2D.at<double>(pos) += BoxX;
            shape2D.at<double>(pos + n) += BoxY;
        }
    }
    clnf.detected_landmarks = shape2D.clone();

    clnf.params_global[0] = clnf.params_global[0]/Scall;
    clnf.params_global[4] = clnf.params_global[4]/Scall + BoxX;
    clnf.params_global[5] = clnf.params_global[5]/Scall + BoxY;

    for (size_t part = 0; part < clnf.hierarchical_models.size(); ++part){
        cv::Mat_<double> shape2D = clnf.hierarchical_models[part].detected_landmarks;

        int n = shape2D.rows/2;
        for(int pos = 0; pos < n; pos++){
            if(Scall != 1.0){
                double x = shape2D.at<double>(pos);
                double y = shape2D.at<double>(pos + n);
                shape2D.at<double>(pos) = BoxX + x/Scall;
                shape2D.at<double>(pos + n) = BoxY + y/Scall;
            }else{
                shape2D.at<double>(pos) += BoxX;
                shape2D.at<double>(pos + n) += BoxY;
            }
        }

        clnf.hierarchical_models[part].detected_landmarks = shape2D.clone();

        clnf.hierarchical_models[part].params_global[0] = clnf.hierarchical_models[part].params_global[0]/Scall;
        clnf.hierarchical_models[part].params_global[4] = clnf.hierarchical_models[part].params_global[4]/Scall + BoxX;
        clnf.hierarchical_models[part].params_global[5] = clnf.hierarchical_models[part].params_global[5]/Scall + BoxY;
    }
}

cv::Mat getSmallImage(std::string ImagePath, double Scall, cv::Size &ImageSize){
    cv::Mat Img = cv::imread(ImagePath, -1);
    if(Img.data){
        ImageSize.height = Img.rows;
        ImageSize.width = Img.cols;
        cv::Mat ret;
        cv::resize(Img, ret, cv::Size(0,0), Scall,Scall, CV_INTER_LINEAR);
        return ret;
    }else{
        ImageSize.height = ImageSize.width = 0;
        return Img;
    }
}

cv::Mat_<uchar> getResizeImage(cv::Mat_<uchar> Image, cv::Size ImageSize, int Typ){
    cv::Mat_<uchar> gray;
    cv::Mat_<uchar> grayNorm;
    cv::resize(Image, gray, ImageSize, 0, 0, Typ);
    cv::normalize(gray, grayNorm, 0, 255, cv::NORM_MINMAX);
    return grayNorm;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    srand (time(NULL));

    std::ifstream fileBox("Gesicht_Box_test.txt");
    vector<cv::Rect2d> Boxes;
    vector<int> BoxesID;
    double bx,by,bw,bh;
    int id;
    while(fileBox >> id, fileBox >> bx, fileBox >> by, fileBox >> bw, fileBox >> bh){
        double sw = bw *2.1;
        double sh = bh*2.1;
        double a = max(0.0,bx-(sw-bw)/2);
        double b = max(0.0,by-(sh-bh)/2);
        Boxes.push_back(cv::Rect2d(a,b,min(sw,640.0-a),min(sh,480.0-b)));
        BoxesID.push_back(id);
    }

    std::vector<cv::String> mImagePaths;
    std::string dataset = "Gesichter";//HeadPose  lfw-deepfunneled
    cv::glob("/home/falko/Uni/Master/Bilder/"+dataset+"/*.png", mImagePaths, true);
    bool getPos = true;

    vector<string> arguments;
    arguments.push_back(""); // Hat arguments keine Werte kann wes wegoptimiert werden und dadurch wirft die Initilaisierung unten Fehler

    /*
        -mloc - the location of landmark detection models
        -sigma -UpdateModelParameters
        -w_reg -
        -reg -
        -multi_view -
        -validate_detections -
        -n_iter -
        -gaze - indicate that gaze estimation should be performed
        -q - specifying to use quiet mode not visualizing output
        -wild - flag specifies when the images are more difficult, model considers extended search regions
    */
    // im build-Ordner muss das model von OpenFace sein
    LandmarkDetector::FaceModelParameters det_params(arguments); // Sollte Parameter beinhalten

    // Always track gaze in feature extraction
    det_params.track_gaze = true;

    det_params.use_face_template = true;
    // This is so that the model would not try re-initialising itself
    det_params.reinit_video_every = -1;

    det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;

    // Maximale Gesichter im Fil, Achtung: Schnitte und vieles Wechseln scherschlechtert die Qualität
    LandmarkDetector::CLNF clnf_model(det_params.model_location);
    clnf_model.face_detector_HAAR.load(det_params.face_detector_location);
    clnf_model.face_detector_location = det_params.face_detector_location;

    vector<LandmarkDetector::FaceModelParameters> det_parameters;
    // The modules that are being used for tracking
    vector<LandmarkDetector::CLNF> clnf_models;

    for (int i = 0; i < 4; ++i)
    {
        clnf_models.push_back(clnf_model);
        det_parameters.push_back(det_params);
    }
    double Scall,fy,cx,cy;
    //cx = 640; //Aus 640 mit best bei 0.95
    //cy = 480;
    Scall = fy = 531.15f;//Scall = 500 * (cx / 640.0);
    //fy = 500 * (cy / 480.0);
    cx = 640.0 / 2.0;
    cy = 480.0 / 2.0;
    std::cout<<"Anzahl: "<<mImagePaths.size()<<" "<<BoxesID.size()<<std::endl;

    int AnzDurchlauf = 1;
    for(size_t typ = 0; typ < 4; typ ++){
        std::ofstream myfilePose;
        myfilePose.open (dataset+"_Gesicht_nB_"+std::to_string(typ)+".txt", std::ios::in | std::ios::app);
        if(getPos){
            myfilePose<<"[PosX, PosY, PosZ] [RotX, RotY, RotZ] ";
        }
        myfilePose<<"Scall "<<"[PCTx, PCTy, PCTz, PCEul_x, PCEul_y, PCEul_z]"
                 <<" [PWTx, PWTy, PWTz, PWEul_x, PWEul_y, PWEul_z]"
                <<" [CPCTx, CPCTy, CPCTz, CPCEul_x, CPCEul_y, CPCEul_z]"
               <<" [CPWTx, CPWTy, CPWTz, CPWEul_x, CPWEul_y, CPWEul_z]"
              <<" BoxW BoxH"<<std::endl;

        for(double scall = 1.0; scall > 0.0; scall -= 0.06){
            std::time_t result = std::time(nullptr);
            std::cout<<"Berechnung auf "<<typ<<" "<<scall<<" "<< std::asctime(std::localtime(&result))<<std::endl;
//            std::cout<<"Berechnung auf "<<scall<<" "<< std::asctime(std::localtime(&result))<<std::endl;
            size_t count = 0, count_ges = 0;
            double avg_H = 0.0, avg_W = 0.0;
            for(size_t i = 0; i < mImagePaths.size(); i += 4){//BoxesID.size(); i+=4){
                for(int anz = 0; anz < AnzDurchlauf; anz++){
                bool success[] = {false, false, false, false};
                //for(int j = 0; j < 4; j++){
                tbb::parallel_for(0, 4, [&](int j){
                    if(i+j < mImagePaths.size()){
                        cv::Size ImageSize;
                        cv::Mat Img = getSmallImage(mImagePaths.at(i+j), scall, ImageSize);
                        if(Img.data){
                            cv::Mat_<uchar> gray;
                            //convert_to_grayscale(ret,grayscale_image);
                            convert_to_grayscale(Img,gray);

                            //setNois8UC1(grayscale_image1,50,51);

                            cv::Mat_<uchar> grayScallImage;
                            if(typ == 0){
                                grayScallImage = getResizeImage(gray,ImageSize, CV_INTER_NN);
                            }else if(typ == 1){
                                grayScallImage = getResizeImage(gray,ImageSize, CV_INTER_LINEAR);
                            }else if(typ == 2){
                                grayScallImage = getResizeImage(gray,ImageSize, CV_INTER_CUBIC);
                            }else{
                                grayScallImage = getResizeImage(gray,ImageSize, CV_INTER_LANCZOS4);
                            }
                            /*
                            cv::imwrite("B"+std::to_string(j)+".png",Img);
                            cv::imwrite("B"+std::to_string(j)+"G.png",grayScallImage);
                            std::cout<<"Bild "<<j<<ImageSize<<std::endl;
                            */
                            clnf_models[j].Reset();

                            // Detect faces in an image
                            vector<cv::Rect_<double> > face_detections;

                            if(det_params.curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                                vector<double> confidences;
                                LandmarkDetector::DetectFacesHOG(face_detections, grayScallImage, clnf_models[j].face_detector_HOG, confidences);
                            }else{
                                LandmarkDetector::DetectFaces(face_detections, grayScallImage, clnf_models[j].face_detector_HAAR);
                            }

                            for(size_t face=0; face < face_detections.size(); ++face){
                                // if there are multiple detections go through them
                                success[j] = LandmarkDetector::DetectLandmarksInImage(grayScallImage, face_detections[face], clnf_models[j], det_parameters[j]);
                            }
                            /*
                            if(success[j]){
                                LandmarkDetector::Draw(grayScallImage,clnf_models[j]);
                                cv::imwrite("B"+std::to_string(j)+"L.png",grayScallImage);
                            }
                            */
                            count_ges++;
                        }
                    }
                });
                for(int j = 0; j < 4; j++){
                    if(success[j]){
                        count++;

                        double x,y,w,h;
                        getCLNFBox(clnf_models[j],x,y,w,h);
                        if(getPos){
                            cv::Vec3d Pos;
                            cv::Vec3d Rot;
                            getCorrectMatrix(mImagePaths.at(i+j),Pos,Rot);
                            myfilePose<<Pos<<" "<<Rot<<" ";
                        }
                        myfilePose<<scall<<" "<<LandmarkDetector::GetPoseCamera(clnf_models[j],Scall,Scall,cx,cy)
                                 <<" "<<LandmarkDetector::GetPoseWorld(clnf_models[j],Scall,Scall,cx,cy)
                                <<" "<<LandmarkDetector::GetCorrectedPoseCamera(clnf_models[j],Scall,Scall,cx,cy)
                               <<" "<<LandmarkDetector::GetCorrectedPoseWorld(clnf_models[j],Scall,Scall,cx,cy)
                              <<" "<<w<<" "<<h<< std::endl;
                        avg_H += h;
                        avg_W += w;
                    }
                }
                }
                if(i%(800/AnzDurchlauf) == 0){
                    sleep(8);
                }
            }
            std::cout<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;

            if(count == 0){
                break;
            }
        }
        myfilePose.close();
        std::cout<<"Ende"<<std::endl;
    }
    return a.exec();
}
