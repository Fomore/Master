#include <QCoreApplication>

#include <stdlib.h>
#include <time.h>
#include <unistd.h>

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
void setNois8UC1(cv::Mat& Img, int Chace, int Step){
    int shift = Step/2;
    for(int x = 0; x < Img.cols; x++){
        for(int y = 0; y < Img.rows; y++){
            if(rand() %100 < Chace){
                Img.at<uchar>(x,y) = max(0,min(Img.at<uchar>(x,y)+rand() %Step - shift,255));
            }
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

int main(int argc, char *argv[])
{
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    srand (time(NULL));

    cv::imwrite("1.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(0,0,0)), compression_params);
    cv::imwrite("2.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(255,0,0)), compression_params);
    cv::imwrite("3.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(0,255,0)), compression_params);
    cv::imwrite("4.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(0,0,255)), compression_params);
    cv::imwrite("5.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(255,255,0)), compression_params);
    cv::imwrite("6.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(0,255,255)), compression_params);
    cv::imwrite("7.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(255,0,255)), compression_params);
    cv::imwrite("8.png",cv::Mat(200,300,CV_8UC3,cv::Scalar(255,255,255)), compression_params);

    cv::Mat tmp(300,300,CV_8UC3);
    cv::imwrite("Nois_1.png",tmp, compression_params);
    setNois8UC3(tmp,30,51);
    cv::imwrite("Nois_2.png",tmp, compression_params);

    cv::Mat tmp2(300,300,CV_8UC1);
    cv::imwrite("Nois_3.png",tmp2, compression_params);
    setNois8UC1(tmp2,30,51);
    cv::imwrite("Nois_4.png",tmp2, compression_params);

    //calcNois();

    std::ofstream myfile;
    myfile.open ("Auswertung_50_51.txt", std::ios::in | std::ios::app);

    std::vector<cv::String> mImagePaths;
    mImagePaths.clear();
    /*
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/Schachbrett.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena100.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena.jpg");
    */
    cv::glob("/home/falko/Uni/Master/lfw-deepfunneled/*.jpg", mImagePaths, true);

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

    for(size_t typ = 0; typ < 4; typ ++){
        if(typ == 0){
            std::cout<<"CV_INTER_NN"<<std::endl;
            myfile<<"CV_INTER_NN"<<std::endl;
        }else if(typ == 1){
            std::cout<<"CV_INTER_LINEAR"<<std::endl;
            myfile<<"CV_INTER_LINEAR"<<std::endl;
        }else if(typ == 2){
            std::cout<<"CV_INTER_CUBIC"<<std::endl;
            myfile<<"CV_INTER_CUBIC"<<std::endl;
        }else{
            std::cout<<"CV_INTER_LANCZOS4"<<std::endl;
            myfile<<"CV_INTER_LANCZOS4"<<std::endl;
        }
        for(double scall = 0.15; scall < 0.35; scall += 0.01){
            std::cout<<"Berechnung auf "<<scall<<" "<<0.01<<std::endl;
            myfile<<"Berechnung auf "<<scall<<" "<<0.01<<std::endl;
            size_t count = 0, count_ges = 0;
            double avg_H = 0.0, avg_W = 0.0;
            for(size_t i = 0; i < mImagePaths.size(); i += 4){
                tbb::parallel_for(0, 4, [&](int j){
                    //for(int j = 0; j < 4; j++){
                    if(i+j < mImagePaths.size()){
                        cv::Mat Img =  cv::imread(mImagePaths.at(i+j), -1);
                        if(Img.data){
                            cv::Mat ret;
                            cv::resize(Img, ret, cv::Size(0,0), scall,scall, CV_INTER_LINEAR);
                            for(int anz = 0; anz < 4; anz++){
                            bool success = false;
                            cv::Mat_<uchar> grayscale_image;
                            cv::Mat_<uchar> grayscale_image1;
                            convert_to_grayscale(ret,grayscale_image1);
                            setNois8UC1(grayscale_image1,50,51);

                            if(typ == 0){
                                cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_NN);
                            }else if(typ == 1){
                                cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_LINEAR);
                            }else if(typ == 2){
                                cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_CUBIC);
                            }else{
                                cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_LANCZOS4);
                            }

                            clnf_models[j].Reset();

                            // Detect faces in an image
                            vector<cv::Rect_<double> > face_detections;

                            if(det_params.curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                                vector<double> confidences;
                                LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[j].face_detector_HOG, confidences);
                            }else{
                                LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[j].face_detector_HAAR);
                            }

                            for(size_t face=0; face < face_detections.size(); ++face){
                                // if there are multiple detections go through them
                                success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_models[j], det_parameters[j]);
                            }

                            count_ges++;
                            if(success){
                                count++;
                                double x,y,w,h;
                                getCLNFBox(clnf_models[j],x,y,w,h);
                                avg_H += h;
                                avg_W += w;
                            }
                        }
                        }
                    }
                });

            }
            std::cout<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
            myfile<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
            sleep(50);// Vermeidung von überhitzung
        }

    }

    QCoreApplication a(argc, argv);

    myfile.close();
    return a.exec();
}
