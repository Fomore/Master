#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include <QStringList>
#include "LandmarkCoreIncludes.h"

void calcDiffernez(std::string name,cv::Mat in , cv::Mat out, std::vector<int> compression_params){
    cv::Mat ret = cv::abs(in-out);
    std::cout<<name<<": "<<cv::sum(ret)<<" "<<cv::sum(cv::sum(ret))<<std::endl;

    cv::imwrite(name+"_differenz.png",ret, compression_params);
}

void resizeImage(std::string name, cv::Mat Img, cv::Mat ref, std::vector<int> compression_params){
    cv::Mat ret_NN, ret_LI, ret_CU, ret_LA;
    cv::resize(Img, ret_NN, cv::Size(512,512), 0,0, CV_INTER_NN);
    cv::resize(Img, ret_LI, cv::Size(512,512), 0,0, CV_INTER_LINEAR);
    cv::resize(Img, ret_CU, cv::Size(512,512), 0,0, CV_INTER_CUBIC);
    cv::resize(Img, ret_LA, cv::Size(512,512), 0,0, CV_INTER_LANCZOS4);

    cv::imshow(name,Img);
    cv::imshow(name+"NN",ret_NN);
    cv::imshow(name+"LINEAR",ret_LI);
    cv::imshow(name+"CUBIC",ret_CU);
    cv::imshow(name+"LANCZOS4",ret_LA);

    calcDiffernez(name+"_NN",ret_NN,ref,compression_params);
    calcDiffernez(name+"_LINEAR",ret_LI,ref,compression_params);
    calcDiffernez(name+"_CUBIC",ret_CU,ref,compression_params);
    calcDiffernez(name+"_LANCZOS4",ret_LA,ref,compression_params);

    cv::imwrite(name+"_NN.png",ret_NN, compression_params);
    cv::imwrite(name+"_LINEAR.png",ret_LI, compression_params);
    cv::imwrite(name+"_CUBIC.png",ret_CU, compression_params);
    cv::imwrite(name+"_LANCZOS4.png",ret_LA, compression_params);

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

int main(int argc, char *argv[])
{
    std::ofstream myfile;
    myfile.open ("Auswertung.txt", std::ios::in | std::ios::app);

    std::vector<cv::String> mImagePaths;
    mImagePaths.clear();
    /*
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/Schachbrett.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena100.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena.jpg");
    */
    cv::glob("/home/falko/Uni/Master/lfw-deepfunneled/*.jpg", mImagePaths, true);

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

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

    cv::Mat ref =  cv::imread(mImagePaths.at(2), -1);

    double fx=260,fy=260,cx=125,cy=125;

    std::cout<<"Anzahl: "<<mImagePaths.size()<<std::endl;
    myfile<<"Anzahl: "<<mImagePaths.size()<<std::endl;

    bool run = true;
    double step = 0.1;
    for(double scall = 1.0; scall > 0.0 && run; scall -= step){
        std::cout<<"Berechnung auf "<<scall<<" "<<step<<std::endl;
        myfile<<"Berechnung auf "<<scall<<" "<<step<<std::endl;
        size_t count = 0, count_ges = 0;
        double avg_H = 0.0, avg_W = 0.0;
        for(size_t i = 0; i < mImagePaths.size(); i++){
            cv::Mat Img =  cv::imread(mImagePaths.at(i), -1);
            if(Img.data){
                std::string name = QString::fromStdString(mImagePaths.at(i)).split("/").last().toStdString();

                cv::Mat ret;
                cv::resize(Img, ret, cv::Size(0,0), scall,scall, CV_INTER_LINEAR);

                bool success = false;
                cv::Mat_<uchar> grayscale_image;
                convert_to_grayscale(ret,grayscale_image);
                //convert_to_grayscale(Img,grayscale_image);

                clnf_model.Reset();

                // Detect faces in an image
                vector<cv::Rect_<double> > face_detections;

                if(det_params.curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                    vector<double> confidences;
                    LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_model.face_detector_HOG, confidences);
                }else{
                    LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_model.face_detector_HAAR);
                }

                for(size_t face=0; face < face_detections.size(); ++face){
                    // if there are multiple detections go through them
                    success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_model, det_params);
                }

                count_ges++;
                if(success){
                    count++;
                    double x,y,w,h;
                    getCLNFBox(clnf_model,x,y,w,h);
                    avg_H += h;
                    avg_W += w;
                }
            }
        }
        std::cout<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
        myfile<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
        if(scall >= 0.51){
            step = 0.1;
        }else{
            step = 0.01;
        }
        if(count == 0){
            run = false;
        }
    }
    //-----------------------------------------

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
        for(double scall = 0.01; scall < 0.5; scall += 0.01){
            std::cout<<"Berechnung auf "<<scall<<" "<<step<<std::endl;
            myfile<<"Berechnung auf "<<scall<<" "<<step<<std::endl;
            size_t count = 0, count_ges = 0;
            double avg_H = 0.0, avg_W = 0.0;
            for(size_t i = 0; i < mImagePaths.size(); i++){
                cv::Mat Img =  cv::imread(mImagePaths.at(i), -1);
                if(Img.data){
                    cv::Mat ret;
                    cv::resize(Img, ret, cv::Size(0,0), scall,scall, CV_INTER_LINEAR);

                    bool success = false;
                    cv::Mat_<uchar> grayscale_image;
                    cv::Mat_<uchar> grayscale_image1;
                    convert_to_grayscale(ret,grayscale_image1);

                    if(typ == 0){
                        cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_NN);
                    }else if(typ == 1){
                        cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_LINEAR);
                    }else if(typ == 2){
                        cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_CUBIC);
                    }else{
                        cv::resize(grayscale_image1, grayscale_image, cv::Size(300,300), 0,0, CV_INTER_LANCZOS4);
                    }

                    clnf_model.Reset();

                    // Detect faces in an image
                    vector<cv::Rect_<double> > face_detections;

                    if(det_params.curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                        vector<double> confidences;
                        LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_model.face_detector_HOG, confidences);
                    }else{
                        LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_model.face_detector_HAAR);
                    }

                    for(size_t face=0; face < face_detections.size(); ++face){
                        // if there are multiple detections go through them
                        success = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_model, det_params);
                    }

                    count_ges++;
                    if(success){
                        count++;
                        double x,y,w,h;
                        getCLNFBox(clnf_model,x,y,w,h);
                        avg_H += h;
                        avg_W += w;
                    }
                }
            }
            std::cout<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
            myfile<<scall<<" Gefunden: "<<count<<"/"<<count_ges<<" - "<<avg_H/((double)count)<<"/"<<avg_W/((double)count)<<std::endl;
        }

    }

    QCoreApplication a(argc, argv);

    myfile.close();
    return a.exec();
}
