#include <QCoreApplication>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <QStringList>
#include "LandmarkCoreIncludes.h"

cv::Vec4f compareEllipse(cv::RotatedRect Ellipse1, cv::RotatedRect Ellipse2, double scall){
    double x = Ellipse1.center.x;
    double y = Ellipse1.center.y;
    double abstandCenter = sqrt(pow(x-scall*(Ellipse2.center.x+282),2)+
                                pow(y-scall*(Ellipse2.center.y+224),2));
    float angle = Ellipse1.angle-Ellipse2.angle;
    if(angle < 0)
        angle *= -1.0;
    float width = Ellipse1.size.width - scall*Ellipse2.size.width;
    float height = Ellipse1.size.height - scall*Ellipse2.size.height;
    return cv::Vec4f(abstandCenter,angle,width,height);
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

cv::Mat setEye(cv::Mat Image, cv::Mat Eye, bool gray){
    cv::Mat ret;
    cv::Mat EyeGray;
    if(gray){
        convert_to_grayscale(Image, ret);
        convert_to_grayscale(Eye, EyeGray);
        cv::normalize(EyeGray, EyeGray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }else{
        ret = Image.clone();
    }
    cv::resize(ret,ret,cv::Size(0,0),0.65,0.65);

    int x = 224;
    int y = 282;
    for(int i=0;i<Eye.rows;i++){
        for(int j=0;j<Eye.cols;j++){
            if(Eye.type() == CV_8UC3){
                if(gray){
                    ret.at<uchar>(i+x,j+y) = EyeGray.at<uchar>(i,j);
                }else{
                    ret.at<cv::Vec3b>(i+x,j+y) = Eye.at<cv::Vec3b>(i,j);
                }
            }else if(Eye.type() == CV_8UC4){
                cv::Vec4b col = Eye.at<cv::Vec4b>(i,j);
                if(col[3] == 255){
                    if(gray){
                        ret.at<uchar>(i+x,j+y) = EyeGray.at<uchar>(i,j);
                    }else{
                        ret.at<cv::Vec3b>(i+x,j+y) = cv::Vec3b(col[0],col[1],col[2]);
                    }
                }
            }
        }
    }
    return ret;
}

std::string EllipseToTring(cv::RotatedRect Ellipse){
    return std::to_string(Ellipse.center.x)+" "+std::to_string(Ellipse.center.y)+"; "
          +std::to_string(Ellipse.size.width)+" "+std::to_string(Ellipse.size.height)+"; "+std::to_string(Ellipse.angle);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // Lade Bilder
    //-------------------------------------------------------------
    std::vector<std::string> mImagePaths;
    std::vector<cv::RotatedRect> mIris;
    std::vector<cv::RotatedRect> mPupils;
    std::vector<cv::Rect> mBoxes;

    cv::Mat Portrait = cv::imread("/home/falko/Uni/Master/Bilder/Learn/portrait.jpg");

    std::ifstream file("/home/falko/Uni/Master/workfile.txt");
    std::string line;
    QRegExp ausdruck("[\\(\\)\\,]");
    if(file.is_open()){
        while (std::getline(file, line)){
            std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
            std::string path;
            float Icx,Icy,Ih,Iw,Ia;
            float Pcx,Pcy,Ph,Pw,Pa;
            int Bx,By,Bw,Bh;
            iss >> path;
            iss >> Icx, iss >> Icy, iss >> Iw, iss >> Ih, iss >> Ia;
            iss >> Pcx, iss >> Pcy, iss >> Pw, iss >> Ph, iss >> Pa;
            iss >> Bx, iss >> By, iss >> Bw, iss >> Bh;
            //(x_centre,y_centre),(minor_axis,major_axis),angle
            //ellipse_i ellipse_p box))
            mImagePaths.push_back(path);
            mIris.push_back(cv::RotatedRect(cv::Point2f(Icx,Icy),cv::Size2f(Iw,Ih),Ia));
            mPupils.push_back(cv::RotatedRect(cv::Point2f(Pcx,Pcy),cv::Size2f(Pw,Ph),Pa));
            mBoxes.push_back(cv::Rect(Bx,By,Bw,Bh));
        }
    }

    // Lade OpenFace
    //-----------------------------------------------------

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

    // Programmablauf
    //-----------------------------------------------------

    std::ofstream myfileGes;
    myfileGes.open ("solution/Auge_ges.txt", std::ios::in | std::ios::app);
    for(double scall = 0.5; scall > 0.0; scall -= 0.02){
        std::cout<<"Skalierung "<<scall<<std::endl;
        size_t countGes = 0;
        std::ofstream myfile;
        myfile.open ("solution/Auge_"+std::to_string(scall)+".txt", std::ios::in | std::ios::app);
        for(size_t i = 0; i < mImagePaths.size(); i += 4){
            //for(int j = 0; j < 4; j++){
            bool found[] = {false, false, false, false};
            tbb::parallel_for(0, 4, [&](int j){
                if(i+j < mImagePaths.size()){
                    cv::Mat ImgEye = cv::imread(mImagePaths.at(i+j));
                    if(ImgEye.data){
                        cv::Mat_<uchar> grayscale_image = setEye(Portrait.clone(),ImgEye,true);
                        cv::resize(grayscale_image,grayscale_image,cv::Size(0,0),scall,scall);

                        clnf_models[j].Reset();

                        // Detect faces in an image
                        vector<cv::Rect_<double> > face_detections;

                        if(det_parameters[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR){
                            vector<double> confidences;
                            LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, clnf_models[j].face_detector_HOG, confidences);
                        }else{
                            LandmarkDetector::DetectFaces(face_detections, grayscale_image, clnf_models[j].face_detector_HAAR);
                        }

                        for(size_t face=0; face < face_detections.size(); ++face){
                            // if there are multiple detections go through them
                            found[j] = LandmarkDetector::DetectLandmarksInImage(grayscale_image, face_detections[face], clnf_models[j], det_parameters[j]);
                        }
                    }
                    //cv::ellipse(Img(mBoxes[i+j]),ellipse[j],cv::Scalar(255,255,0),2);
                    //cv::imshow("Ergebnis_"+std::to_string(j),Img);
                }
            });
            //cv::waitKey(300);
            for(int j = 0; j < 4; j++){
                if(found[j]){
                    countGes++;

//                    cv::Mat Img = setEye(Portrait,cv::imread(mImagePaths[1],-1),false);
//                    LandmarkDetector::Draw(Img, clnf_models[j].hierarchical_models[2]);

                    cv::Mat_<double> shape2D = clnf_models[j].hierarchical_models[2].detected_landmarks;
                    std::vector<cv::Point2f> pos, pos2;
                    int n = shape2D.rows/2;
                    for(size_t a = 20; a < 28; a++){
                        pos.push_back(cv::Point2f(shape2D.at<double>(a),shape2D.at<double>(a+n)));
                        pos2.push_back(cv::Point2f(shape2D.at<double>(a-20),shape2D.at<double>(a-20+n)));
                    }

                    cv::RotatedRect eyePupil = cv::fitEllipse(pos);
                    cv::RotatedRect eyeIris = cv::fitEllipse(pos2);

                    myfile<<compareEllipse(eyeIris,mIris[i+j],scall)<<" "
                         <<compareEllipse(eyePupil,mPupils[i+j],scall)<<std::endl;

//                    std::cout<<EllipseToTring(eyeIris)<<" | "<<EllipseToTring(mIris[i+j])<<" "<<compareEllipse(eyeIris,mIris[i+j],scall)<<std::endl
//                            <<EllipseToTring(eyePupil)<<" | "<<EllipseToTring(mPupils[i+j])<<" "<<compareEllipse(eyePupil,mPupils[i+j],scall)<<std::endl<<std::endl;
                }
            }
            break;
        }
        myfile.close();
        std::cout<<"Gefunden "<<countGes<<"/"<<mImagePaths.size()<<std::endl;
        myfileGes<<scall<<" Gefunden "<<countGes<<"/"<<mImagePaths.size()<<std::endl;
        if(countGes == 0){
            std::cout<<"Abbruch"<<std::endl;
            break;
        }
        sleep(90);// Vermeidung von überhitzung
    }
    myfileGes.close();
    std::cout<<"Ende"<<std::endl;

    return a.exec();
}
