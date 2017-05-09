#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include <QStringList>

void calcDiffernez(std::string name,cv::Mat in , cv::Mat out){
    cv::Mat ret = cv::abs(in-out);
    std::cout<<name<<": "<<cv::sum(cv::sum(ret))<<std::endl;

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite(name+"_differenz.png",ret, compression_params);
}

int main(int argc, char *argv[])
{
    std::vector<cv::String> mImagePaths;
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/Schachbrett.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena100.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Bilder/Learn/lena.jpg");
    //cv::glob("/home/falko/Uni/Master/Bilder/Test/*.*g", mImagePaths, true);

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    for(int i = 0; i < 2; i++){
    cv::Mat Img =  cv::imread(mImagePaths.at(i), -1);
    cv::Mat ref =  cv::imread(mImagePaths.at(2), -1);
        std::string name = QString::fromStdString(mImagePaths.at(i)).split("/").last().toStdString();


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

        calcDiffernez(name+"_NN",ret_NN,ref);
        calcDiffernez(name+"_LINEAR",ret_LI,ref);
        calcDiffernez(name+"_CUBIC",ret_CU,ref);
        calcDiffernez(name+"_LANCZOS4",ret_LA,ref);

        cv::imwrite(name+"_NN.png",ret_NN, compression_params);
        cv::imwrite(name+"_LINEAR.png",ret_LI, compression_params);
        cv::imwrite(name+"_CUBIC.png",ret_CU, compression_params);
        cv::imwrite(name+"_LANCZOS4.png",ret_LA, compression_params);
       }
    QCoreApplication a(argc, argv);

    return a.exec();
}
