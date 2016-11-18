#include "image.h"

Image::Image()
{
    std::string file = "/home/falko/Uni/Master/Film/Chor_01_Img_01.png";
    cv::Mat read_image = cv::imread(file, -1);
    if(!read_image.empty()){
        cv::namedWindow("Skalliertes Bild",1);
        cv::Mat img = get_Face_Image(read_image,155,280,36,42);
        imshow("Skalliertes Bild", img);

        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        imwrite("alpha.png", img, compression_params);
    }else{
        std::cout<<"Kein Bild"<<std::endl;
    }
}

Image::~Image()
{

}

cv::Mat Image::get_Face_Image(cv::Mat image, int X, int Y, int Width, int Height){
    cv::Mat image_cut = image(cv::Rect(X,Y,Width,Height));
    if(Width < 120){
        double fx = 120.0/Width;
        cv::Mat ret;
        resize(image_cut, ret, cv::Size(), fx, fx, CV_INTER_LINEAR);
        return ret;
    }
    return image_cut;
}
