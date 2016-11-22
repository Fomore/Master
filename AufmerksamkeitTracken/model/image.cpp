#include "image.h"

Image::Image()
{
    /*
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
    */
    mImagePaths.clear();
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_14.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_00.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_01.png");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_02.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_03.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_04.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_05.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_06.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_07.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_08.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_09.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_10.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_11.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_12.jpg");
    mImagePaths.push_back("/home/falko/Uni/Master/Film/face_13.jpg");

    ID = 0;
}

Image::~Image()
{

}

bool Image::getNextImage(cv::Mat& out){
    if(mImagePaths.size() <= ID){
        ID = 0;
        out = cv::Mat();
        return false;
    }
    bool ret = getImage(out);
    ID++;
    return ret;
}

bool Image::getImage(cv::Mat &out){
    out = cv::imread(mImagePaths.at(ID), -1);
    if(!out.data){
        std::cout<<"Fehler in Der ImageList"<<std::endl;
        return false;
    }
    return true;
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

// Diese Methode stammt von OpenFace
void Image::convert_to_grayscale(const cv::Mat& in, cv::Mat& out)
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
