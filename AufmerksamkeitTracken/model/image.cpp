#include "image.h"

Image::Image()
{
    mImagePaths.clear();
//    cv::glob("/home/falko/Uni/Master/Bilder/*g", mImagePaths, true);
    cv::glob("/home/falko/Uni/Master/Bilder/Learn/*g", mImagePaths, true);//erhalten durch cv::Mat img = get_Face_Image(read_image,155,280,36,42); auf Grau setzen

    ID = 0;
}

Image::~Image()
{

}

bool Image::getNextImage(cv::Mat& out){
    if(mImagePaths.size() <= ID){
        ID = 0;
        std::cout<<"Reset"<<std::endl;
        return false;
    }
    bool ret = getImage(out);
    ID++;
    return ret;
}

bool Image::getImage(cv::Mat &out){
    out = cv::imread(mImagePaths.at(ID), -1);
    if(out.data){
        if(out.cols < 200){
            int col = out.cols;
            double fx = (200.0)/out.cols;
            resize(out, out, cv::Size(), fx, fx, CV_INTER_LINEAR);
            convert_to_grayscale(out,out);
            std::cout<<"Bildbreite: "<<col<<" "<<fx<<" Skalliert: "<<200.0-(ID*10.0)<<" - "<<out.cols<<"/"<<out.rows<<std::endl;
        }
        return true;
    }else{
        std::cout<<"Fehler in Der ImageList"<<std::endl;
        return false;
    }
}

bool Image::getScallImage(cv::Mat &out){
    convert_to_grayscale(cv::imread("/home/falko/Uni/Master/Film/face_14.png", -1),out);
    if(out.data){
        int col = out.cols;
        double fx = (200.0-(ID*10.0))/out.cols;
        resize(out, out, cv::Size(), fx, fx, CV_INTER_LINEAR);
        std::cout<<"Bildbreite: "<<col<<" "<<fx<<" Skalliert: "<<200.0-(ID*10.0)<<" - "<<out.cols<<"/"<<out.rows<<std::endl;
        ID++;
        if(ID>=10){
            ID=0;
        }
        return true;
    }
    return false;
}

cv::Mat Image::get_Face_Image(cv::Mat image, int X, int Y, int Width, int Height){
    cv::Mat image_cut = image(cv::Rect(X,Y,Width,Height));
    if(Width < 200){
        double fx = 200.0/Width;
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

// Diese Methode stammt von http://www.qtcentre.org/threads/56482-efficient-way-to-display-opencv-image-into-Qt
QImage Image::MatToQImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }else if(mat.type()==CV_8UC3){    // 8-bits unsigned, NO. OF CHANNELS=3
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }else{
        //        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

void Image::saveImage(cv::Mat img, std::string name){
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite("Image/"+name+".png", img, compression_params);
}

int Image::getID(){
    return ID;
}
