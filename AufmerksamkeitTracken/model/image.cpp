#include "image.h"

Image::Image()
{
}

Image::~Image()
{

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

void Image::toGrayGleam(const cv::Mat &Image, cv::Mat &Out)
{
    cv::Mat grayImage;
    if(Image.channels() == 3 || Image.channels() == 4){
        if(Image.depth() == CV_16U){
            cv::Mat tmp = Image / 256;
            tmp.convertTo(Image, CV_8U);
        }
        grayImage=cv::Mat::zeros(Image.size(),CV_8UC1);
        for(int i=0;i<Image.rows;i++){
            for(int j=0;j<Image.cols;j++){
                if(Image.type() == CV_8UC4){
                    cv::Vec4b pix = Image.at<cv::Vec4b>(i,j);
                    if(pix[3] == 255){
                        double faktor = (pow(pix[0]/255.0,1.0/2.2) + pow(pix[1]/255.0,1.0/2.2) + pow(pix[2]/255.0,1.0/2.2))/3.0;
                        int colore = std::max(0,std::min(255,(int)(255.0*faktor + 0.5)));
                        grayImage.at<uchar>(i,j) = colore;
                    }else{
                        grayImage.at<uchar>(i,j) = 255;
                    }
                }else if(Image.type() == CV_8UC3){
                    cv::Vec3b pix = Image.at<cv::Vec3b>(i,j);
                    double faktor = (pow(pix[0]/255.0,1.0/2.2) + pow(pix[1]/255.0,1.0/2.2) + pow(pix[2]/255.0,1.0/2.2))/3.0;
                    int colore = std::max(0,std::min(255,(int)(255.0*faktor + 0.5)));
                    grayImage.at<uchar>(i,j) = colore;
                }
            }
        }
    }else{
        convert_to_grayscale(Image,grayImage);
    }
    Out = normalize(grayImage,Image);
}

cv::Mat Image::normalize(cv::Mat GrayImage, cv::Mat Image){
    cv::Mat gaus;
    cv::GaussianBlur(GrayImage, gaus, cv::Size(7,7), 1.5, 1.5);

    int minGray = 255, maxGray = 0;
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            if(!(Image.type() == CV_8UC4 && Image.at<cv::Vec4b>(i,j)[3] != 255)){
                int colore = gaus.at<uchar>(i,j);
                maxGray = std::max(maxGray,colore);
                minGray = std::min(minGray,colore);
            }
        }
    }

    cv::Mat out=cv::Mat::zeros(Image.size(),CV_8UC1);
    double a = (255.0+minGray)/maxGray;
    for(int i=0;i<Image.rows;i++){
        for(int j=0;j<Image.cols;j++){
            out.at<uchar>(i,j)= std::min(255,std::max((int)(GrayImage.at<uchar>(i,j)*a-minGray+0.5),0));
        }
    }
    return out;
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

void Image::CLAHE(cv::Mat in, cv::Mat &out, double clip)
{
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(clip);
    clahe->apply(in,out);
}

void Image::Histogram(cv::Mat in, cv::Mat &out)
{
    cv::equalizeHist(in,out);
}
