#include "imagesection.h"

ImageSection::ImageSection(int width, int height)
{
    mImageSize.width = width;
    mImageSize.height = height;
    Rec_new.x = Rec_new.y = Rec_new.width = Rec_new.height = 0;
    Rec_old.x = Rec_old.y = Rec_old.width = Rec_old.height = 0;
    mMinSize.width = 150;
    mMinSize.height = 200;
}


ImageSection::~ImageSection()
{

}

bool ImageSection::getSection(int &x, int &y, int &w, int &h){
    x = Rec_new.x;
    y = Rec_new.y;
    w = Rec_new.width;
    h = Rec_new.height;
    return true;
}

cv::Rect ImageSection::getRect()
{
    return Rec_new;
}

void ImageSection::getImage(cv::Mat Image,cv::Mat &Part)
{
    if(fx > 1.0){
        cv::resize(Image(Rec_new),Part,cv::Size(0,0),fx,fx);
    }else{
        Part = Image(Rec_new);
    }
}

void ImageSection::newRect(cv::Rect rec)
{
    Rec_old.x = Rec_new.x;
    Rec_old.y = Rec_new.y;
    Rec_old.width = Rec_new.width;
    Rec_old.height = Rec_new.height;

    double w = rec.width * mScall;
    double h = rec.height * mScall;

    if(w > 0.0 && h > 0.0){
        if(w < mMinSize.width || h < mMinSize.height){
            fx = max(mMinSize.width/w, mMinSize.height/h);
        }else{
            fx = 1.0;
        }
        Rec_new.x = max((int)(rec.x-(w-rec.width)/2.0),0);
        Rec_new.y = max((int)(rec.y-(h-rec.height)/2.0),0);

        Rec_new.width = min((int)w,mImageSize.width-Rec_new.x);
        Rec_new.height = min((int)h,mImageSize.height-Rec_new.y);
    }else{
        Rec_new.x = Rec_new.y = Rec_new.width = Rec_new.height = 0;
    }
}

void ImageSection::setBoxScall(double s)
{
    mScall = s;
}

double ImageSection::getBoxScall()
{
    return mScall;
}

void ImageSection::setBoxMinSize(int w, int h)
{
    mMinSize.width = w;
    mMinSize.height = h;
}

void ImageSection::toImage(LandmarkDetector::CLNF &clnf)
{
    cv::Mat_<double> shape2D = clnf.detected_landmarks;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        if(mAutoSize && fx != 1.0){
            double x = shape2D.at<double>(pos);
            double y = shape2D.at<double>(pos + n);
            shape2D.at<double>(pos) = Rec_new.x + x/fx;
            shape2D.at<double>(pos + n) = Rec_new.y + y/fx;
        }else{
            shape2D.at<double>(pos) += Rec_new.x;
            shape2D.at<double>(pos + n) += Rec_new.y;
        }
    }
    clnf.detected_landmarks = shape2D.clone();

    clnf.params_global[0] = clnf.params_global[0]/fx;
    clnf.params_global[4] = clnf.params_global[4]/fx + Rec_new.x;
    clnf.params_global[5] = clnf.params_global[5]/fx + Rec_new.y;

    for (size_t part = 0; part < clnf.hierarchical_models.size(); ++part){
        cv::Mat_<double> shape2D = clnf.hierarchical_models[part].detected_landmarks;

        int n = shape2D.rows/2;
        for(int pos = 0; pos < n; pos++){
            if(fx != 1.0){
                double x = shape2D.at<double>(pos);
                double y = shape2D.at<double>(pos + n);
                shape2D.at<double>(pos) = Rec_new.x + x/fx;
                shape2D.at<double>(pos + n) = Rec_new.y + y/fx;
            }else{
                shape2D.at<double>(pos) += Rec_new.x;
                shape2D.at<double>(pos + n) += Rec_new.y;
            }
        }

        clnf.hierarchical_models[part].detected_landmarks = shape2D.clone();

        clnf.hierarchical_models[part].params_global[0] = clnf.hierarchical_models[part].params_global[0]/fx;
        clnf.hierarchical_models[part].params_global[4] = clnf.hierarchical_models[part].params_global[4]/fx + Rec_new.x;
        clnf.hierarchical_models[part].params_global[5] = clnf.hierarchical_models[part].params_global[5]/fx + Rec_new.y;
    }
}

void ImageSection::setAutoSize(bool use)
{
    mAutoSize = true;
}

double ImageSection::getImageScall()
{
    return fx;
}

void ImageSection::toSection(LandmarkDetector::CLNF &clnf)
{
    cv::Mat_<double> shape2D = clnf.detected_landmarks;

    int n = shape2D.rows/2;
    for(int pos = 0; pos < n; pos++){
        if(mAutoSize && fx != 1.0){
            double x = shape2D.at<double>(pos);
            double y = shape2D.at<double>(pos + n);
            shape2D.at<double>(pos) = (x - Rec_new.x)*fx;
            shape2D.at<double>(pos + n) = (y - Rec_new.y)*fx;
        }else{
            shape2D.at<double>(pos) -= Rec_new.x;
            shape2D.at<double>(pos + n) -= Rec_new.y;
        }
    }
    clnf.detected_landmarks = shape2D.clone();

    clnf.params_global[0] = clnf.params_global[0]*fx;
    clnf.params_global[4] = (clnf.params_global[4]- Rec_new.x)*fx;
    clnf.params_global[5] = (clnf.params_global[5]- Rec_new.y)*fx;

    for (size_t part = 0; part < clnf.hierarchical_models.size(); ++part){
        cv::Mat_<double> shape2D = clnf.hierarchical_models[part].detected_landmarks;

        int n = shape2D.rows/2;
        for(int pos = 0; pos < n; pos++){
            if(fx != 1.0){
                double x = shape2D.at<double>(pos);
                double y = shape2D.at<double>(pos + n);
                shape2D.at<double>(pos) = (x-Rec_new.x)*fx;
                shape2D.at<double>(pos + n) = (y-Rec_new.y)*fx;
            }else{
                shape2D.at<double>(pos) -= Rec_new.x;
                shape2D.at<double>(pos + n) -= Rec_new.y;
            }
        }

        clnf.hierarchical_models[part].detected_landmarks = shape2D.clone();

        clnf.hierarchical_models[part].params_global[0] = clnf.hierarchical_models[part].params_global[0]*fx;
        clnf.hierarchical_models[part].params_global[4] = (clnf.hierarchical_models[part].params_global[4]- Rec_new.x)*fx;
        clnf.hierarchical_models[part].params_global[5] = (clnf.hierarchical_models[part].params_global[5]- Rec_new.y)*fx;
    }
}
