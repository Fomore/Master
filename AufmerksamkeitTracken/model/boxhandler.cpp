#include "boxhandler.h"

BoxHandler::BoxHandler(int width, int height)
{
    mImageSize.width = width;
    mImageSize.height = height;
    Rec_new.x = Rec_new.y = Rec_new.width = Rec_new.height = 0;
    Rec_old.x = Rec_old.y = Rec_old.width = Rec_old.height = 0.0;
    mMinSize.width = 200;
    mMinSize.height = 220;
}


BoxHandler::~BoxHandler()
{

}

void BoxHandler::getSection(int &x, int &y, int &w, int &h){
    x = Rec_new.x;
    y = Rec_new.y;
    w = Rec_new.width;
    h = Rec_new.height;
}

void BoxHandler::getSection(cv::Rect &rect)
{
    getSection(rect.x, rect.y, rect.width, rect.height);
}

cv::Rect BoxHandler::getRect()
{
    return Rec_new;
}

void BoxHandler::getImage(cv::Mat Image,cv::Mat &Part)
{
    if(fx > 1.0){
        cv::resize(Image(Rec_new),Part,cv::Size(0,0),fx,fx);
    }else{
        Part = Image(Rec_new);
    }
}

void BoxHandler::setNewRect(cv::Rect rec)
{
    double w = std::max(rec.width,10) * mScall * 1.2;
    double h = std::max(rec.height,12) * mScall;

    if(w > 0.0 && h > 0.0){
        if(rec.width < mMinSize.width || rec.height < mMinSize.height){
            fx = max(mMinSize.width/rec.width, mMinSize.height/rec.height);
        }else{
            fx = 1.0;
        }
        double px_min = rec.x-(w-rec.width)/2.0;
        double py_min = rec.y-(h-rec.height)/2.0;
        double px_max = px_min+w;
        double py_max = py_min+h;

        px_min = min(px_min, Rec_old.x);
        py_min = min(py_min, Rec_old.y);
        px_max = max(px_max, Rec_old.x+Rec_old.width);
        py_max = max(py_max, Rec_old.y+Rec_old.height);

        Rec_new.x = max((int)px_min,0);
        Rec_new.y = max((int)py_min,0);

        Rec_new.width = min((int)(px_max-Rec_new.x),mImageSize.width-Rec_new.x);
        Rec_new.height = min((int)(py_max-Rec_new.y),mImageSize.height-Rec_new.y);
    }else{
        Rec_new.x = Rec_new.y = 0;
        Rec_new.width = mImageSize.width;
        Rec_new.height = mImageSize.height;
    }
}

void BoxHandler::setOldRect(cv::Rect2d rec)
{
    Rec_old.x = rec.x;
    Rec_old.y = rec.y;
    Rec_old.width = rec.width;
    Rec_old.height= rec.height;
}

void BoxHandler::setBoxScall(double s)
{
    mScall = s;
}

double BoxHandler::getBoxScall()
{
    return mScall;
}

void BoxHandler::setBoxMinSize(int w, int h)
{
    mMinSize.width = w;
    mMinSize.height = h;
}

void BoxHandler::setImageSize(int w, int h)
{
    mImageSize.width = w;
    mImageSize.height = h;
}

void BoxHandler::toImage(LandmarkDetector::CLNF &clnf)
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

void BoxHandler::setAutoSize(bool use)
{
    mAutoSize = true;
}

double BoxHandler::getImageScall()
{
    return fx;
}

void BoxHandler::toSection(LandmarkDetector::CLNF &clnf)
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
