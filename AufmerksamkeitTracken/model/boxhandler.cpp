#include "boxhandler.h"

BoxHandler::BoxHandler(int width, int height)
{
    mImageSize.width = width;
    mImageSize.height = height;
    Rec_new.x = Rec_new.y = Rec_new.width = Rec_new.height = 0;
    Rec_old.x = Rec_old.y = Rec_old.width = Rec_old.height = 0.0;
    mMinSize.width = 150;
    mMinSize.height = 170;
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
    if(rec.width > 0.0 && rec.height > 0.0){
        double w = rec.width * mScall * 1.2;
        double h = rec.height* mScall;

        if(rec.width < mMinSize.width || rec.height < mMinSize.height){
            fx = max(mMinSize.width/rec.width, mMinSize.height/rec.height);
        }else{
            fx = 1.0;
        }
        Rec_new.x = max((int)(rec.x-(w-rec.width)/2.0),0);
        Rec_new.y = max((int)(rec.y-(h-rec.height)/2.0),0);

        Rec_new.width = min((int)w,mImageSize.width-Rec_new.x);
        Rec_new.height = min((int)h,mImageSize.height-Rec_new.y);

        if(Rec_old.width > 0 && Rec_old.height > 0){
            double px_min = max(0.0,min((double)Rec_new.x,Rec_old.x));
            double py_min = max(0.0,min((double)Rec_new.y,Rec_old.y));

            double px_max = min((double)mImageSize.width ,max((double)Rec_new.x+Rec_new.width ,Rec_old.x+Rec_old.width));
            double py_max = min((double)mImageSize.height,max((double)Rec_new.y+Rec_new.height,Rec_old.y+Rec_old.height));

            px_min = min(px_min, Rec_old.x);
            py_min = min(py_min, Rec_old.y);
            px_max = max(px_max, Rec_old.x+Rec_old.width);
            py_max = max(py_max, Rec_old.y+Rec_old.height);

            Rec_new.x = px_min;
            Rec_new.y = py_min;
            Rec_new.width = px_max-px_min+1.0;
            Rec_new.height= py_max-py_min+1.0;
        }
    }else{
        Rec_new.x = Rec_new.y = Rec_new.width = Rec_new.height = 0;
    }
}

void BoxHandler::setOldRect(cv::Rect2d rec)
{
    setOldRect(rec.x,rec.y,rec.width,rec.height);
}

void BoxHandler::setOldRect(double X, double Y, double W, double H)
{
    Rec_old.x = X;
    Rec_old.y = Y;
    Rec_old.width = W;
    Rec_old.height= H;
}

bool BoxHandler::existBox()
{
    return Rec_new.width > 0.0 && Rec_new.height > 0.0;
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
