#include "camera.h"

#include <iostream>
#include <math.h>

#include "model/image.h"

Camera::Camera(int id)
{
    setCameraParameter(id);
    setPath("/home/falko/Uni/Master/Film/Test_Positionen_1.mp4");
    mRotation = (cv::Mat_<double>(3,3) << 0.992766452308293, 0.06368711653559152, -0.1017778087727747,
                 -0.08705908421335735, -0.2018935270331317, -0.9755304811219656,
                 -0.08267700422207537, 0.9773346176870561, -0.1948885785349126);
}

Camera::~Camera()
{

}

bool Camera::setPath(QString path){
    video.open(path.toStdString());
    if(video.isOpened()){
        return true;
    }else{
        qDebug()<< "Fehler beim Video öffen: "<<path;
        return false;
    }
}


void Camera::setCameraParameter(int id){
    ID = id;
    if(id == 1){ //Webcam (fx und fy besser bei OpenFace wenn halb so groß)
        ImageWight = 640; ImageHeight = 480;
        cameraMatrix = (cv::Mat_<double>(3,3) << 1543.184291356096, 0, 350.9031480800631,
                        0, 1526.994309676135, 303.4424747270602,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) <<-0.5957204276742056, 19.80483478953746, -0.02046345685904995, 0.01027510169914053, -159.5254263670344);
    }else if(id == 2){ //1280P der 4k Actioncam (als Webcam)
        ImageWight = 1280; ImageHeight = 720;
        cameraMatrix = (cv::Mat_<double>(3,3) << 4505.771917224674, 0, 627.2704519691812,
                        0, 2986.23823820304, 365.9469872012109,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << -5.683648805753482, 69.69903660169872, -0.1033539021069702, -0.0165845448486779, -487.6393497545911);
    }else if(id == 3){ // 1940P der 4K Actioncam (1080P einstellung)
        ImageWight = 1920; ImageHeight = 1080;
        cameraMatrix = (cv::Mat_<double>(3,3) << 13343.09623288915, 0, 966.1848227467876,
                        0, 7594.338338846001, 535.2483346076116,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -9.599320311558495, 153.5177497796487, -0.02105648553599707, -0.03765560152948207, 4.241928717530834);
    }else if(id == 4){ //2688P der 4k Actioncam (2.7K Einstellung)
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 15373.97717428267, 0, 1321.996093444815,
                        0, 17764.65120151666, 735.3988503456478,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -18.60440739952032, 523.0811532248169, 0.02032995403105285, -0.04626945929212306, 8.68919963553518);
    }else if (id == 5){ //2688P der 4k Actioncam (2.7K Einstellung in Box)
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 6245.985171734248, 0, 1340.851224182062,
                        0, 6593.452572771233, 755.2799615988556,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.194666252362591, -10.98776274024639, 0.01492657086110723, -0.04010910007443097, 176.6908906375982);
    }else if (id == 6){ //2688P der 4k Actioncam (2.7K Einstellung in Box, Neu)
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 5906.900190890472, 0, 1350.264438984915,
                        0, 5979.120910258862, 763.7137371523507,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.97804942570734, -2.119731335786197, -0.05503551191292154, -0.0192320523439882, 43.87783092626537);
    }else if (id == 6){ //2688P der 4k Actioncam (2.7K Einstellung in Box, Neu 2)
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 6900.555681192552, 0, 1337.61573087611,
                        0, 7444.153586551173, 749.1908462284828,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.917718077998721, -20.47575286011954, -0.03300201474788447, 0.00172225776199838, 236.2651786331631);
    }else if(id == 9){ // 3840P der 4K Actioncam
        ImageWight = 3840; ImageHeight = 2160;
        cameraMatrix = (cv::Mat_<double>(3,3) << 7409.28638524711, 0, 1868.435847081091,
                        0, 7512.705802013185, 977.0636190423108,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -2.006696653082546, 14.50478814130672, 0.01196899857324854, -0.0326620616728269, -56.30904541044546);
    }else{//Default Parameter
        ImageWight = 640; ImageHeight = 480;
        float fx = 500 * (ImageWight / 640.0);
        float fy = 500 * (ImageHeight / 480.0);

        fx = (fx + fy) / 2.0;
        cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0, ImageWight/2.0,
                        0, fx, ImageHeight/2.0,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    }
    cv::Size imageSize(ImageWight,ImageHeight);
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                                CV_16SC2, map1, map2);
}

void Camera::correct_Image(cv::Mat img){
        cv::undistort(img.clone(),img, cameraMatrix, distCoeffs);//Korrektur mit beschneiden
        //cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung
}

void Camera::setUseCorrection(bool c)
{
    mUseCorrection = c;
}

void Camera::setFrame(size_t frame)
{
    video.set(CV_CAP_PROP_POS_FRAMES,(double)frame-1);
}

void Camera::setImageSize(int Wight, int Height)
{
    bool calc = false;
    if(ImageWight != Wight){
        ImageWight = Wight;
        calc = true;
    }
    if(ImageHeight != Height){
        ImageHeight = Height;
        calc = true;
    }
    if(calc){
        float fx = 500 * (ImageWight / 640.0);
        float fy = 500 * (ImageHeight / 480.0);

        fx = (fx + fy) / 2.0;
        cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0, ImageWight/2.0,
                        0, fx, ImageHeight/2.0,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    }
}

cv::Mat Camera::rotateWorldToCamera(cv::Mat in)
{
    return mRotation.mul(in);
}

cv::Mat Camera::rotateCameraToWorld(cv::Mat in)
{
    return mRotation.t().mul(in);
}

size_t Camera::getFrameNr()
{
    return (size_t)video.get(CV_CAP_PROP_POS_FRAMES);
}

// Gibt die Werte der Kamera aus
void Camera::get_camera_params(double &fx, double &fy, double &cx, double &cy, int &x, int &y){
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);
    x = ImageWight;
    y = ImageHeight;
}

int Camera::getCameraID()
{
    return ID;
}

bool Camera::getFrame(cv::Mat &img)
{
    if(video.isOpened()){
        bool ret = video.read(img);
        if(ret && mUseCorrection){
            correct_Image(img);
        }
        return ret;
    }else{
        return false;
    }
}

bool Camera::getFrame(cv::Mat &img, size_t pos)
{
    video.set(CV_CAP_PROP_POS_FRAMES,(double)pos-1);
    return getFrame(img);
}
