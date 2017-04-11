#include "camera.h"

#include <iostream>
#include <math.h>

#include "model/image.h"

Camera::Camera(int id)
{
    setCameraParameter(id);
    setPath("/home/falko/Uni/Master/Film/Test_Positionen_1.mp4");
    //mRotation = cv::Vec3d(1.852973068655717-M_PI/2, -0.04104046258180141, -0.1144534716463462);
    mRotation = cv::Vec3d(1.852973068655717, -0.04104046258180141, -0.1144534716463462);
    //cv::Rodrigues(mRotation,mRotMatrix);

    //mTranslation = cv::Vec3d(0, 206, 31);
    //mTranslation = cv::Vec3d(0, 148+40, 0);
    //mTranslation = cv::Vec3d(23.41559466243473, 239.1545806718657, 69.81405332352804);

    mRotMatrix = cv::Matx33d(0.994502496350403, 0.03084993904995423, -0.1000653087409978,
                             -0.08740706975903308, -0.2816457732931845, -0.9555289961807667,
                             -0.0576609825528202, 0.9590223874585506, -0.2774009218520103);
    mTranslation = cv::Vec3d(23.41559466243473, 239.1545806718657, 69.81405332352804);

    std::cout<<"Null"<<std::endl;
    correctTest(cv::Scalar(255, 0, 0,255),"A");

    mRotMatrix = cv::Matx33d(0.994502496350403, 0.1000653087409978, 0.03084993904995423,
                             -0.08740706975903308, 0.9555289961807667, -0.2816457732931845,
                             -0.0576609825528202, 0.2774009218520103, 0.9590223874585506);
    mTranslation = cv::Vec3d(0, 206, 31);
    std::cout<<"Neu"<<std::endl;
    correctTest(cv::Scalar(0, 255, 255,255),"B");
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

cv::Rect Camera::correct_Rect(cv::Rect rec)
{
    std::vector<cv::Point2d> PointTestIn;
    PointTestIn.push_back(cv::Point2d(rec.x,rec.y));
    PointTestIn.push_back(cv::Point2d(rec.x+rec.width,rec.y+rec.height));
    std::vector<cv::Point2d> PointTestOut;

    cv::undistortPoints(PointTestIn,PointTestOut,cameraMatrix,distCoeffs,cv::noArray(), cameraMatrix);

    cv::Rect ret;
    ret.x = PointTestOut[0].x+0.5;
    ret.y = PointTestOut[0].y+0.5;
    ret.width = PointTestOut[1].x-PointTestOut[0].x+0.5;
    ret.height= PointTestOut[1].y-PointTestOut[0].y+0.5;

    return ret;
}

void Camera::correctTest(cv::Scalar col, std::string name)
{
    std::vector<cv::Point3d> PointTestIn;

    PointTestIn.push_back(cv::Point3d(-300,0,100));
    PointTestIn.push_back(cv::Point3d( 300,0,100));
    PointTestIn.push_back(cv::Point3d(-300,0,1000));
    PointTestIn.push_back(cv::Point3d( 300,0,1000));

    PointTestIn.push_back(cv::Point3d( 0,0,500));
    PointTestIn.push_back(cv::Point3d( 0,170,500));

    for(size_t i = 0; i < PointTestIn.size(); i++){
        std::cout<<PointTestIn[i]<<rotateToCamera(PointTestIn[i])<<std::endl;
    }

    PointTestIn.clear();
    PointTestIn.push_back(cv::Point3d(-300,0,100));
    PointTestIn.push_back(cv::Point3d(-200,0,100));
    PointTestIn.push_back(cv::Point3d(-100,0,100));
    PointTestIn.push_back(cv::Point3d(   0,0,100));
    PointTestIn.push_back(cv::Point3d( 100,0,100));
    PointTestIn.push_back(cv::Point3d( 200,0,100));
    PointTestIn.push_back(cv::Point3d( 300,0,100));

    PointTestIn.push_back(cv::Point3d(-300,0,400));
    PointTestIn.push_back(cv::Point3d(-200,0,400));
    PointTestIn.push_back(cv::Point3d(-100,0,400));
    PointTestIn.push_back(cv::Point3d(   0,0,400));
    PointTestIn.push_back(cv::Point3d( 100,0,400));
    PointTestIn.push_back(cv::Point3d( 200,0,400));
    PointTestIn.push_back(cv::Point3d( 300,0,400));

    PointTestIn.push_back(cv::Point3d(-300,0,800));
    PointTestIn.push_back(cv::Point3d(-200,0,800));
    PointTestIn.push_back(cv::Point3d(-100,0,800));
    PointTestIn.push_back(cv::Point3d(   0,0,800));
    PointTestIn.push_back(cv::Point3d( 100,0,800));
    PointTestIn.push_back(cv::Point3d( 200,0,800));
    PointTestIn.push_back(cv::Point3d( 300,0,800));

    PointTestIn.push_back(cv::Point3d(-300,0,1000));
    PointTestIn.push_back(cv::Point3d(-200,0,1000));
    PointTestIn.push_back(cv::Point3d(-100,0,1000));
    PointTestIn.push_back(cv::Point3d(   0,0,1000));
    PointTestIn.push_back(cv::Point3d( 100,0,1000));
    PointTestIn.push_back(cv::Point3d( 200,0,1000));
    PointTestIn.push_back(cv::Point3d( 300,0,1000));

    PointTestIn.push_back(cv::Point3d( 0,170,500));

    cv::Mat Testbild = cv::imread("/home/falko/Bilder/Bildschirmfoto von Test_Positionen_1.mp4.png", -1);
    for(size_t i = 0; i < PointTestIn.size(); i++){
        cv::Vec3d pos = rotateToCamera(PointTestIn[i]);
        cv::Point2d pnt(1380.443342245189*pos[0]/pos[2]+1134.2184893459,
                        1173.443931755887*pos[1]/pos[2]+596.7089856685562);
        cv::circle(Testbild,pnt,4,col);
    }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("Test_Img_"+name+".png",Testbild,compression_params);
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

cv::Matx33d Camera::rotateWorldToCamera(cv::Matx33d in)
{
    return mRotMatrix * in;
}

cv::Matx33d Camera::rotateCameraToWorld(cv::Matx33d in)
{
    return mRotMatrix.t() * in;
}

cv::Vec3d Camera::rotateToWorld(cv::Point3f in)
{
    return rotateToWorld(cv::Vec3d(in.x, in.y, in.z));
}

cv::Vec3d Camera::rotateToWorld(cv::Vec3d in)
{
    return mRotMatrix.t() * in;
}

cv::Vec3d Camera::rotateToCamera(cv::Vec3d in)
{
    return mRotMatrix * cv::Vec3d(in[0],in[2],in[1]) + mTranslation;
}

cv::Vec3d Camera::getRotation()
{
    return mRotation;
}

cv::Matx33d Camera::getRotationMatrix()
{
    return mRotMatrix;
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
