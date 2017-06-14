#include "camera.h"

#include <iostream>
#include <math.h>

#include "model/image.h"

Camera::Camera(int id)
{
    setCameraParameter(id);
    setPath("/home/falko/Uni/Master/Film/Test_Positionen_1.mp4");
    /*
    //Innenaufnahme
    mRotMatrix = cv::Matx33d(0.9936581272313783, 0.03038307716496669, -0.1082607722539323,
                             -0.06568480537591176, 0.9382912981081369, -0.3395510951790254,
                             0.09126353340605708, 0.3445087930902981, 0.9343364805859329);
    mTranslation = cv::Vec3d(0, 206, -31);
    //fx=1130.69 fy=1124.03
    */

    //Video Messung
    mRotMatrix = cv::Matx33d(0.9998900056918177, -0.01474785878659751, 0.001573905596829441,
                             0.01473015278795625, 0.9998339988008559, 0.01072368596769573,
                             -0.001731795732936419, -0.01069932255336143, 0.9999412609650821);

    mTranslation = cv::Vec3d(1340,745+240, 125);
   //fx=1185.06 fy=1201.78

    /*
    // Außenaufnahme
    mRotMatrix = cv::Matx33d(0.9991731060535562, 0.01466352322878778, -0.03792209416432796,
                             -0.02132243566782049, 0.9831316519883933, -0.1816521637519485,
                             0.03461875035989887, 0.1823105480905914, 0.9826313684075159);
    mTranslation = cv::Vec3d(0, 148+40, 0);
    //fx=1046.86 fy=974.971
    */

    //correctTest(cv::Scalar(255, 0, 0,255),"A");
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
        mfx = mfy = 500;
        ImageWight = 640; ImageHeight = 480;
        cameraMatrix = (cv::Mat_<double>(3,3) << 1543.184291356096, 0, 350.9031480800631,
                        0, 1526.994309676135, 303.4424747270602,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) <<-0.5957204276742056, 19.80483478953746, -0.02046345685904995, 0.01027510169914053, -159.5254263670344);
    }else if(id == 2){ //1280P der 4k Actioncam (als Webcam)
        mfx = mfy = 740;
        ImageWight = 1280; ImageHeight = 720;
        cameraMatrix = (cv::Mat_<double>(3,3) << 4505.771917224674, 0, 627.2704519691812,
                        0, 2986.23823820304, 365.9469872012109,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << -5.683648805753482, 69.69903660169872, -0.1033539021069702, -0.0165845448486779, -487.6393497545911);
    }else if(id == 3){ // 1940P der 4K Actioncam (1080P einstellung)
        mfx = mfy = 1110;
        ImageWight = 1920; ImageHeight = 1080;
        cameraMatrix = (cv::Mat_<double>(3,3) << 13343.09623288915, 0, 966.1848227467876,
                        0, 7594.338338846001, 535.2483346076116,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -9.599320311558495, 153.5177497796487, -0.02105648553599707, -0.03765560152948207, 4.241928717530834);
    }else if(id == 4){ //2688P der 4k Actioncam (2.7K Einstellung)
        mfx = mfy = 1435.78;
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 15373.97717428267, 0, 1321.996093444815,
                        0, 17764.65120151666, 735.3988503456478,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -18.60440739952032, 523.0811532248169, 0.02032995403105285, -0.04626945929212306, 8.68919963553518);
    }else if (id == 5){ //2688P der 4k Actioncam (2.7K Einstellung in Box)
        mfx = mfy = 1428.86;
        ImageWight = 2688; ImageHeight = 1520;
        cameraMatrix = (cv::Mat_<double>(3,3) << 6900.555681192552, 0, 1337.61573087611,
                        0, 7444.153586551173, 749.1908462284828,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -1.917718077998721, -20.47575286011954, -0.03300201474788447, 0.00172225776199838, 236.2651786331631);
    }else if(id == 7){ // 3840P der 4K Actioncam
        mfx = mfy = 2220;
        ImageWight = 3840; ImageHeight = 2160;
        cameraMatrix = (cv::Mat_<double>(3,3) << 7409.28638524711, 0, 1868.435847081091,
                        0, 7512.705802013185, 977.0636190423108,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << -2.006696653082546, 14.50478814130672, 0.01196899857324854, -0.0326620616728269, -56.30904541044546);
    }else if(id == 8){ // Logitech Webcam
        mfx = mfy = 1200;
        ImageWight = 1600; ImageHeight = 896;
        cameraMatrix = (cv::Mat_<double>(3,3) << 1013.2644029303632, 0, 832.62006512079449,
                        0, 1011.9932034499595, 412.54861968002001,
                        0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1,5) << 0.017420348798968862, -0.11512510437406340,
                      -0.0076981462879360234, -0.0012927278894819274, -0.0079674065513256048);
    }else{//Default Parameter
        ImageWight = 640; ImageHeight = 480;
        mfx = 500 * (ImageWight / 640.0);
        mfy = 500 * (ImageHeight / 480.0);

        double fx = (mfx + mfy) / 2.0;
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

double Camera::getTimeSec()
{
    return video.get(CV_CAP_PROP_POS_MSEC)/1000.0;
}

bool Camera::UseCorrection()
{
    return mUseCorrection;
}

void Camera::correct_Image(cv::Mat img){
        cv::undistort(img.clone(),img, cameraMatrix, distCoeffs);//Korrektur mit beschneiden
        //cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);//Korrektur mit skallierung
}

cv::Rect Camera::correct_Rect(cv::Rect rec)
{
    if(mUseCorrection){
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
    }else{
        return rec;
    }
}

void Camera::correctTest(cv::Scalar col, std::string name)
{
    cv::Mat Eingabe = cv::imread("/home/falko/Bilder/Bildschirmfoto von Test_Positionen_1.mp4.png", -1);

    correct_Image(Eingabe);

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("Kalibrierung_"+name+".png",Eingabe,compression_params);
    /*
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

    cv::imwrite("Test_Img_"+name+".png",Testbild,compression_params);
    */
}

void Camera::setUseCorrection(bool c)
{
    mUseCorrection = c;
}

void Camera::setFrame(size_t frame)
{
    video.set(CV_CAP_PROP_POS_FRAMES,(double)frame-1);
}
void Camera::setFrame(double frame)
{
    video.set(CV_CAP_PROP_POS_FRAMES,frame);
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
        mfx = 500 * (ImageWight / 640.0);
        mfy = 500 * (ImageHeight / 480.0);
        double fx = (mfx + mfy) / 2.0;
        cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0, ImageWight/2.0,
                        0, fx, ImageHeight/2.0,
                        0, 0, 1 );
        distCoeffs = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0);
    }
}

cv::Vec3d Camera::rotateToWorld(cv::Point3f in)
{
    return rotateToWorld(cv::Vec3d(in.x, in.y, in.z));
}

cv::Vec3d Camera::rotateToWorld(cv::Vec3d in)
{
    return mRotMatrix.t() * in + mTranslation;
}

cv::Vec3d Camera::rotateToCamera(cv::Point3d in)
{
    return rotateToCamera(cv::Vec3d(in.x, in.y, in.z));
}

cv::Vec3d Camera::rotateToCamera(cv::Vec3d in)
{
    return mRotMatrix * (in - mTranslation);
}

cv::Matx33d Camera::getRotationMatrix()
{
    return mRotMatrix;
}

size_t Camera::getFrameNr()
{
    return (size_t)video.get(CV_CAP_PROP_POS_FRAMES);
}

void Camera::setFxFy(double fx, double fy)
{
    if(fx > 0){
        mfx = fx;
    }else{
        mfx = 500 * (ImageWight / 640.0);
    }
    if(fy > 0){
        mfy = fy;
    }else{
        mfy = 500 * (ImageHeight / 480.0);
    }
}

double Camera::getFx()
{
    return mfx;
}

double Camera::getFy()
{
    return mfy;
}

// Gibt die Werte der Kamera aus
void Camera::get_camera_params(double &fx, double &fy, double &cx, double &cy, int &x, int &y){
    fx = mfx;
    fy = mfy;
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

cv::Vec3d Camera::getTranslation()
{
    return mTranslation;
}
