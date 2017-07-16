#include <QCoreApplication>

#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include <QString>
#include <QStringList>
#include <QXmlStreamReader>
#include <QFile>
#include <QRegExp>
#include <QTextStream>

#include <boost/algorithm/string.hpp>


bool myfunction (double i,double j) { return (i<j); }

void CalcAbstand(std::vector<cv::Vec6d> &As, std::vector<cv::Vec6d> &Bs,const int pos, cv::Vec3d &Abweichung, cv::Vec3d &Werte){
    if(As.size() != Bs.size() || As.size() == 0){
        Abweichung[0] = Abweichung[1] = Abweichung[2] = 0.0;
        Werte[0] = Werte[1] = Werte[2] = 0.0;
        return;
    }
    std::vector<double> abstand;
    Abweichung[0] = 0.0;
    Abweichung[1] = 0.0;
    Abweichung[2] = 0.0;
    Werte[0] = As[0][pos];
    Werte[1] = 0;
    Werte[2] = As[0][pos];
    for(size_t face = 0; face < As.size(); face++){
        Werte[0] = std::min(As[face][pos],Werte[0]);
        Werte[1] += As[face][pos];
        Werte[2] = std::max(As[face][pos],Werte[2]);

        double diff = std::abs(As[face][pos]-Bs[face][pos]);
        abstand.push_back(diff);
        Abweichung[1] += diff;
    }
    std::sort (abstand.begin(), abstand.end(),myfunction);
    Abweichung[0] = abstand[As.size()/4];
    Abweichung[1] = Abweichung[1]/As.size();
    Abweichung[2] = abstand[3*As.size()/4];

    Werte[1] = Werte[1]/As.size();
}

void CalcValue(std::vector<cv::Vec6d> &Correct, std::vector<cv::Vec6d> &Calcualte, double Skalierung, std::string name){
    std::ofstream myfile;
    myfile.open("Solution_resize_"+name+".txt", std::ios::in | std::ios::app);
    myfile<<Skalierung<<" ";
    for(int i = 0; i < 6; i++){
        cv::Vec3d abw;
        cv::Vec3d val;
        CalcAbstand(Correct,Calcualte,i,abw,val);
        myfile<<abw[0]<<" "<<abw[1]<<" "<<abw[2]<<" "<<val[0]<<" "<<val[1]<<" "<<val[2]<<" ";
    }
    myfile<<std::endl;
    myfile.close();
}

void FileToXML(std::string FileName, QString Name){
    /*
    1050
    [1242, 908, 1476] 3
    [2680, 1490, 0] 3
    [0.886307, -0.0287634, -0.599153, -0.0551769, 794.244, 335.67] 6
    [-43.2988, -86.7402, 1353.93, 0.0120588, -0.629671, -0.0725003] 6
    [0.202427, 0.144975, -0.968507] 3
    [0.0824298, 0.127675, -0.988385] 3
    [703.095, 280.301, 122.956, 120.604] Box
    0.457387
    0.115279
    0.533845
    */
    QXmlStreamWriter xmlWriter;

    QFile XMLfile(Name+".xml");
    XMLfile.open(QFile::WriteOnly);

    xmlWriter.setDevice(&XMLfile);
    xmlWriter.setAutoFormatting(true);

    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("dataset");

    xmlWriter.writeStartElement("images");

    std::ifstream file(FileName);
    std::string line;
    QRegExp ausdruck("[\\[\\]\\,\\|]");
    if(file.is_open()){
        while (std::getline(file, line)){
            std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
            std::string tmp;
            int FrameNr;
            iss >> FrameNr;
            for(size_t i = 0; i < 3+3+6+6+3+3; i++){
                iss >> tmp;
            }
            cv::Rect Box;
            double x,y,w,h;
            iss >> x, iss >> y, iss >>w, iss>>h;
            Box.x = x;
            Box.y = y;
            Box.width = (w+0.5)+1;
            Box.height= (h+0.5)+1;

            xmlWriter.writeStartElement("image");
            xmlWriter.writeAttribute("file","Probant0-"+QString("%1").arg(FrameNr, 6, 10, QChar('0'))+".jpg");

            xmlWriter.writeStartElement("box");
            xmlWriter.writeAttribute("left",QString::number(Box.x));
            xmlWriter.writeAttribute("top",QString::number(Box.y));
            xmlWriter.writeAttribute("width",QString::number(Box.width));
            xmlWriter.writeAttribute("height",QString::number(Box.height));
            xmlWriter.writeEndElement();

            xmlWriter.writeEndElement();
        }
        xmlWriter.writeEndElement();
    }

    xmlWriter.writeEndElement();
    xmlWriter.writeEndDocument();

}

void fixXML(QString Path){

    QXmlStreamReader xmlRead;
    QXmlStreamWriter xmlWriter;

    QFile xmlFileR(Path);
    QFile xmlFileW("N"+Path);

    if(xmlFileR.open(QIODevice::ReadOnly) && xmlFileW.open(QFile::WriteOnly)){
        xmlRead.setDevice(&xmlFileR);
        xmlWriter.setDevice(&xmlFileW);
        xmlWriter.setAutoFormatting(true);
        xmlWriter.writeStartDocument();
        if (!xmlRead.hasError() && xmlRead.readNextStartElement() && xmlRead.name() == "dataset"){
            xmlWriter.writeStartElement("dataset");
            while (xmlRead.readNextStartElement()) {
                if (xmlRead.name() == "images"){
                    xmlWriter.writeStartElement("images");
                    int LastFrame = -1, LastHeight = -1, LastLeft = -1, LastTop = -1, LastWidth = -1;
                    while (xmlRead.readNextStartElement()) {
                        if (xmlRead.name() == "image"){
                            QString file = xmlRead.attributes().value("file").toString();
                            int frame = -1;
                            if(!file.isNull()){
                                QStringList myStringList = file.split('-').last().split('.');
                                if(myStringList.size() >=2){
                                    frame = myStringList[myStringList.size()-2].toInt();
                                }else{
                                    frame = -1;
                                }
                            }
                            int height, left, top, width;
                            while (xmlRead.readNextStartElement()) {
                                if (xmlRead.name() == "box"){
                                    QXmlStreamAttributes att = xmlRead.attributes();
                                    height = att.value("height").toInt();
                                    left = att.value("left").toInt();
                                    top = att.value("top").toInt();
                                    width = att.value("width").toInt();
                                }
                                xmlRead.skipCurrentElement();
                            }
                            if(frame <= LastFrame){
                                std::cout<<"Fehler: "<<LastFrame<<" "<<frame<<std::endl;
                            }
                            if(LastFrame != -1 && frame-1 != LastFrame){
                                std::cout<<"Aufruf: "<<LastFrame<<" ["<<LastLeft<<", "<<LastTop<<", "<<LastWidth<<", "<<LastHeight<<"] ->"
                                           <<frame<<" ["<<left<<", "<<top<<", "<<width<<", "<<height<<std::endl;

                                double step = frame - LastFrame;
                                double h = (height-LastHeight)/step;
                                double l = (left-LastLeft)/step;
                                double w = (width-LastWidth)/step;
                                double t = (top-LastTop)/step;
                                for(double i = 1.0; i < step; i++){
                                    xmlWriter.writeStartElement("image");
                                    int frm = frame+i;
                                    xmlWriter.writeAttribute("file","Probant0-"+QString("%1").arg(frm, 6, 10, QChar('0'))+".jpg");

                                    xmlWriter.writeStartElement("box");
                                    int lft = left+l*i+0.5;
                                    xmlWriter.writeAttribute("left",QString::number(lft));
                                    int tp = top+t*i+0.5;
                                    xmlWriter.writeAttribute("top",QString::number(tp));
                                    int wdt = width+w*i+0.5;
                                    xmlWriter.writeAttribute("width",QString::number(wdt));
                                    int hgt = height+h*i+0.5;
                                    xmlWriter.writeAttribute("height",QString::number(hgt));
                                    xmlWriter.writeEndElement();

                                    xmlWriter.writeEndElement();
                                }
                            }
                            xmlWriter.writeStartElement("image");
                            xmlWriter.writeAttribute("file","Probant0-"+QString("%1").arg(frame, 6, 10, QChar('0'))+".jpg");

                            xmlWriter.writeStartElement("box");
                            xmlWriter.writeAttribute("left",QString::number(left));
                            xmlWriter.writeAttribute("top",QString::number(top));
                            xmlWriter.writeAttribute("width",QString::number(width));
                            xmlWriter.writeAttribute("height",QString::number(height));
                            xmlWriter.writeEndElement();

                            xmlWriter.writeEndElement();

                            LastFrame = frame;
                            LastHeight = height;
                            LastLeft = left;
                            LastTop = top;
                            LastWidth = width;
                        }
                    }
                    xmlWriter.writeEndElement();
                }
                xmlRead.skipCurrentElement();
            }
            xmlWriter.writeEndElement();
        }
     }
}

void calcHeadPose(std::string path){
    /*
    Berechnung auf 1 0,01
    [-0,132771; 0,0574386; -0,0273429]
    [-681,645; -451,636; 835,856; -0,0920862; 0,0700892; -0,0240977]
    [-681,645; -451,636; 835,856; 0,352608; -0,520017; 0,182353]
    [-815,035; -539,909; 1000,62; -0,122679; 0,105487; -0,120964]
    [-815,035; -539,909; 1000,62; 0,288935; -0,496432; -0,0192208]

    myfilePose<<getCorrectMatrix(mImagePaths.at(BoxesID[i+j]))
              <<LandmarkDetector::GetPoseCamera(clnf_models[j],fx,fy,cx,cy)
              <<LandmarkDetector::GetPoseWorld(clnf_models[j],fx,fy,cx,cy)
              <<LandmarkDetector::GetCorrectedPoseCamera(clnf_models[j],fx,fy,cx,cy)
              <<LandmarkDetector::GetCorrectedPoseWorld(clnf_models[j],fx,fy,cx,cy)
                */
    std::vector<cv::Vec6d> Correct;
    std::vector<cv::Vec6d> PoseCamera;
    std::vector<cv::Vec6d> PoseWorld;
    std::vector<cv::Vec6d> CorrectedPoseCamera;
    std::vector<cv::Vec6d> CorrectedPoseWorld;

    std::ifstream file(path);
    std::string line;

    double Skalierung = -1;
    while (std::getline(file, line)){
        if(line.compare(0,14,"Berechnung auf") == 0){
            std::cout<<"Berechne "<<Skalierung<<std::endl;
            CalcValue(Correct, PoseCamera, Skalierung, "PoseCamera");
            CalcValue(Correct, PoseWorld, Skalierung, "PoseWorld");
            CalcValue(Correct, CorrectedPoseCamera, Skalierung, "CorrectedPoseCamera");
            CalcValue(Correct, CorrectedPoseWorld, Skalierung, "CorrectedPoseWorld");

            Skalierung = std::stod(line.substr(15,4));

            Correct.clear();
            PoseCamera.clear();
            PoseWorld.clear();
            CorrectedPoseCamera.clear();
            CorrectedPoseWorld.clear();
        }else if(line.compare(0,8,"CV_INTER") == 0){
            std::ofstream myfile;
            myfile.open("Solution_resize_PoseCamera.txt", std::ios::in | std::ios::app);
            myfile<<line<<std::endl;
            myfile.close();

            myfile.open("Solution_resize_PoseWorld.txt", std::ios::in | std::ios::app);
            myfile<<line<<std::endl;
            myfile.close();

            myfile.open("Solution_resize_CorrectedPoseCamera.txt", std::ios::in | std::ios::app);
            myfile<<line<<std::endl;
            myfile.close();

            myfile.open("Solution_resize_CorrectedPoseWorld.txt", std::ios::in | std::ios::app);
            myfile<<line<<std::endl;
            myfile.close();
        }else{
            boost::erase_all(line, "[");
            boost::erase_all(line, ",");
            boost::replace_all(line, "]"," ");

            std::istringstream iss(line);
            double a,b,c,d,e,f;
            iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
            Correct.push_back(cv::Vec6d(a,b,c,d,e,f));

            iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
            PoseCamera.push_back(cv::Vec6d(a,b,c,d,e,f));

            iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
            PoseWorld.push_back(cv::Vec6d(a,b,c,d,e,f));

            iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
            CorrectedPoseCamera.push_back(cv::Vec6d(a,b,c,d,e,f));

            iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
            CorrectedPoseWorld.push_back(cv::Vec6d(a,b,c,d,e,f));
        }
    }
    CalcValue(Correct, PoseCamera, Skalierung, "PoseCamera");
    CalcValue(Correct, PoseWorld, Skalierung, "PoseWorld");
    CalcValue(Correct, CorrectedPoseCamera, Skalierung, "CorrectedPoseCamera");
    CalcValue(Correct, CorrectedPoseWorld, Skalierung, "CorrectedPoseWorld");
}

void allSacllQuality(){
    std::ofstream myfile;
    myfile.open("Solution_S005.txt", std::ios::in | std::ios::app);
    myfile<<"Eul_x Eul_y Eul_ges Head_x Head_y Head_ges Head_err Eye0_x Eye0_y Eye0_ges Eye0_err Eye1_x Eye1_y Eye1_ges Eye1_err EyeAVG_x EyeAVG_y EyeAVG_ges EyeAVG_err"<<std::endl;

    /*
    0 //FrameNr
    [1317, 981, 1556] //worlPoint
    [1332.5, 757.417, 0] //target
    [0.766757, -0.00969044, 0.0841916, 0.0401817, 730.427, 411.92] //Model
   |[-133.28, -0.819833, 1565.03, -0.0108466, -0.000978553, 0.0397614] //HeadPoseWorld
    [0.0819931, -0.100866, -0.991516] //GazeDirection0
    [0.00562416, -0.0127377, -0.999903] //GazeDirection1
    [678.815, 363.191, 108.592, 109.416] //Box
   |[0.00996111, -0.142714, 0.143052] //C-Welt
    [0.00097861, -0.0108466, 0.0108907, 1.43728] //C-Head
    [0.082507, -0.10138, 0.130357, 0.502081] //C-Eye0
    [0.00562465, -0.0127382, 0.0139245, 1.41023] //C-Eye1
    [0.0439691, -0.0569848, 0.0719179, 0.764222] //C-EyeAVG
    */

    std::vector<cv::String> FileNames;
    cv::glob("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/*_S005.txt",FileNames,false);

    cv::Rect2d Box(0,0,0,0);
    size_t count = 0;

    QRegExp ausdruck("[\\[\\]\\,\\|]");
    for(size_t fileNr = 0; fileNr < FileNames.size(); fileNr++){
        std::ifstream file(FileNames[fileNr]);
        std::string line;
        if(file.is_open()){
            std::cout<<FileNames[fileNr]<<std::endl;
            int lastFrame = -1;
            while (std::getline(file, line)){
                std::istringstream iss(QString::fromStdString(line).remove(ausdruck).toStdString());
                std::string tmp;
                int frame;
                double x,y,w,h;
                iss >> frame;
                if(lastFrame >= frame){
                    std::cout<<lastFrame<<" "<<frame<<std::endl;
                    std::cout<<line<<std::endl<<std::endl;
                }
                for(size_t i = 0; i < 3+3+6+6+3+3; i++){
                    iss >> tmp;
                }
                iss >> x, iss >> y, iss >> w, iss >> h;

                Box.x += x;
                Box.y += y;
                Box.width += w;
                Box.height += h;
                count++;

                iss >> tmp;
                myfile<<tmp;

                for(size_t i = 1; i < 4*5-1; i++){
                    iss >> tmp;
                    myfile<<" "<<tmp;
                }
                myfile<<std::endl;
            }
        }
    }
    myfile.close();
    std::cout<<Box.x/count<<" "<<Box.y/count<<" "<<Box.width/count<<" "<<Box.height/count<<std::endl;
}

void plotToFileX(int x, int y, double left, double right, std::string file)
{
    cv::Mat mat(50, 100, CV_8UC3, cv::Scalar(255,255,255));
    if(left == right){
        cv::line(mat, cv::Point(50,0),cv::Point2d(50+40*cos(left+M_PI/2.0),40*sin(left+M_PI/2.0)),cv::Scalar(0,0,0));
    }else{
        cv::ellipse(mat,cv::Point(50,0),cv::Size(40,40),0,left/M_PI*180+90,right/M_PI*180+90,cv::Scalar(0,0,0),-1);
    }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("img/"+file+"Winkel_X_"+std::to_string(x)+"_"+std::to_string(y)+".png",mat,compression_params);
}

void plotToFileY(int x, int y, double left, double right, std::string file)
{
    cv::Mat mat(60, 50, CV_8UC3, cv::Scalar(255,255,255));
    if(left == right){
        cv::line(mat, cv::Point(0,10),cv::Point2d(40*cos(left),10+40*sin(left)),cv::Scalar(0,0,0));
    }else{
        cv::ellipse(mat,cv::Point(0,10),cv::Size(40,40),0,(left)/M_PI*180,(right)/M_PI*180,cv::Scalar(0,0,0),-1);
    }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("img/"+file+"Winkel_Y_"+std::to_string(x)+"_"+std::to_string(y)+".png",mat,compression_params);
}


void clacMinMax(QString path, std::string file){
    std::cout<<"Verarbeitung: "<<file<<std::endl;
    QFile inputFile(path);

    QRegExp ausdruck("[\\[\\]\\,\\|]");

    /*
  1  1706
  3  [-2000, 1700, 3000]
  3  [0, 1880, 0]
  6  [0.419901, 0.0755197, 0.0360489, -0.0332329, 505.025, 618.92]
  6  | [-1982.83, -310.243, 3402.85, 0.174941, -0.484262, 0.0529449]
  3  [0.482726, 0.122777, -0.867122]
  3  [0.467892, 0.116284, -0.876102]
  4  [474.556, 592.56, 60.0952, 59.0288]
  3  | [0.588003, 0.0599282, 0.589866]
  4  [0.490627, 0.174941, 0.512522, 0.598107]
  4  [0.507972, 0.140657, 0.521401, 0.562244]
  4  [0.490524, 0.131958, 0.503079, 0.91263]
  4  [0.499244, 0.136287, 0.512238, 0.748886]
    */

    std::vector<cv::Vec4d> Werte;
    if (inputFile.open(QIODevice::ReadOnly))
    {
       QTextStream in(&inputFile);
       while (!in.atEnd())
       {
          QString line = in.readLine();
          QStringList list = line.remove(ausdruck).split(" ");
          if(list.size() >= 32){
              Werte.push_back(cv::Vec4d(list[1].toDouble(), list[3].toDouble(), list[29].toDouble(), list[31].toDouble()));
              //std::cout<<list[1].toStdString()<<" "<<list[3].toStdString()<<" "<<list[29].toStdString()<<" "<<list[30].toStdString()<<std::endl;
          }
       }
       inputFile.close();
    }
    std::vector<cv::Vec6d> Grenze;
    for(size_t i = 0; i < Werte.size(); i++){
        size_t pos = 0;
        while(pos < Grenze.size() && !(Grenze[pos][0] == Werte[i][0] && Grenze[pos][1] == Werte[i][1])){
            pos++;
        }
        if(pos >= Grenze.size()){
            Grenze.push_back(cv::Vec6d(Werte[i][0], Werte[i][1],
                                       Werte[i][2], Werte[i][2],
                                       Werte[i][3], Werte[i][3]));
        }else{
            Grenze[pos][2] = std::min(Grenze[pos][2],Werte[i][2]);
            Grenze[pos][3] = std::max(Grenze[pos][3],Werte[i][2]);
            Grenze[pos][4] = std::min(Grenze[pos][4],Werte[i][3]);
            Grenze[pos][5] = std::max(Grenze[pos][5],Werte[i][3]);
        }
    }
    for(size_t i = 0; i < Grenze.size(); i++){
        std::cout<<"Bereich:"<<Grenze[i][0]<<" "<<Grenze[i][1]
                <<" X: "<<Grenze[i][2]*180/M_PI<<" "<<Grenze[i][3]*180/M_PI
               <<" | Y: "<<Grenze[i][4]*180/M_PI<<" "<<Grenze[i][5]*180/M_PI<<std::endl;
        plotToFileX((int)Grenze[i][0], (int)Grenze[i][1], Grenze[i][2], Grenze[i][3],file);
        plotToFileY((int)Grenze[i][0], (int)Grenze[i][1], Grenze[i][4], Grenze[i][5],file);
    }
}

int main(int argc, char *argv[])
{
    clacMinMax("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/Test_Versuch_1_video_resize.txt", "V1_vid_res_");
    clacMinMax("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/Test_Versuch_1_video.txt", "V1_vid_");
    clacMinMax("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/Test_Versuch_1_image_resize.txt", "V1_img_res_");
    clacMinMax("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/Test_Versuch_1_image.txt", "V1_img_");

    //allSacllQuality();
    //fixXML("Thiago.xml");
    //FileToXML("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/Video_Analyse_falko_ng.txt","Falko_ng");
    //calcHeadPose("/home/falko/Uni/Master/build-Basisanwendung-Desktop-Debug/Gesicht_Pose_resize.txt");
    return 0;

    QCoreApplication a(argc, argv);

    return a.exec();
}
