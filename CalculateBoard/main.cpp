#include <QCoreApplication>

#include "LandmarkCoreIncludes.h"
#include <opencv2/opencv.hpp>

bool LineToVector(std::string Line, cv::Vec6d &Pose){
    if(Line.size() <= 1)
        return false;

    std::size_t Pa = Line.find('|')+2;
    std::size_t Pb = Line.rfind('|');
    if(Pb-2 - Pa+1 < 11 || Pa >= Line.size() || (int)Pb-2 <= 0)
        return false;

    double a,b,c,d,e,f;
    std::istringstream iss(Line.substr(Pa,Pb-Pa-1));
    iss >> a,iss >> b,iss >> c, iss >> d,iss >> e,iss >> f;
    Pose[0]=a;
    Pose[1]=b;
    Pose[2]=c;
    Pose[3]=d;
    Pose[4]=e;
    Pose[5]=f;

    return true;
}

void VectorToNP(cv::Vec6d Pose, cv::Vec3d &P, cv::Vec3d &N){
    N = LandmarkDetector::Euler2RotationMatrix(cv::Vec3d(Pose[3],Pose[4],Pose[5])) * cv::Vec3d(0,0,-1);
    P = cv::Vec3d(Pose[0],Pose[1],Pose[2]);
}

cv::Vec3d CalcCenter(std::vector<cv::Vec3d> Ps, std::vector<cv::Vec3d> Ns){
    cv::Matx33d rot(0,0,0,
                    0,0,0,
                    0,0,0);
    cv::Vec3d sum(0,0,0);

    cv::Matx33d I(1,0,0,
                  0,1,0,
                  0,0,1);
    cv::Vec3d avgP(0,0,0);
    cv::Vec3d avgN(0,0,0);
    for(size_t i = 0; i < Ps.size(); i++){
        cv::Vec3d n = Ns[i];
        double norm = sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
        n *= 1/norm;
        cv::Matx33d e = I-n*n.t();
        rot += e;
        sum += e*Ps[i];
        avgN += n;
        avgP += Ps[i];
    }
    avgP = avgP * (1.0/((double)Ps.size()));
    avgN = avgN * (1.0/((double)Ps.size()));
    std::cout<<Ps.size()<<" AVG: "<<avgP<<" "<<avgN<<" -> "<<avgP+avgN*avgP[2]<<std::endl;
    return rot.inv()*sum;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    std::ifstream file("/home/falko/Uni/Master/build-AufmerksamkeitTracken-Desktop-Debug/data/BerechnungWinkel_Gaze_2.txt");
    std::string line;

    std::vector<cv::Vec3d> Ps;
    std::vector<cv::Vec3d> Ns;
    while (std::getline(file, line)){
        cv::Vec6d pose;
        if(!LineToVector(line,pose)){
            std::cout<<line<<std::endl;
        }else{
            cv::Vec3d p,n;
            VectorToNP(pose, p,n);
            Ps.push_back(p);
            Ns.push_back(n);
        }
    }
    std::cout<<"Ergebnis: "<<CalcCenter(Ps,Ns)<<std::endl;

    std::vector<cv::Vec3d> TPs;
    std::vector<cv::Vec3d> TNs;

    TPs.push_back(cv::Vec3d(3,0,0));
    TNs.push_back(cv::Vec3d(0,1,0));

    TPs.push_back(cv::Vec3d(0,0,0));
    TNs.push_back(cv::Vec3d(0.5,0.5,0));

    TPs.push_back(cv::Vec3d(0,3,0));
    TNs.push_back(cv::Vec3d(-1,0,0));

    TPs.push_back(cv::Vec3d(2,0,0));
    TNs.push_back(cv::Vec3d(1,3,0));

    TPs.push_back(cv::Vec3d(0,2,0));
    TNs.push_back(cv::Vec3d(-3,-1,0));
    std::cout<<"Test: "<<CalcCenter(TPs,TNs)<<std::endl;

    return a.exec();
}
