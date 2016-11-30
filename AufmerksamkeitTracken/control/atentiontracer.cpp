#include "atentiontracer.h"

#include <iostream>

AtentionTracer::AtentionTracer()
{

}

AtentionTracer::~AtentionTracer()
{

}

void AtentionTracer::newEulerPosition(int model, cv::Vec6d headPose){
    std::cout<<"Modell "<<model<<": "<<headPose<<std::endl;
}
