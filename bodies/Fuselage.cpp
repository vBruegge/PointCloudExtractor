#include "Fuselage.hpp"

Fuselage::Fuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr section) {
    fuselage = section;
    computeFuselageParameter();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fuselage::getFuselage() {
    return fuselage;
}
    
void Fuselage::setFuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage_) {
    fuselage = fuselage_;
}

FuselageParameter Fuselage::getFuselageParameter() {
    return parameters;
}

void Fuselage::setAnyFuselageParameter(FuselageParameter::parameterType type, float value) {
    switch (type)
    {
    case FuselageParameter::parameterType::CuttingDistance:
        parameters.cuttingDistance = value;
        break;
    case FuselageParameter::parameterType::DisX:
        parameters.disX = value;
        break;
    case FuselageParameter::parameterType::DisY:
        parameters.disY = value;
        break;
    case FuselageParameter::parameterType::XM:
        parameters.xM = value;
        break;
    case FuselageParameter::parameterType::YM:
        parameters.yM = value;
        break;
    case FuselageParameter::parameterType::Epsilon:
        parameters.epsilon = value;
        break;
    default:
        break;
    }
}

void Fuselage::setAllFuselageParameter(FuselageParameter parameters_) {
    parameters = parameters_;
}

void Fuselage::computeFuselageParameter() {
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*fuselage, min, max);

    double disX = abs(min.x-max.x)/2;
    double disY = abs(min.y-max.y)/2;

    double xM = (min.x+max.x)/2;
    double yM = (min.y+max.y)/2;

    FuselageParameter params = {"", 0, disX, disY, xM, yM, 1};
    parameters = params;
}
