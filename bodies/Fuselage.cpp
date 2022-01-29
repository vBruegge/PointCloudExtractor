#include "Fuselage.hpp"

Fuselage::Fuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr section) {
    fuselage = section;
    computeFuselageParameter();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fuselage::getFoil() {
    return foil;
}
    
void Fuselage::setFuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage_) {
    fuselage = fuselage_;
}

FuselageParameter Fuselage::getFuselageParameter() {
    return parameters;
}

void Fuselage::setAnyFuselageParameter(std::string parameter, float value) {
    switch (parameter)
    {
    case "cuttingDistance":
        parameters.cuttingDistance = value;
        break;
    case "disX":
        parameters.disX = value;
        break;
    case "disY":
        parameters.disY = value;
        break;
    case "xM":
        parameters.xM = value;
        break;
    case "yM":
        parameters.yM = value;
        break;
    case "epsilon":
        parameters.epsilon = value;
        break;
    default:
        break;
    }
}

void setAllFuselageParameter(FuselageParameter parameters_) {
    parameters = parameters_;
}

void computeFuselageParameter() {
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*fuselage, min, max);

    double disX = abs(min.x-max.x)/2;
    double disY = abs(min.y-max.y)/2;

    double xM = (min.x+max.x)/2;
    double yM = (min.y+max.y)/2;

    FuselageParameters params = {"", 0, disX, disY, xM, yM, 1};
    parameters = params;
}
