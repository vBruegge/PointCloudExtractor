#include <sstream>

#include "Airfoil.hpp"

Airfoil::Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, AirfoilParameter& parameters_) {
    foil = foil_;
    parameters = parameters_;
    computeChordLength();  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Airfoil::getFoil() {
    return foil;
}
    
void Airfoil::setFoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_) {
    foil = foil_;
}

AirfoilParameter Airfoil::getAirfoilParameter() {
    return parameters;
}

void Airfoil::setAnyAirfoilParameter(std::string& parameter, float value) {
    switch (parameter)
    {
    case "dihedral":
        parameters.dihedral = value;
        break;
    case "twist":
        parameters.twist = value;
        break;
    case "cuttingDistance":
        parameters.cuttingDistance = value;
        break;
    case "chordLength":
        parameters.chordLength = value;
        break;
    case "flapPosition":
        parameters.flapPosition = value;
        break;
    case "offset":
        parameters.offset = value;
        break;
    case "sweep":
        parameters.sweep = value;
        break;
    case "trailingEdgeWidth":
        parameters.trailingEdgeWidth = value;
        break;
    default:
        break;
    }
}

void setAllAirfoilParameter(AirfoilParameter& parameters_) {
    parameters = parameters_;
}

void Airfoil::computeChordLength(){
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*foil, minPt, maxPt);

  parameters.chordLength = std::abs(minPt.y-maxPt.y);
}

void Airfoil::computeRotatedFlapPosition() {
  //returns flap position normalized
  std:vector<int> minMax = findingLeadingTrailingEdge(foil);
  pcl::PointXYZ minPt, maxPt;
  minPt = foil.getFoil()->points[indexMinMax[0]];
  maxPt = foil.getFoil()->points[indexMinMax[1]];

  float maxDis = sqrt(pow(minPt.y-maxPt.y, 2)+pow(minPt.z-maxPt.z, 2)+pow(minPt.x-maxPt.x,2));

  AirfoilParameter parameters = foil.getAirfoilParameter();
  float flap = maxDis/2 + parameters.flapPosition/cos(parameters.twist);

  float flapPos = flap / maxDis;
  foil.setAnyAirfoilParameter("flapPosition", flapPos);
}

float Airfoil::computeOffsetFromFirstSection(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float offsetFirstPoint) {
  int indexLeadingEdge = findLeadingTrailingEdge(inputCloud)[0];
  pcl::PointXYZ leadingEdge = inputCloud->points[indexLeadingEdge];
  return abs(offsetFirstPoint-leadingEdge.y);
}

void Airfoil::setName(std::string& sectionType) {
    stringstream ss;
    ss << std::setprecision(2);
    ss << "../Results/" << sectionType << parameters.cuttingDistance << "mm.dat";

    parameters.name = ss.str();
}

void Airfoil::generateMissingAirfoilParameter(std::string& sectionType, float offsetFirstPoint, float firstSection) {

    parameters.offset = computeOffsetFromFirstSection(foil, offsetFirstPoint); //offset in mm
    parameters.sweep = std::atan2(parameters.offset, (parameters.cuttingDistance - firstSection));
    
    setName(sectionType);
    computeRotatedFlapPosition();
}