#include <sstream>
#include <iomanip>

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

void Airfoil::setAnyAirfoilParameter(AirfoilParameter::parameterType type, float value) {
    switch (type)
    {
    case AirfoilParameter::Dihedral:
        parameters.dihedral = value;
        break;
    case AirfoilParameter::Twist:
        parameters.twist = value;
        break;
    case AirfoilParameter::CuttingDistance:
        parameters.cuttingDistance = value;
        break;
    case AirfoilParameter::ChordLength:
        parameters.chordLength = value;
        break;
    case AirfoilParameter::FlapPosition:
        parameters.flapPosition = value;
        break;
    case AirfoilParameter::Offset:
        parameters.offset = value;
        break;
    case AirfoilParameter::Sweep:
        parameters.sweep = value;
        break;
    case AirfoilParameter::TrailingEdgeWidth:
        parameters.trailingEdgeWidth = value;
        break;
    default:
        break;
    }
}

void Airfoil::setAllAirfoilParameter(AirfoilParameter& parameters_) {
    parameters = parameters_;
}

void Airfoil::computeChordLength(){
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*foil, minPt, maxPt);

  parameters.chordLength = std::abs(minPt.y-maxPt.y);
}

void Airfoil::computeRotatedFlapPosition() {
  //returns flap position normalized
  std::vector<int> indexMinMax = findLeadingTrailingEdge(foil);
  pcl::PointXYZ minPt, maxPt;
  minPt = foil->points[indexMinMax[0]];
  maxPt = foil->points[indexMinMax[1]];

  float maxDis = sqrt(pow(minPt.y-maxPt.y, 2)+pow(minPt.z-maxPt.z, 2)+pow(minPt.x-maxPt.x,2));

  AirfoilParameter parameters = getAirfoilParameter();
  float flap = maxDis/2 + parameters.flapPosition/cos(parameters.twist);

  float flapPos = flap / maxDis;
  setAnyAirfoilParameter(AirfoilParameter::parameterType::FlapPosition, flapPos);
}

float Airfoil::computeOffsetFromFirstSection(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float offsetFirstPoint) {
  int indexLeadingEdge = findLeadingTrailingEdge(inputCloud)[0];
  pcl::PointXYZ leadingEdge = inputCloud->points[indexLeadingEdge];
  return abs(offsetFirstPoint-leadingEdge.y);
}

void Airfoil::setName(std::string& sectionType) {
    std::stringstream ss;
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