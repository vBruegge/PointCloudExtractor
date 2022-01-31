#include <sstream>
#include <iomanip>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

#include "Airfoil.hpp"

Airfoil::Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, AirfoilParameter& parameters_) {
    foil = foil_;
    airfoilParameters = parameters_;
    computeChordLength();  
}

Airfoil::Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, MorphingWingParameter& parameters_) {
    foil = foil_;
    morphingWingParameters = parameters_; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Airfoil::getFoil() {
    return foil;
}
    
void Airfoil::setFoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_) {
    foil = foil_;
}

AirfoilParameter Airfoil::getAirfoilParameter() {
    return airfoilParameters;
}

MorphingWingParameter Airfoil::getMorphingWingParameter() {
    return morphingWingParameters;
}

void Airfoil::setAnyAirfoilParameter(AirfoilParameter::parameterType type, float value) {
    switch (type)
    {
    case AirfoilParameter::Dihedral:
        airfoilParameters.dihedral = value;
        break;
    case AirfoilParameter::Twist:
        airfoilParameters.twist = value;
        break;
    case AirfoilParameter::CuttingDistance:
        airfoilParameters.cuttingDistance = value;
        break;
    case AirfoilParameter::ChordLength:
        airfoilParameters.chordLength = value;
        break;
    case AirfoilParameter::FlapPosition:
        airfoilParameters.flapPosition = value;
        break;
    case AirfoilParameter::Offset:
        airfoilParameters.offset = value;
        break;
    case AirfoilParameter::Sweep:
        airfoilParameters.sweep = value;
        break;
    case AirfoilParameter::TrailingEdgeWidth:
        airfoilParameters.trailingEdgeWidth = value;
        break;
    default:
        break;
    }
}

void Airfoil::setAnyMorphingWingParameter(MorphingWingParameter::parameterType type, float value) {
    switch (type)
    {
    case MorphingWingParameter::parameterType::CuttingDistance:
        morphingWingParameters.cuttingDistance = value;
        break;
    case MorphingWingParameter::parameterType::IndexFirstReference:
        morphingWingParameters.indexFirstReference = (int)value;
        break;
    case MorphingWingParameter::parameterType::IndexSecondReference:
        morphingWingParameters.indexSecondReference = (int)value;
        break;
    case MorphingWingParameter::parameterType::Scale:
        morphingWingParameters.scale = value;
        break;
    case MorphingWingParameter::parameterType::RotationAngle:
        morphingWingParameters.rotationAngle = value;
        break;
    default:
        break;
    }
}

void Airfoil::setAllAirfoilParameter(AirfoilParameter& parameters_) {
    airfoilParameters = parameters_;
}

void Airfoil::setAllMorphingWingParameter(MorphingWingParameter& parameters_) {
    morphingWingParameters = parameters_;
}

void Airfoil::computeChordLength(){
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*foil, minPt, maxPt);

  airfoilParameters.chordLength = std::abs(minPt.y-maxPt.y);
}

void Airfoil::computeRotatedFlapPosition() {
  //returns flap position normalized
  std::vector<int> indexMinMax = findLeadingTrailingEdge(foil);
  pcl::PointXYZ minPt, maxPt;
  minPt = foil->points[indexMinMax[0]];
  maxPt = foil->points[indexMinMax[1]];

  float maxDis = sqrt(pow(minPt.y-maxPt.y, 2)+pow(minPt.z-maxPt.z, 2)+pow(minPt.x-maxPt.x,2));

  AirfoilParameter airfoilParameters = getAirfoilParameter();
  float flap = maxDis/2 + airfoilParameters.flapPosition/cos(airfoilParameters.twist);

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
    ss << "../Results/" << sectionType << airfoilParameters.cuttingDistance << "mm.dat";

    airfoilParameters.name = ss.str();
}

void Airfoil::generateMissingAirfoilParameter(std::string& sectionType, float offsetFirstPoint, float firstSection) {

    airfoilParameters.offset = computeOffsetFromFirstSection(foil, offsetFirstPoint); //offset in mm
    airfoilParameters.sweep = std::atan2(airfoilParameters.offset, (airfoilParameters.cuttingDistance - firstSection));
    
    setName(sectionType);
    computeRotatedFlapPosition();
}

std::vector<int> Airfoil::findLeadingTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    //find the index of the min and max of the input cloud
    int indexTrailingEdge = findTrailingEdge(inputCloud);
    std::vector<int> indexLeadingTrailingEdge(2);
    indexLeadingTrailingEdge[1] = indexTrailingEdge;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (inputCloud);

    int n = inputCloud->size();
    std::vector<int> pointIndexSearch(n);
    std::vector<float> pointDistanceSearch(n);
    kdtree.nearestKSearch (inputCloud->points[indexTrailingEdge], n, pointIndexSearch, pointDistanceSearch);
    
    indexLeadingTrailingEdge[0] = pointIndexSearch[n-1];
    
    return indexLeadingTrailingEdge;
}

int Airfoil::findTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
  //returns position of the trailing edge
  //comparison of the z distance at the front and the back -> smaller distance: trailing edge
  pcl::PointXYZ center(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < inputCloud->size(); i++) {
    center.x += inputCloud->points[i].x;
    center.y += inputCloud->points[i].y;
    center.z += inputCloud->points[i].z;
  }
  center.x /= inputCloud->size();
  center.y /= inputCloud->size();
  center.z /= inputCloud->size();
  //std::cout << center.x << " "  << center.y << " " << center.z << std::endl;
  

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (inputCloud);

  int n = inputCloud->size();
  std::vector<int> pointIndexSearch(n);
  std::vector<float> pointDistanceSearch(n);
  kdtree.nearestKSearch (center, n, pointIndexSearch, pointDistanceSearch);
  
  int indexTrailingEdge = pointIndexSearch[n-1];
  pcl::PointXYZ trailingEdgeTmp = inputCloud->points[indexTrailingEdge];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (inputCloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (trailingEdgeTmp.y-0.5, trailingEdgeTmp.y+0.5);
  pass.filter (*cloudPassThrough);

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*cloudPassThrough, min, max);

  pcl::PointXYZ centerTrailingEdge = pcl::PointXYZ(trailingEdgeTmp.x, trailingEdgeTmp.y, (min.z+max.z)/2);
  kdtree.nearestKSearch (centerTrailingEdge, 1, pointIndexSearch, pointDistanceSearch);
  indexTrailingEdge = pointIndexSearch[0];

  //std::cout << inputCloud->points[indexTrailingEdge].x << " "  << inputCloud->points[indexTrailingEdge].y << " " << inputCloud->points[indexTrailingEdge].z << std::endl;
  return indexTrailingEdge;
}
