#include <sstream>
#include <iomanip>
#include <cmath>

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
        case AirfoilParameter::PosLeadingEdgeX:
        airfoilParameters.posLeadingEdge.x = value;
    case AirfoilParameter::PosLeadingEdgeY:
        airfoilParameters.posLeadingEdge.y = value;
    case AirfoilParameter::PosLeadingEdgeZ:
        airfoilParameters.posLeadingEdge.z = value;
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
  std::vector<int> indexMinMax = findLeadingTrailingEdge();
  pcl::PointXYZ minPt, maxPt;
  minPt = foil->points[indexMinMax[0]];
  maxPt = foil->points[indexMinMax[1]];

  float maxDis = sqrt(pow(minPt.y-maxPt.y, 2)+pow(minPt.z-maxPt.z, 2)+pow(minPt.x-maxPt.x,2));

  AirfoilParameter airfoilParameters = getAirfoilParameter();
  float flap = maxDis/2 + airfoilParameters.flapPosition/cos(airfoilParameters.twist);

  float flapPos = flap / maxDis;
  setAnyAirfoilParameter(AirfoilParameter::parameterType::FlapPosition, flapPos);
}

void Airfoil::setName(std::string& sectionType) {
    std::stringstream ss;
    ss << "../Results/" << sectionType << "_" << airfoilParameters.cuttingDistance << "mm.dat";

    airfoilParameters.name = ss.str();
}

void Airfoil::generateMissingAirfoilParameter(std::string& sectionType, pcl::PointXYZ posFirstLeadingEdge) {

    airfoilParameters.offset = posFirstLeadingEdge.y - airfoilParameters.posLeadingEdge.y; //offset in mm
    airfoilParameters.sweep = std::atan2(airfoilParameters.offset, (airfoilParameters.cuttingDistance - posFirstLeadingEdge.x))*180.0/M_PI;
    
    setName(sectionType);
    if(airfoilParameters.flapPosition != 0)
        computeRotatedFlapPosition();
}

std::vector<int> Airfoil::findLeadingTrailingEdge() {
    //find the index of the min and max of the input cloud
    int indexTrailingEdge = findTrailingEdge();
    std::vector<int> indexLeadingTrailingEdge(2);
    indexLeadingTrailingEdge[1] = indexTrailingEdge;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (foil);

    int n = foil->size();
    std::vector<int> pointIndexSearch(n);
    std::vector<float> pointDistanceSearch(n);
    kdtree.nearestKSearch (foil->points[indexTrailingEdge], n, pointIndexSearch, pointDistanceSearch);
    
    indexLeadingTrailingEdge[0] = pointIndexSearch[n-1];
    
    return indexLeadingTrailingEdge;
}

int Airfoil::findTrailingEdge() {
  //returns position of the trailing edge
  //comparison of the z distance at the front and the back -> smaller distance: trailing edge
  pcl::PointXYZ center(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < foil->size(); i++) {
    center.x += foil->points[i].x;
    center.y += foil->points[i].y;
    center.z += foil->points[i].z;
  }
  center.x /= foil->size();
  center.y /= foil->size();
  center.z /= foil->size();
  //std::cout << center.x << " "  << center.y << " " << center.z << std::endl;
  

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (foil);

  int n = foil->size();
  std::vector<int> pointIndexSearch(n);
  std::vector<float> pointDistanceSearch(n);
  kdtree.nearestKSearch (center, n, pointIndexSearch, pointDistanceSearch);
  
  int indexTrailingEdge = pointIndexSearch[n-1];
  pcl::PointXYZ trailingEdgeTmp = foil->points[indexTrailingEdge];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (foil);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (trailingEdgeTmp.y-5, trailingEdgeTmp.y+5);
  pass.filter (*cloudPassThrough);

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*cloudPassThrough, min, max);

  pcl::PointXYZ centerTrailingEdge = pcl::PointXYZ(trailingEdgeTmp.x, trailingEdgeTmp.y, (min.z+max.z)/2);
  kdtree.nearestKSearch (centerTrailingEdge, 1, pointIndexSearch, pointDistanceSearch);
  indexTrailingEdge = pointIndexSearch[0];

  //std::cout << inputCloud->points[indexTrailingEdge].x << " "  << inputCloud->points[indexTrailingEdge].y << " " << inputCloud->points[indexTrailingEdge].z << std::endl;
  return indexTrailingEdge;
}

void Airfoil::deleteMorphingWingReferences(float widthReferences) {
    int indexTrailingEdge = findTrailingEdge();
    pcl::PointXYZ trailingEdgePoint = foil->points[indexTrailingEdge];

    pcl::PointCloud<pcl::PointXYZ>::Ptr upper(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lower(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (foil);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (trailingEdgePoint.z, FLT_MAX);
    pass.filter (*upper);
    pass.setFilterLimits (-FLT_MAX, trailingEdgePoint.z);
    pass.filter (*lower);
    pcl::PointXYZ minUpper, maxUpper, minLower, maxLower;
    pcl::getMinMax3D(*upper, minUpper, maxUpper);
    pcl::getMinMax3D(*lower, minLower, maxLower);
    pcl::PointCloud<pcl::PointXYZ>::Ptr save(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr leftSide(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rightSide(new pcl::PointCloud<pcl::PointXYZ>);
    if(abs(minUpper.z-maxUpper.z) > abs(minLower.z-maxLower.z)) {
        pass.setInputCloud(lower);
        save = upper;
    }
    else {
        pass.setInputCloud(upper);
        save = lower;
    }

    pcl::PointXYZ firstReference = foil->points[morphingWingParameters.indexFirstReference];
    pcl::PointXYZ secondReference = foil->points[morphingWingParameters.indexSecondReference];
    pcl::PointCloud<pcl::PointXYZ>::Ptr deleted(new pcl::PointCloud<pcl::PointXYZ>);

    pass.setFilterFieldName("y");
    if(firstReference.y < trailingEdgePoint.y) {
        pass.setFilterLimits(firstReference.y+0.25, FLT_MAX);
        pass.filter(*rightSide);
        pass.setFilterLimits(-FLT_MAX, firstReference.y-0.25-widthReferences);
        pass.filter(*leftSide);
        pcl::concatenate(*leftSide, *rightSide, *deleted);
        pass.setInputCloud(deleted);
        pass.setFilterLimits(secondReference.y+0.25, FLT_MAX);
        pass.filter(*rightSide);
        pass.setFilterLimits(-FLT_MAX, secondReference.y-0.25-widthReferences);
        pass.filter(*leftSide);
    }
    else {
        pass.setFilterLimits(-FLT_MAX, firstReference.y-0.25);
        pass.filter(*rightSide);
        pass.setFilterLimits(firstReference.y+0.25+widthReferences, FLT_MAX);
        pass.filter(*leftSide);
        pcl::concatenate(*leftSide, *rightSide, *deleted);
        pass.setInputCloud(deleted);
        pass.setFilterLimits(-FLT_MAX, secondReference.y-0.25);
        pass.filter(*rightSide);
        pass.setFilterLimits(secondReference.y+0.25+widthReferences, FLT_MAX);
        pass.filter(*leftSide);
    }
    pcl::concatenate(*leftSide, *rightSide, *deleted);
    pcl::concatenate(*deleted, *save, *foil);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (foil);
    std::vector<int> pointIndexSearch(1);
    std::vector<float> pointDistanceSearch(1);
    kdtree.nearestKSearch (firstReference, 1, pointIndexSearch, pointDistanceSearch);
    morphingWingParameters.indexFirstReference = pointIndexSearch[0];
    kdtree.nearestKSearch (secondReference, 1, pointIndexSearch, pointDistanceSearch);
    morphingWingParameters.indexSecondReference = pointIndexSearch[0];
}