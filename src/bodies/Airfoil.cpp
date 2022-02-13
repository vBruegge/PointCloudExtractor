#include <sstream>
#include <iomanip>
#include <cmath>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

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
    case AirfoilParameter::PosLeadingEdgeX:
        parameters.posLeadingEdge.x = value;
    case AirfoilParameter::PosLeadingEdgeY:
        parameters.posLeadingEdge.y = value;
    case AirfoilParameter::PosLeadingEdgeZ:
        parameters.posLeadingEdge.z = value;
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
  std::vector<int> indexMinMax = findLeadingTrailingEdge();
  pcl::PointXYZ minPt, maxPt;
  minPt = foil->points[indexMinMax[0]];
  maxPt = foil->points[indexMinMax[1]];

  float maxDis = sqrt(pow(minPt.y-maxPt.y, 2)+pow(minPt.z-maxPt.z, 2)+pow(minPt.x-maxPt.x,2));

  AirfoilParameter parameters = getAirfoilParameter();
  float flap = maxDis/2 + parameters.flapPosition/cos(parameters.twist);

  float flapPos = flap / maxDis;
  setAnyAirfoilParameter(AirfoilParameter::parameterType::FlapPosition, flapPos);
}

void Airfoil::setName(std::string& sectionType) {
    std::stringstream ss;
    ss << "../Results/" << sectionType << "_" << parameters.cuttingDistance << "mm.dat";

    parameters.name = ss.str();
}

void Airfoil::generateMissingAirfoilParameter(std::string& sectionType, pcl::PointXYZ posFirstLeadingEdge) {

    parameters.offset = posFirstLeadingEdge.y-parameters.posLeadingEdge.y;
    parameters.sweep = std::atan2(parameters.offset, (parameters.cuttingDistance - posFirstLeadingEdge.x))*180.0/M_PI;
    
    setName(sectionType);
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
