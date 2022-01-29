#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>

#include "Airfoil.hpp"
#include "Fuselage.hpp"


bool writingPointCloud( const std::string& filename, const std::vector<Eigen::Vector2d>& points );

bool writingWingDataInCSV( std::ofstream& outStream, AirfoilParameters data[], std::string sectionType, int length);

bool writingFuselageDataInCSV( std::ofstream& outStream, FuselageParameters data[], int length);

void writing_airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::string name, float trailingEdgeWidth);

void writing_wing_section_data(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, std::vector<float> sections,
  std::string sectionCut, bool flapRotationNeeded, std::ofstream& aircraftDataFile);

void writing_fuselage_section_data(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::vector<float> sections,
  std::ofstream& aircraftDataFile, std::string fittingType = "superellipse");