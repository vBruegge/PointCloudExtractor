#ifndef AIRFOIL_H
#define AIRFOIL_H

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

#include <string>
#include <vector>

class AirfoilParameter {
public:
    std::string name;
    float cuttingDistance;
    float chordLength;
    float dihedral;
    float twist;
    float flapPosition;
    float offset;
    float sweep;
    float trailingEdgeWidth;
    enum parameterType {CuttingDistance, ChordLength, Dihedral, Twist, FlapPosition, Offset, Sweep, TrailingEdgeWidth};
};

class Airfoil {
public:
    Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, AirfoilParameter& parameters_);

    Airfoil() = default;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFoil();

    void setFoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_);

    AirfoilParameter getAirfoilParameter();

    void setAnyAirfoilParameter(AirfoilParameter::parameterType type, float value);

    void setAllAirfoilParameter(AirfoilParameter& parameters_);

    std::vector<int> findLeadingTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    void generateMissingAirfoilParameter(std::string& sectionType, float offsetFirstPoint, float firstSection);
    
private:
    int findTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    void computeChordLength();

    void computeFlapPosition(Airfoil& foil, std::vector<int> indexMinMax);

    void setName(std::string& sectionType);

    float computeOffsetFromFirstSection(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float offsetFirstPoint);

    void computeRotatedFlapPosition();

    pcl::PointCloud<pcl::PointXYZ>::Ptr foil;
    AirfoilParameter parameters;
};

#endif