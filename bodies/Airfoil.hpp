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
};

class Airfoil {
public:
    Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, AirfoilParameter& parameters_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFoil();

    void setFoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_);

    AirfoilParameter getAirfoilParameter();

    void setAnyAirfoilParameter(std::string& parameter, float value);

    void setAllAirfoilParameter(AirfoilParameter& parameters_);

    std::vector<int> findLeadingTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    float computeRotatedFlapPosition(Airfoil& foil, std::vector<int>& indexMinMax);

    float computeOffsetFromFirstSection(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float offsetFirstPoint);

private:
    int findTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    void computeChordLength();

    void computeFlapPosition(Airfoil& foil, std::vector<int> indexMinMax);

    pcl::PointCloud<pcl::PointXYZ>::Ptr foil;
    AirfoilParameter parameters;
};