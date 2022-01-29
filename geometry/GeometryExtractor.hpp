#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Core>

#include "Airfoil.hpp"
#include "Fuselage.hpp"

class GeometryExtractor {
public:

Airfoil section_cloud_x(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded);
//generates a section in a distance of the x-Axis. Surface normals are used to create the plane.

Fuselage section_cloud_y(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float cuttingDistance);
//genereates a section in a distance of the y-Axis

Airfoil section_cloud_z(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded);
//generates a section in a distance of the z-Axis

void derotate_section(Airfoil& foil);
//calculates the twist of an airfoil to the xy-plane

void translate_section(Airfoil& foil);

Airfoil derotate_flap (pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float dihedral);

float deleteTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, int indexTrailingEdge, float distanceFromTrailingEdge);

private:
    int getIndexFlapPosition(pcl::PointCloud <pcl::PointNormal>::Ptr inputCloud, std::vector<int>& indexLeadingTrailingEdge);

    Eigen::Vector3f computeAverageNormalOfFoil(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint, int indexTrailingEdge, int iterate, int n, int direction);

    pcl::PointXYZ getOriginalPosition(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint);
};