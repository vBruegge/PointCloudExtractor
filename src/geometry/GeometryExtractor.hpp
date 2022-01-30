#ifndef GEOMETRYEXTRACTOR_H
#define GEOMETRYEXTRACTOR_H

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Core>
#include <vector>

#include "Airfoil.hpp"
#include "Fuselage.hpp"

class GeometryExtractor {
public:

    Airfoil sectioningCloudX(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded);
    //generates a section in a distance of the x-Axis. Surface normals are used to create the plane.

    Fuselage sectioningCloudY(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float cuttingDistance);
    //genereates a section in a distance of the y-Axis

    Airfoil sectioningCloudZ(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded);
    //generates a section in a distance of the z-Axis

    void derotateSection(Airfoil& foil);
    //calculates the twist of an airfoil to the xy-plane

    void translateSection(Airfoil& foil);

    void deleteTrailingEdge(Airfoil& foil, int indexTrailingEdge, float distanceFromTrailingEdge);

private:
    int getIndexFlapPosition(pcl::PointCloud <pcl::PointNormal>::Ptr inputCloud, std::vector<int>& indexLeadingTrailingEdge);

    Eigen::Vector3f computeAverageNormalOfFoil(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint, int indexTrailingEdge, int iterate, int n, int direction);

    pcl::PointXYZ getOriginalPosition(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint);

    Airfoil derotateFlap (pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float dihedral);

    bool getPolynomialCoeff(std::vector<double> x, std::vector<double> y, double coeff[], int degree);

};

#endif