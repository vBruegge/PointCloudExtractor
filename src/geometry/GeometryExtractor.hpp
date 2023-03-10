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

    /**
     * @brief generates a section in a distance of the x-Axis rotated around the dihedral. Surface normals are used to create the plane.
     * used for sectioning the wing and the horizontal or v-tail respectively
     * 
     * @param inputCloud cloud which should be sectioned
     * @param cuttingDistance distance where to section
     * @param sectioningType 0: no flaps, no morphing wing, 1: flaps, 2: morphing wing
     * @return Airfoil sectioned airfoil data
     */
    Airfoil sectioningCloudX(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, int sectioningType);

    /**
     * @brief genereates a section in a distance of the y-Axis
     * used for sectioning the fuselage
     * 
     * @param inputCloud cloud which should be sectioned
     * @param cuttingDistance distance where to section
     * @return Fuselage sectioned fuselage data
     */
    Fuselage sectioningCloudY(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float cuttingDistance);

    /**
     * @brief generates a section in a distance of the z-Axis
     * used for sectioning the vertical tail
     * 
     * @param inputCloud cloud which should be sectioned
     * @param cuttingDistance distance where to section
     * @param flapRotationNeeded boolean if the flap was rotated before the scanning
     * @return Airfoil sectioned airfoil data
     */
    Airfoil sectioningCloudZ(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded);

    /**
     * @brief derotates the given foil around the twist angle
     * 
     * @param foil foil which should be derotated
     */
    void derotateSection(Airfoil& foil);

    /**
     * @brief translates the center of the foil to the center of the CoSy
     * 
     * @param foil foil which should be translated
     */
    void translateSection(Airfoil& foil);

    /**
     * @brief deletes all points at the trailing edge in a given distance
     * 
     * @param foil foil where to delete the trsiling edge
     * @param indexTrailingEdge index of the point of the trailing edge in the foil
     * @param distanceFromTrailingEdge distance from where to delete in mm
     */
    void deleteTrailingEdge(Airfoil& foil, int indexTrailingEdge, float distanceFromTrailingEdge);

    /**
     * @brief find the reference points of the extracted morphing wing by a comparison if the normal vectors of neighbouring points
     * 
     * @param inputCloud sectioned morphing wing with normals
     * @return Airfoil sectioned morphing wing without normals, deleted references and additional morphing wing parameters
     */
    Airfoil findingMorphedReferencePoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);

    /**
     * @brief rotates, scales and translates the detected reference points of the airfoil on the given reference points
     * 
     * @param foil sectioned morphing wing
     * @param firstReference coordinates of the first reference point
     * @param secondReference coordinates of the second reference point
     */
    void derotateToReferencePoints(Airfoil& foil, pcl::PointXYZ& firstReference, pcl::PointXYZ& secondReference);

private:
    int getIndexFlapPosition(pcl::PointCloud <pcl::PointNormal>::Ptr inputCloud, std::vector<int>& indexLeadingTrailingEdge);

    Eigen::Vector3f computeAverageNormalOfFoil(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint, int indexTrailingEdge, int iterate, int n, int direction);

    pcl::PointXYZ getOriginalPosition(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint);

    Airfoil derotateFlap (pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float dihedral);

    bool getPolynomialCoeff(std::vector<double> x, std::vector<double> y, double coeff[], int degree);

    void translateSectionToReference(Airfoil& foil, pcl::PointXYZ reference);

};

#endif