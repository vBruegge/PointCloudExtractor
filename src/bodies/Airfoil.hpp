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

class MorphingWingParameter {
public:
    std::string name;
    float cuttingDistance;
    int indexFirstReference;
    int indexSecondReference;
    float scale;
    float rotationAngle;
    enum parameterType {CuttingDistance, IndexFirstReference, IndexSecondReference, Scale, RotationAngle};
};

class Airfoil {
public:
    /**
     * @brief Construct a new Airfoil object
     * 
     * @param foil_ Point Cloud of the foil
     * @param parameters_ known airfoil parameter (e.g. chord)
     */
    Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, AirfoilParameter& parameters_);

    Airfoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_, MorphingWingParameter& parameters_);

    Airfoil() = default;

    /**
     * @brief Get the Foil object
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr pointer to the foil point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFoil();

    /**
     * @brief Set the Foil object
     * 
     * @param foil_ pointer to the new foil point cloud
     */
    void setFoil(pcl::PointCloud<pcl::PointXYZ>::Ptr foil_);

    /**
     * @brief Get the Airfoil Parameter object
     * 
     * @return AirfoilParameter known parameter
     */
    AirfoilParameter getAirfoilParameter();

    /**
     * @brief Get the Morphing Wing Parameter object
     * 
     * @return MorphingWingParameter  known parameter
     */
    MorphingWingParameter getMorphingWingParameter();

    /**
     * @brief Set the one of the Morphing Wing Parameter
     * 
     * @param type parameter type (CuttingDistance, Scale, RotationAnge...)
     * @param value value of the parameter
     */
    void setAnyMorphingWingParameter(MorphingWingParameter::parameterType type, float value);

    /**
     * @brief replace all Morphing Wing parameters with new ones
     * 
     * @param parameters_ new parameters
     */
    void setAllMorphingWingParameter(MorphingWingParameter& parameter_);

    /**
     * @brief Set the one of the Airfoil Parameter
     * 
     * @param type parameter type (CuttingDistance, Dihedral, ChordLength...)
     * @param value value of the parameter
     */
    void setAnyAirfoilParameter(AirfoilParameter::parameterType type, float value);

    /**
     * @brief replace all Airfoil parameters with new ones
     * 
     * @param parameters_ new parameters
     */
    void setAllAirfoilParameter(AirfoilParameter& parameters_);

    /**
     * @brief finds the index of the leading and trailing edge
     * 
     * @param inputCloud foil to search for
     * @return std::vector<int> index of the point cloud of (first) leading and (second) trailing edge
     */
    std::vector<int> findLeadingTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    /**
     * @brief computes missing airfoil parameter which are not set in previous methods
     * 
     * @param sectionType type of the section (e.g. wing/horizontal_tail...)
     * @param offsetFirstPoint y position of the leading edge of the first section
     * @param firstSection cutting distance of the first section
     */
    void generateMissingAirfoilParameter(std::string& sectionType, float offsetFirstPoint, float firstSection);

    void deleteMorphingWingReferences(float widthReferences);
    
private:
    int findTrailingEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    void computeChordLength();

    void computeFlapPosition(Airfoil& foil, std::vector<int> indexMinMax);

    void setName(std::string& sectionType);

    float computeOffsetFromFirstSection(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float offsetFirstPoint);

    void computeRotatedFlapPosition();

    pcl::PointCloud<pcl::PointXYZ>::Ptr foil;
    AirfoilParameter airfoilParameters;
    MorphingWingParameter morphingWingParameters;
};

#endif