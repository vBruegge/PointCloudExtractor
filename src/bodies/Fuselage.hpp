#ifndef FUSELAGE_H
#define FUSELAGE_H

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

#include <string>

class FuselageParameter {
public:
    std::string name;
    float cuttingDistance;
    double disX;
    double disY;
    double xM;
    double yM;
    double epsilon = 1.0;
    enum parameterType{CuttingDistance, DisX, DisY, XM, YM, Epsilon};
};

class Fuselage {
public:
    /**
     * @brief Construct a new Fuselage object
     * 
     * @param section new fuselage section
     */
    Fuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr section);
    Fuselage() = default;

    /**
     * @brief Get the Fuselage object
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr pointer to the fuselage point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFuselage();

    /**
     * @brief replace section in the fuselage object
     * 
     * @param fuselage_ new section
     */
    void setFuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage_);

    /**
     * @brief Get the Fuselage Parameter object
     * 
     * @return FuselageParameter saved fuselage parameters
     */
    FuselageParameter getFuselageParameter();

    /**
     * @brief Set one of the Fuselage parameters
     * 
     * @param type type of parameter (e.g. CuttingDistance, DisX, DisY...)
     * @param value value of parameter
     */
    void setAnyFuselageParameter(FuselageParameter::parameterType type, float value);

    /**
     * @brief replace the fuselage Parameters
     * 
     * @param parameters_ new fuselage parameters
     */
    void setAllFuselageParameter(FuselageParameter parameters_);

    /**
     * @brief computes geometric fuselage parameters of a ellipse
     * 
     */
    void computeFuselageParameter();

    /**
     * @brief Sets the name of the fuselage section
     * 
     * @param name 
     */
    void setName(std::string& name);
    
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage;
    FuselageParameter parameters;
};

#endif
