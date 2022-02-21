#ifndef POINTCLOUDOPERATOR_H
#define POINTCLOUDOPERATOR_H

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <string>

class PointCloudOperator {
public:
    /**
     * @brief Construct a new Point Cloud Operator object, the given point cloud is aligned and normal vectors are computed
     * 
     * @param inputCloud point cloud which should be operated
     * @param fuselageGreaterThanWing boolean if the fuselage is greater then the wing wide (e.g. jets)
     * @param splittingDistance distance between the wing and tail where the fuselage will be sectioned
     */
    PointCloudOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing, bool completeAricraft);

    /**
     * @brief Construct a new Point Cloud Operator object
     * 
     * @param inputCloud already aligned point cloud with normals
     */
    PointCloudOperator(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);

    /**
     * @brief Construct a new Point Cloud Operator object, the given point cloud is aligned and normal vectors are computed
     * 
     * @param filename filepath and name of the point cloud (PCD) file
     * @param fuselageGreaterThanWing boolean if the fuselage is greater then the wing wide (e.g. jets)
     */
    PointCloudOperator(std::string& filename, bool fuselageGreaterThanWing, bool completeAricraft);

    /**
     * @brief splits the saved point clouds in wing and tails at the given distance
     * 
     * @param wing pointer to the then splitted wing cloud
     * @param horizontalTail pointer to the then splitted horizontal tail cloud
     * @param verticalTail pointer to the then splitted vertical tail cloud
     * @param splittingDistance distance where to split the cloud
     */
    void splitCloudInWingAndTail(pcl::PointCloud<pcl::PointNormal>::Ptr wing, pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail, 
        pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail, float splittingDistance);

    /**
     * @brief Get the Point Cloud With Normals
     * 
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr pointer to the point cloud with computed normals
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr getPointCloudWithNormals();

    /**
     * @brief Get the Point Cloud Without Normals
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr pointer to the point cloud without the computed normals
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudWithoutNormals();

    /**
     * @brief checks if the orientation of the point cloud is right and rotates if needed
     * @param splittingDistance distance betweeen the wing and tail
     * 
     */
    void checkOrientation(float splittingDistance);
private:
    void estimateNormals();
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void aligningPointCloud(bool fuselageGreaterThanWing, bool completeAricraft);
    void downsize();
    float getAngleXZPlane(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled;
    Eigen::Matrix3f rotational_matrix;
};

#endif