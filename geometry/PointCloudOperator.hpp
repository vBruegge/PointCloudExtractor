#ifndef POINTCLOUDOPERATOR_H
#define POINTCLOUDOPERATOR_H

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <string>

class PointCloudOperator {
public:
    PointCloudOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing);
    PointCloudOperator(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);
    PointCloudOperator(std::string& filename, bool fuselageGreaterThanWing);
    void splitCloudInWingAndTail(pcl::PointCloud<pcl::PointNormal>::Ptr wing, pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail, 
        pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail, float splittingDistance);
    pcl::PointCloud<pcl::PointNormal>::Ptr getPointCloudWithNormals();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudWithoutNormals();
private:
    void estimateNormals();
    void aligningPointCloud(bool fuselageGreaterThanWing);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoNormals;
};

#endif