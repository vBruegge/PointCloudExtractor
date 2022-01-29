#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

#include <string>

class PointCloudOperator {
public:
    PointCloudOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    PointCloudOperator(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);
    PointCloudOperator(std::string filename);
private:
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    void estimateNormals(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud);
    void aligningPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing);
    void aligningPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, bool fuselageGreaterThanWing);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
}