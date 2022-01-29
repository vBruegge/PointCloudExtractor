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
};

class Fuselage {
public:
    Fuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr section);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFuselage();

    void setFuselage(pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage_);

    FuselageParameter getFuselageParameter();

    void setAnyFuselageParameter(std::string parameter, float value);

    void setAllFuselageParameter(FuselageParameter parameters_);

    void computeFuselageParameter();
    
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage;
    FuselageParameter parameters;
}
