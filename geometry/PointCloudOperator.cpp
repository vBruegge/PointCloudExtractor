#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "PointCloudOperator.hpp"

PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointXYZ> inputCloud) {
    aligningPointCloud(inputCloud);
    estimateNormals(inputCloud);
}
PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointNormal> inputCloud) {
    aligningPointCloud(inputCloud);
    estimateNormals(inputCloud);
}
PointCloudOperator::PointCloudOperator(std::string filename) {
    if (pcl::io::loadPCDFile (filename, *cloud) == -1){
        std::cerr << "Please enter a valid cloud file" << std::endl;
        return(-1);
    }
    aligningPointCloud(inputCloud);
    estimateNormals(inputCloud);
}

void aligningPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing) {
//moment of inertia estimating feature extractor definition

  std::cout << "Calculating OBB and AABB.." << std::endl;
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  std::cout << "Feature extraction computation complete" << std::endl;

  //Variable declerations for feature extractor
  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  float d_min,d_max;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  std::cout << "Calculating rotational matrix..." << std::endl;
  //asign values from feature extractor to variables
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);
  std::cout << "Rotational matrix: "<< std::endl << rotational_matrix_OBB << std::endl;

  //draw OBB
  Eigen::Vector3f transformation_vector (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf rotation_quaternion (rotational_matrix_OBB);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << transformation_vector;
  transform.rotate (rotation_quaternion);
  Eigen::Affine3f inverse_transform = transform.inverse();
  pcl::transformPointCloud (*cloud, *cloudTransformed, inverse_transform);

  if(fuselageGreaterThanWing == 1) {
    const Eigen::Vector3f   translationVector (0,0,0);
    Eigen::AngleAxisf rotationQuaternion (M_PI/2, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transformationSection = Eigen::Affine3f::Identity();
    transformationSection.translation() << translationVector;
    transformationSection.rotate (rotationQuaternion);

    pcl::transformPointCloud (*cloudTransformed, *cloudTransformed, transformationSection);
  }
}
    void estimateNormals();