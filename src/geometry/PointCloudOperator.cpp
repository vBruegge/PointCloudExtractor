#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "PointCloudOperator.hpp"

PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing) {
    cloudNoNormals = inputCloud;
    aligningPointCloud(fuselageGreaterThanWing);
    estimateNormals();
}

PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud) {
    cloud = inputCloud;
}

PointCloudOperator::PointCloudOperator(std::string& filename, bool fuselageGreaterThanWing) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (filename, *inputCloud) == -1){
        std::cerr << "Please enter a valid cloud file" << std::endl;
    }
    cloudNoNormals = inputCloud;
    aligningPointCloud(fuselageGreaterThanWing);
    estimateNormals();
}

void PointCloudOperator::aligningPointCloud(bool fuselageGreaterThanWing) {
//moment of inertia estimating feature extractor definition

  std::cout << "Calculating OBB and AABB.." << std::endl;
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloudNoNormals);
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << transformation_vector;
  transform.rotate (rotation_quaternion);
  Eigen::Affine3f inverse_transform = transform.inverse();
  pcl::transformPointCloud (*cloudNoNormals, *cloudTransformed, inverse_transform);

  if(fuselageGreaterThanWing == 1) {
    const Eigen::Vector3f   translationVector (0,0,0);
    Eigen::AngleAxisf rotationQuaternion (M_PI/2, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transformationSection = Eigen::Affine3f::Identity();
    transformationSection.translation() << translationVector;
    transformationSection.rotate (rotationQuaternion);

    pcl::transformPointCloud (*cloudTransformed, *cloudTransformed, transformationSection);
  }
  cloudNoNormals = cloudTransformed;
}

void PointCloudOperator::estimateNormals() {
    std::cout << "Calculate normals of point cloud...\n";

  //process normals

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloudNoNormals);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 2.5mm
  ne.setRadiusSearch (2.5);

  // Compute the features
  ne.compute (*cloudNormals);
  pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloudNoNormals, *cloudNormals, *inputCloudWithNormals);


  //pcl::io::savePCDFile("mlsNormals.txt", *inputCloudWithNormals);
  std::cout << "Normal calculation complete...\n";

  cloud = inputCloudWithNormals;
}

void PointCloudOperator::splitCloudInWingAndTail(pcl::PointCloud<pcl::PointNormal>::Ptr wing,
    pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail, pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail,
    float splittingDistance) {

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudShort(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-FLT_MAX, splittingDistance);
    pass.filter (*cloudShort);
    
    pcl::PointNormal minOrg, maxOrg, minShort, maxShort;
    pcl::getMinMax3D(*cloud, minOrg, maxOrg);
    pcl::getMinMax3D(*cloudShort, minShort, maxShort);

    bool tail = false;
    if(abs(minOrg.x-maxOrg.x-1) > abs(minShort.x-maxShort.x)) {
        pcl::copyPointCloud(*cloudShort, *verticalTail);
        tail = true;
    }
    //split uav in half to speed up the calculation
    pass.setInputCloud (cloudShort);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, FLT_MAX);
    if(tail == true)
        pass.filter (*horizontalTail);
    else
        pass.filter(*wing);
    

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (splittingDistance, FLT_MAX);
    pass.filter (*cloudShort);

    if(tail == false) {
        pcl::copyPointCloud(*cloudShort, *verticalTail);
    }

    //split uav in half to speed up the calculation
    pass.setInputCloud (cloudShort);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, FLT_MAX);
    if(tail == false)
        pass.filter (*horizontalTail);
    else
        pass.filter(*wing);
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudOperator::getPointCloudWithNormals() {
    return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudOperator::getPointCloudWithoutNormals() {
    return cloudNoNormals;
}