#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "PointCloudOperator.hpp"

PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool fuselageGreaterThanWing, bool completeAircaft) {
    cloudNoNormals = inputCloud;
    if(cloudNoNormals->points.size()>1000000) {
        downsize();
    }
    aligningPointCloud(fuselageGreaterThanWing, completeAircaft);
    estimateNormals();
}

PointCloudOperator::PointCloudOperator(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud) {
    cloud = inputCloud;
    pcl::copyPointCloud(*inputCloud, *cloudNoNormals);
}

PointCloudOperator::PointCloudOperator(std::string& filename, bool fuselageGreaterThanWing, bool completeAircaft) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (filename, *inputCloud) == -1){
        std::cerr << "Please enter a valid cloud file" << std::endl;
    }
    cloudNoNormals = inputCloud;
    if(cloudNoNormals->points.size()>1000000) {
        downsize();
    }
    else {
        downsampled = inputCloud;
    }
    aligningPointCloud(fuselageGreaterThanWing, completeAircaft);
    estimateNormals();
}

void PointCloudOperator::aligningPointCloud(bool fuselageGreaterThanWing, bool completeAircaft) {
//moment of inertia estimating feature extractor definition

    std::cout << "Calculating OBB and AABB.." << std::endl;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(downsampled);
    feature_extractor.compute();
    std::cout << "Feature extraction computation complete" << std::endl;

    //Variable declerations for feature extractor
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f cog;

    std::cout << "Calculating rotational matrix..." << std::endl;
    //asign values from feature extractor to variables
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getMassCenter(cog);
    std::cout << "Rotational matrix: "<< std::endl << rotational_matrix_OBB << std::endl;

    //draw OBB
    Eigen::Quaternionf rotation_quaternion (rotational_matrix_OBB);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << cog;
    transform.rotate (rotation_quaternion);
    Eigen::Affine3f inverse_transform = transform.inverse();
    pcl::transformPointCloud (*cloudNoNormals, *cloudTransformed, inverse_transform);
    pcl::transformPointCloud (*downsampled, *downsampled, inverse_transform);

    if(fuselageGreaterThanWing == true) {
        const Eigen::Vector3f translationVector (0,0,0);
        Eigen::AngleAxisf rotationQuaternion (M_PI/2, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f transformationSection = Eigen::Affine3f::Identity();
        transformationSection.translation() << translationVector;
        transformationSection.rotate (rotationQuaternion);

        pcl::transformPointCloud (*cloudTransformed, *cloudTransformed, transformationSection);
    }
    if(completeAircaft == true) {
        float angle = getAngleXZPlane(downsampled);
        Eigen::AngleAxisf rotate(-angle, Eigen::Vector3f::UnitX());
        Eigen::Affine3f transformCorrected = Eigen::Affine3f::Identity();
        transformCorrected.rotate(rotate);
        pcl::transformPointCloud (*cloudTransformed, *cloudTransformed, transformCorrected);
        pcl::transformPointCloud (*downsampled, *downsampled, transformCorrected);
    }
    cloudNoNormals = cloudTransformed;
}

float PointCloudOperator::getAngleXZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);
    float cutX = 350;
    float cutY = 275;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-cutX, cutX);
    pass.filter (*filtered);

    pass.setInputCloud (filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-cutY, cutY);
    pass.filter (*filtered);

    float angleMin = -M_PI/2;
    float angleMax = M_PI/2;
    
    float angle = 1.0/180*M_PI;
    float step = angle;
    pcl::PointXYZ minSave, maxSave, min, max;
    pcl::getMinMax3D(*filtered, minSave, maxSave);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::AngleAxisf rotateLeft(angle, Eigen::Vector3f::UnitX());
    Eigen::Affine3f transformCorrected = Eigen::Affine3f::Identity();
    transformCorrected.rotate(rotateLeft);
    Eigen::Affine3f inverse_transform = transformCorrected.inverse();
    pcl::transformPointCloud (*filtered, *tmp, inverse_transform);
    pcl::getMinMax3D(*tmp, min, max);
    bool positiveDirection = true;
    if(abs(min.z-max.z) > abs(minSave.z-maxSave.z)) {
        angle = -1.0/180*M_PI;
        Eigen::AngleAxisf rotateRight(angle, Eigen::Vector3f::UnitX());
        transformCorrected = Eigen::Affine3f::Identity();
        transformCorrected.rotate(rotateRight);
        inverse_transform = transformCorrected.inverse();
        pcl::transformPointCloud (*filtered, *tmp, inverse_transform);
        pcl::getMinMax3D(*tmp, min, max);
        positiveDirection = false;
    }
    do {
        if(positiveDirection == true)
            angle += step;
        else
            angle -= step;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::AngleAxisf rotate(angle, Eigen::Vector3f::UnitX());
        transformCorrected = Eigen::Affine3f::Identity();
        transformCorrected.rotate(rotate);
        inverse_transform = transformCorrected.inverse();
        pcl::transformPointCloud (*filtered, *tmp, inverse_transform);
        pcl::getMinMax3D(*tmp, min, max);

        if(abs(min.z-max.z) < abs(minSave.z-maxSave.z)) {
            minSave = min;
            maxSave = max;
        }
        else {
            step /= 2;
            positiveDirection = !positiveDirection;
        }

    } while(step > 5e-4);
    
    return angle;
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

  std::cout << "Normal calculation complete...\n";

  cloud = inputCloudWithNormals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudOperator::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    std::cout << "Calculate normals of point cloud...\n";

  //process normals

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (inputCloud);

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
  pcl::concatenateFields(*inputCloud, *cloudNormals, *inputCloudWithNormals);


  //pcl::io::savePCDFile("mlsNormals.txt", *inputCloudWithNormals);
  std::cout << "Normal calculation complete...\n";

  return inputCloudWithNormals;
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

void PointCloudOperator::downsize() {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr smallerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (cloudNoNormals);
    sor.setLeafSize (5, 5, 5);
    sor.filter (*smallerCloud);
    downsampled = smallerCloud;
}