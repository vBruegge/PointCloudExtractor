#include "GeometryExtractor.hpp"

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>

#include <gsl/gsl_multifit.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>

#include <pcl/io/pcd_io.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>

Airfoil GeometryExtractor::sectioningCloudX(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded){
  // function generates a plane section at a defined distance from origin. It uses the normal vectors of the mesh to create a plane.

  //filter small section near the cuttingDistance for surface normal calculation
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PassThrough<pcl::PointNormal> pass;
  pass.setInputCloud (inputCloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (cuttingDistance - 0.5, cuttingDistance + 0.5);
  pass.filter (*cloudPassThrough);

  //pcl::io::savePCDFile("filter1.pcd", *cloudPassThrough);
 
  Eigen::Vector3f surfaceNormal (0.0, 0.0, 0.0);
  //generate an average of all normals near the cuttingDistance
  for (std::size_t i = 0; i < cloudPassThrough->size(); i++) {
    Eigen::Vector3f normal(cloudPassThrough->points[i].normal_x, cloudPassThrough->points[i].normal_y, cloudPassThrough->points[i].normal_z);
    if (normal[2] > 0.0) 
      surfaceNormal += normal;
    else
      surfaceNormal -= normal;
  }
  
  //search after a Point of the filtered section in the inputCloud using nearest neighbor search
  //the point tells the new cuttingDistance after rotating the foil
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud (inputCloud);

  std::vector<int> pointIndexSearch(1);
  std::vector<float> pointDistanceSearch(1);
  kdtree.nearestKSearch (cloudPassThrough->points[0], 1, pointIndexSearch, pointDistanceSearch);

  //creating normal of cuttingPlane, plane parallel to y-axis and surface normal
  Eigen::Vector3f cuttingPlaneNormal;
  cuttingPlaneNormal = surfaceNormal.cross(Eigen::Vector3f::UnitY());

  //angle between CuttingPlane and XY Plane for rotating cloud
  float angleCuttingPlane = pcl::getAngle3D(cuttingPlaneNormal, Eigen::Vector3f::UnitZ());
  if(abs(angleCuttingPlane - M_PI/2) < (float) 15/180*M_PI)
    angleCuttingPlane -= M_PI/2;
  else if (angleCuttingPlane > M_PI/2)
    angleCuttingPlane -= M_PI;

  //Definition of rotation matrix with angle of the cuttingPlane
  const Eigen::Vector3f   translationVectorCuttingPlane (0,0,0);
  const Eigen::AngleAxisf rotationQuaternionCuttingPlane (angleCuttingPlane, Eigen::Vector3f::UnitY());
  Eigen::Affine3f transformationCuttingPlaneNormal = Eigen::Affine3f::Identity();
  transformationCuttingPlaneNormal.translation() << translationVectorCuttingPlane;
  transformationCuttingPlaneNormal.rotate (rotationQuaternionCuttingPlane);

  pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudTransformed (new pcl::PointCloud<pcl::PointNormal>);

  //Transform cloud into a system parallel to the cutting plane
  Eigen::Affine3f transform = transformationCuttingPlaneNormal.inverse();
  pcl::transformPointCloud (*inputCloud, *inputCloudTransformed, transform);

  //plane definition
  pcl::ModelCoefficients::Ptr sectionCoefficients (new pcl::ModelCoefficients ());
  sectionCoefficients->values.resize (4);
  sectionCoefficients->values[0] = cuttingPlaneNormal[0];
  sectionCoefficients->values[1] = cuttingPlaneNormal[1];
  sectionCoefficients->values[2] = cuttingPlaneNormal[2];
  float d = inputCloudTransformed->points[pointIndexSearch[0]].x; //new cutting Distance -> point of filtered cloud, now rotated
  sectionCoefficients->values[3] = d;

  //Filter out points above and below a plane using pass through filtering
  float projDistance = 0.1;
  int k = 1;
  do {
    pass.setInputCloud (inputCloudTransformed);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (sectionCoefficients->values[3]-projDistance*k, sectionCoefficients->values[3]+projDistance*k);
    pass.filter (*cloudPassThrough);
    k++;
  } while(cloudPassThrough->size() < 750);

  // project the foil
  pcl::ModelCoefficients::Ptr projectionPlane (new pcl::ModelCoefficients ());
  projectionPlane->values.resize (4);
  projectionPlane->values[0] = 1;
  projectionPlane->values[1] = 0;
  projectionPlane->values[2] = 0;
  
  pcl::ProjectInliers<pcl::PointNormal> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloudPassThrough);
  proj.setModelCoefficients (projectionPlane);
  proj.filter (*cloudPassThrough);


  //pcl::io::savePCDFile("filter2.txt", *cloudPassThrough);

  pcl::transformPointCloud (*cloudPassThrough, *cloudPassThrough, transform);

  Airfoil foil;
  if(flapRotationNeeded == true)
    //derotate flaps
  foil =  derotateFlap(cloudPassThrough, abs(angleCuttingPlane));
  else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoNormals (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloudPassThrough, *cloudNoNormals);
    AirfoilParameter parameters;
    parameters.dihedral = abs(angleCuttingPlane)*180/M_PI;
    parameters.flapPosition = 0.0;
    foil = Airfoil(cloudNoNormals, parameters);
  }
  foil.setAnyAirfoilParameter(AirfoilParameter::parameterType::CuttingDistance, cuttingDistance);

  //pcl::io::savePCDFile("foil_new.txt", *foil->section);
  return foil;

}

Fuselage GeometryExtractor::sectioningCloudY(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float cuttingDistance){
  //
  //This function creates a section at a defined distance from the origin and a defined angle
  //

  Eigen::Vector3f cuttingPlaneNormal(0.0,1.0,0.0);

  //Plane definition in form ax+by+cz=d in normal form (x-p)*n=0
  pcl::ModelCoefficients::Ptr sectionCoefficients (new pcl::ModelCoefficients ());
  sectionCoefficients->values.resize (4);
  sectionCoefficients->values[0] = cuttingPlaneNormal[0];
  sectionCoefficients->values[1] = cuttingPlaneNormal[1];
  sectionCoefficients->values[2] = cuttingPlaneNormal[2];
  sectionCoefficients->values[3] = cuttingDistance;

  pcl::PointCloud<pcl::PointXYZ>::Ptr section (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointXYZ>);

  //Filter out points above and below a plane using pass through filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  int k = 1;
  do {
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (sectionCoefficients->values[3]-0.1*k, sectionCoefficients->values[3]+0.1*k);
    pass.filter (*cloudPassThrough);
    k++;
  } while(cloudPassThrough->size() < 200);

  // Create the projection  object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloudPassThrough);
  proj.setModelCoefficients (sectionCoefficients);
  proj.filter (*section);

  Fuselage fuselageSection;
  fuselageSection.setFuselage(section);
  fuselageSection.computeFuselageParameter();
  fuselageSection.setAnyFuselageParameter(FuselageParameter::parameterType::CuttingDistance, cuttingDistance);
  std::string name = "../Results/fuselage_" + std::to_string(cuttingDistance) + "mm.dat";
  fuselageSection.setName(name);

  return fuselageSection;
}

Airfoil GeometryExtractor::sectioningCloudZ(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float cuttingDistance, bool flapRotationNeeded){
  //
  //This function creates a section at a defined distance from the origin and a defined angle in z-direction
  //

  Eigen::Vector3f cuttingPlaneNormal(0.0,0.0,1.0);

  //Plane definition in form ax+by+cz=d in normal form (x-p)*n=0
  pcl::ModelCoefficients::Ptr sectionCoefficients (new pcl::ModelCoefficients ());
  sectionCoefficients->values.resize (4);
  sectionCoefficients->values[0] = cuttingPlaneNormal[0];
  sectionCoefficients->values[1] = cuttingPlaneNormal[1];
  sectionCoefficients->values[2] = cuttingPlaneNormal[2];
  sectionCoefficients->values[3] = cuttingDistance;

  pcl::PointCloud<pcl::PointNormal>::Ptr section (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointNormal>);

  //Filter out points above and below a plane using pass through filtering
  pcl::PassThrough<pcl::PointNormal> pass;
  int k = 1;
  do {
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (sectionCoefficients->values[3]-0.1*k, sectionCoefficients->values[3]+0.1*k);
    pass.filter (*cloudPassThrough);
    k++;
  } while(cloudPassThrough->size() < 500);

  // Create the projection  object
  pcl::ProjectInliers<pcl::PointNormal> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloudPassThrough);
  proj.setModelCoefficients (sectionCoefficients);
  proj.filter (*section);

  //rotate by 90 degrees for flap derotation
  const Eigen::Vector3f   translationVector (0,0,0);
  Eigen::AngleAxisf rotationQuaternion (M_PI/2, Eigen::Vector3f::UnitY());
  Eigen::Affine3f transformationSection = Eigen::Affine3f::Identity();
  transformationSection.translation() << translationVector;
  transformationSection.rotate (rotationQuaternion);

  pcl::PointCloud<pcl::PointNormal>::Ptr sectionRotated(new pcl::PointCloud<pcl::PointNormal>);
  pcl::transformPointCloud (*section, *sectionRotated, transformationSection);

  Airfoil foil;
  if(flapRotationNeeded == true) {
    //derotate flaps
    foil = derotateFlap(sectionRotated, 0.0);
    foil.setAnyAirfoilParameter(AirfoilParameter::parameterType::Dihedral, 90.0);
  }
  else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoNormals(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*sectionRotated, *cloudNoNormals);
    AirfoilParameter parameter;
    parameter.dihedral = 90.0;
    parameter.flapPosition = 0.0;
    foil = Airfoil(cloudNoNormals, parameter);
  }
  foil.setAnyAirfoilParameter(AirfoilParameter::parameterType::CuttingDistance, cuttingDistance);
  //pcl::io::savePCDFile("foil_new.txt", *flap->section);
  return foil;
}

void GeometryExtractor::derotateSection(Airfoil& foil){
//calculates the twist of an airfoil to the xy-plane

  float rotationAngle;

  std::vector<int> indexMinMax = foil.findLeadingTrailingEdge(foil.getFoil()); //find index of min and max of the input cloud

  //vector from trailingEdge to leadingEdge
  Eigen::Vector3f centerLineVector;
  centerLineVector[0] = 0.0;
  centerLineVector[1] = foil.getFoil()->points[indexMinMax[0]].y-foil.getFoil()->points[indexMinMax[1]].y; 
  centerLineVector[2] = foil.getFoil()->points[indexMinMax[0]].z-foil.getFoil()->points[indexMinMax[1]].z;
  rotationAngle = pcl::getAngle3D(Eigen::Vector3f::UnitY(), centerLineVector, false);
  if(rotationAngle > M_PI/2)
    rotationAngle = M_PI - rotationAngle;

  //rotate both clockwise and counterclockwise -> unknown direction of deflection
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedInverse (new pcl::PointCloud<pcl::PointXYZ>);
  
  const Eigen::Vector3f translationVector (0,0,0);
  const Eigen::AngleAxisf rotationQuaternion (rotationAngle, Eigen::Vector3f::UnitX());
  Eigen::Affine3f transformationAffine = Eigen::Affine3f::Identity();
  transformationAffine.translation() << translationVector;
  transformationAffine.rotate (rotationQuaternion);

  pcl::transformPointCloud (*foil.getFoil(), *rotated, transformationAffine);

  const Eigen::AngleAxisf rotationQuaternionInverse (-rotationAngle, Eigen::Vector3f::UnitX());
  Eigen::Affine3f transformationAffine2 = Eigen::Affine3f::Identity();
  transformationAffine2.translation() << translationVector;
  transformationAffine2.rotate (rotationQuaternionInverse);

  pcl::transformPointCloud (*foil.getFoil(), *rotatedInverse, transformationAffine2);

  //choose right rotation -> difference between the z-distance
  pcl::PointXYZ minR, maxR, minRI, maxRI;
  pcl::getMinMax3D(*rotated, minR, maxR);
  pcl::getMinMax3D(*rotatedInverse, minRI, maxRI);
  if(maxR.z-minR.z < maxRI.z - minRI.z) {
    foil.setFoil(rotated);
  }
  else
  {
    foil.setFoil(rotatedInverse);
  }
  foil.setAnyAirfoilParameter(AirfoilParameter::parameterType::Twist, rotationAngle*180.0/M_PI);
}

void GeometryExtractor::translateSection(Airfoil& foil){
  //translates center of the Point Cloud to Point Zero

  std::vector<int> indexMinMax = foil.findLeadingTrailingEdge(foil.getFoil());
  pcl::PointXYZ minPt = foil.getFoil()->points[indexMinMax[0]];
  pcl::PointXYZ maxPt = foil.getFoil()->points[indexMinMax[1]];

  const Eigen::Vector3f translationVector (-(maxPt.x+minPt.x)/2, -(maxPt.y + minPt.y)/2, -(maxPt.z + minPt.z)/2);

  Eigen::Affine3f transformationAffine = Eigen::Affine3f::Identity();
  transformationAffine.translation() << translationVector;

  pcl::PointCloud<pcl::PointXYZ>::Ptr translated (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*foil.getFoil(), *translated, transformationAffine);
  foil.setFoil(translated);

  //shift of the position of the flap
  AirfoilParameter parameters = foil.getAirfoilParameter();
  parameters.flapPosition = parameters.flapPosition + translationVector[1];
  foil.setAllAirfoilParameter(parameters);
}

int GeometryExtractor::getIndexFlapPosition(pcl::PointCloud <pcl::PointNormal>::Ptr inputCloud, std::vector<int>& indexLeadingTrailingEdge) {
  //find flap position in foil
  //output: x coordinate of the flap, rotation angle and index of the flap position
  int indexLeadingEdge = indexLeadingTrailingEdge[0];
  int indexTrailingEdge = indexLeadingTrailingEdge[1];
  pcl::PointNormal searchPoint;
  searchPoint.x = inputCloud->points[indexLeadingEdge].x;
  searchPoint.z = (inputCloud->points[indexLeadingEdge].z+inputCloud->points[indexTrailingEdge].z)/2;
  searchPoint.y = (inputCloud->points[indexLeadingEdge].y+inputCloud->points[indexTrailingEdge].y)/2;  

  //pcl::io::savePCDFile("foil.txt", *inputCloud);

  //calculate approximal normal of the foil from the mid of the foil
  Eigen::Vector3f normalFoil =  computeAverageNormalOfFoil(inputCloud, searchPoint, indexTrailingEdge, 30, 10, -1);

//calculate normal of the flap and its position with nearest neighbor search
  //starting point: 9/10 distance between leading edge and trailing edge
  searchPoint.y = inputCloud->points[indexTrailingEdge].y-(inputCloud->points[indexTrailingEdge].y-inputCloud->points[indexLeadingEdge].y)/8.5;
  searchPoint.z = (inputCloud->points[indexTrailingEdge].z);
  
  //nearest neighbor search for iterating through neighboring points in none-ordered point cloud
  //setup of the search tree
  int n = 40;
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud (inputCloud);

  std::vector<int> pointIndexSearch(n);
  std::vector<float> pointDistanceSearch(n);
  float angle;
  int indexFlapPosition = -1;
  pcl::PointNormal save = searchPoint;
  float saveAngle = 0.0;

  Eigen::Vector3f normalFlap(0,0,0);
  float trailingEdge = inputCloud->points[indexTrailingEdge].y;

  //perform search
  kdtree.nearestKSearch (searchPoint, 1, pointIndexSearch, pointDistanceSearch);
  searchPoint = inputCloud->points[pointIndexSearch[0]];
  do {
    kdtree.nearestKSearch (searchPoint, n, pointIndexSearch, pointDistanceSearch);
    save = searchPoint;
    int i;
    float angle;
    //find nearest point in direction to the leading edge
    for(i = 1; i < n; i++) {
      if(abs(inputCloud->points[pointIndexSearch[i]].y-trailingEdge) > abs(searchPoint.y-trailingEdge)) {
        searchPoint = inputCloud->points[pointIndexSearch[i]];
        normalFlap += Eigen::Vector3f(inputCloud->points[pointIndexSearch[i]].normal_x, inputCloud->points[pointIndexSearch[i]].normal_y, inputCloud->points[pointIndexSearch[i]].normal_z);
        angle = pcl::getAngle3D(Eigen::Vector3f(save.normal_x, save.normal_y, save.normal_z),Eigen::Vector3f(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z));
        //calculate angle betweeen the surface normal of the flap and the foil if there is a discontinuity
        if(angle > 0.015 && (indexFlapPosition == -1 || abs(saveAngle-angle) < 0.1)) {
          saveAngle = angle;
          indexFlapPosition = pointIndexSearch[i];
        }
        break;
      }      
    }
    //no point nearer to the leading edge found -> interrupt
    if(i == n) {
      break;
    }
  } while (abs(searchPoint.y-trailingEdge) < abs(inputCloud->points[indexTrailingEdge].y-inputCloud->points[indexLeadingEdge].y)*2/3);

  return indexFlapPosition;
}

Airfoil GeometryExtractor::derotateFlap (pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, float dihedral) {
  //derotate flap

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoNormals (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*inputCloud, *cloudNoNormals);
  Airfoil foilTmp;
  foilTmp.setFoil(cloudNoNormals);
  //find position of the trailingEdge
  std::vector<int> indexLeadingTrailingEdge = foilTmp.findLeadingTrailingEdge(cloudNoNormals);
  float trailingEdge =  cloudNoNormals->points[indexLeadingTrailingEdge[1]].y;

  int indexFlapPosition = getIndexFlapPosition(inputCloud, indexLeadingTrailingEdge);
  pcl::PointCloud<pcl::PointNormal>::Ptr foil(new pcl::PointCloud<pcl::PointNormal>);
  float flapDistance = inputCloud->points[indexFlapPosition].y;

  if(indexFlapPosition != -1) {
    //split foil at the found flap position
    pcl::PointCloud<pcl::PointNormal>::Ptr withoutFlap (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PassThrough<pcl::PointNormal> pass;
    pcl::PointCloud<pcl::PointNormal>::Ptr flap (new pcl::PointCloud<pcl::PointNormal>);
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("y");
    if(flapDistance < trailingEdge) {
      pass.setFilterLimits (flapDistance, FLT_MAX);
      pass.filter (*flap);

      pass.setFilterLimits (-FLT_MAX, flapDistance);
      pass.filter (*withoutFlap);
    }
    else {
      pass.setFilterLimits (flapDistance, FLT_MAX);
      pass.filter (*withoutFlap);

      pass.setFilterLimits (-FLT_MAX, flapDistance);
      pass.filter (*flap);
    }

    //pcl::io::savePCDFile("flap.txt", *flap);
    //pcl::io::savePCDFile("without_flap.txt", *withoutFlap);

    //calculate approximal normal of the foil from the mid of the foil
    pcl::PointNormal searchPoint = inputCloud->points[indexFlapPosition];
    Eigen::Vector3f normalFoil =  computeAverageNormalOfFoil(inputCloud, searchPoint, indexLeadingTrailingEdge[1], 30, 10, 1);
    Eigen::Vector3f normalFlap =  computeAverageNormalOfFoil(inputCloud, searchPoint, indexLeadingTrailingEdge[1], 30, 10, -1);
    float angleFlap = pcl::getAngle3D(normalFoil, normalFlap);

    if(abs(angleFlap - M_PI/2) < (float) 15/180*M_PI)
      angleFlap = angleFlap - M_PI/2;
    else if (angleFlap > M_PI/2)
      angleFlap = M_PI - angleFlap;

    //find flap position in input cloud
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud (inputCloud);
    std::vector<int> pointIndexSearch(5);
    std::vector<float> pointDistanceSearch(5);
    kdtree.nearestKSearch (inputCloud->points[indexFlapPosition], 5, pointIndexSearch, pointDistanceSearch);

    int indexFlap = pointIndexSearch[0];
    for(int i = 1; i < 5; i++) {
      if(abs(inputCloud->points[pointIndexSearch[i]].y - trailingEdge) > abs(inputCloud->points[indexFlapPosition].y - trailingEdge)) {
        indexFlap = pointIndexSearch[i];
        break;
      }
    }

    pcl::PointNormal oldFlapPosition = inputCloud->points[indexFlap];

    //find point index of the found flap position
    kdtree.setInputCloud (flap);
    kdtree.nearestKSearch (inputCloud->points[indexFlapPosition], 1, pointIndexSearch, pointDistanceSearch);

    //derotate flap both clockwise and counterclockwise -> unknown direction of deflection
    Eigen::Vector3f translationVector(0,0,0);
    Eigen::AngleAxisf rotationQuaternion (angleFlap, Eigen::Vector3f::UnitX());
    Eigen::Affine3f transformationAffine = Eigen::Affine3f::Identity();
    transformationAffine.translation() << translationVector;
    transformationAffine.rotate (rotationQuaternion);

    pcl::PointCloud<pcl::PointNormal>::Ptr flapRotated (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloud (*flap, *flapRotated, transformationAffine);

    //pcl::io::savePCDFile("flap_rotated.txt", *flapRotated);
    
    rotationQuaternion =  Eigen::AngleAxisf(-angleFlap, Eigen::Vector3f::UnitX());
    transformationAffine = Eigen::Affine3f::Identity();
    transformationAffine.translation() << translationVector;
    transformationAffine.rotate (rotationQuaternion);

    pcl::PointCloud<pcl::PointNormal>::Ptr flapInverse (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloud (*flap, *flapInverse, transformationAffine);

    //pcl::io::savePCDFile("flap_rotated_inverse.txt", *flapInverse);

  // decide which rotation

    pcl::copyPointCloud(*flapRotated, *cloudNoNormals);
    std::vector<int> indexMinMax = foilTmp.findLeadingTrailingEdge(cloudNoNormals);
    //vector from flap position to trailing edge
    Eigen::Vector3f flapSurfaceVector;
    if(abs(flapRotated->points[pointIndexSearch[0]].y-flapRotated->points[indexMinMax[0]].y) > abs(flapRotated->points[pointIndexSearch[0]].y-flapRotated->points[indexMinMax[1]].y)) {
      flapSurfaceVector[0] = flapRotated->points[pointIndexSearch[0]].x-flapRotated->points[indexMinMax[0]].x;
      flapSurfaceVector[1] = flapRotated->points[pointIndexSearch[0]].y-flapRotated->points[indexMinMax[0]].y;
      flapSurfaceVector[2] = flapRotated->points[pointIndexSearch[0]].z-flapRotated->points[indexMinMax[0]].z;
    }
    else {
      flapSurfaceVector[0] = flapRotated->points[pointIndexSearch[0]].x-flapRotated->points[indexMinMax[1]].x;
      flapSurfaceVector[1] = flapRotated->points[pointIndexSearch[0]].y-flapRotated->points[indexMinMax[1]].y;
      flapSurfaceVector[2] = flapRotated->points[pointIndexSearch[0]].z-flapRotated->points[indexMinMax[1]].z;
    }
    //calculate normal of flap
    Eigen::Vector3f normalFlapRotated = flapSurfaceVector.cross(Eigen::Vector3f::UnitX());

    float angle = pcl::getAngle3D(normalFoil, normalFlapRotated);
    if(abs(angle - M_PI/2) < 0.05)
      normalFlapRotated = flapSurfaceVector.cross(Eigen::Vector3f::UnitZ());

    //decide which rotation
    if(abs(angle-dihedral) < 0.1 || abs(abs(angle-M_PI)-dihedral) < 0.15)
      flap = flapRotated;
    else
      flap = flapInverse;

    //translate flap to original position
    pcl::PointXYZ flapPos = getOriginalPosition(flap, flap->points[pointIndexSearch[0]]);
    pcl::PointXYZ foilPos = getOriginalPosition(withoutFlap, oldFlapPosition);
    translationVector = Eigen::Vector3f(foilPos.x-flapPos.x, foilPos.y-flapPos.y, foilPos.z-flapPos.z);
    transformationAffine = Eigen::Affine3f::Identity();
    transformationAffine.translation() << translationVector;
    pcl::transformPointCloud (*flap, *flap, transformationAffine);

    pass.setInputCloud (flap);
    pass.setFilterFieldName ("y");
    if(flapDistance < trailingEdge)
      pass.setFilterLimits (foilPos.y, FLT_MAX);
    else
      pass.setFilterLimits (-FLT_MAX, foilPos.y);
    pass.filter (*flap);  

    //combine foil
    pcl::concatenate(*withoutFlap, *flap, *foil);
  }
  else {
    foil = inputCloud;
  }
    
  //delete normals
  pcl::copyPointCloud(*foil, *cloudNoNormals);
  //pcl::io::savePCDFile("flap_derotated.txt", *cloudNoNormals);

  AirfoilParameter parameters;
  parameters.dihedral = dihedral;
  parameters.flapPosition = flapDistance;
  Airfoil extractedFoil(cloudNoNormals, parameters);
  return extractedFoil;
}

void GeometryExtractor::deleteTrailingEdge(Airfoil& foil, int indexTrailingEdge, float distanceFromTrailingEdge) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = foil.getFoil();
  pcl::PointXYZ trailingEdge = inputCloud->points[indexTrailingEdge];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (inputCloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (trailingEdge.y-2, trailingEdge.y+2);
  pass.filter (*cloudPassThrough);

  pcl::PointXYZ min, max;
  pcl::getMinMax3D(*cloudPassThrough, min, max);
  float trailingEdgeWidth = abs(max.z-min.z);

  pcl::getMinMax3D(*inputCloud, min, max);
  if(abs(trailingEdge.y-max.y) < abs(trailingEdge.y-min.y)) {
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (FLT_MIN, trailingEdge.y-distanceFromTrailingEdge);
    pass.filter (*inputCloud);
  }
  else {
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (trailingEdge.y+distanceFromTrailingEdge, FLT_MAX);
    pass.filter (*inputCloud);
  }

  pcl::io::savePCDFile("deletedTrailingEdge.txt", *inputCloud);
  foil.setFoil(inputCloud);
  foil.setAnyAirfoilParameter(AirfoilParameter::parameterType::TrailingEdgeWidth, trailingEdgeWidth);
}

Eigen::Vector3f GeometryExtractor::computeAverageNormalOfFoil(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint, int indexTrailingEdge, int iterate, int n, int direction) {
  //calculates the normal from a point to/from the leading edge with nearest neighbor search
  //direction = -1: -> to the trailing edge
  //direction = 1: -> to the leading edge
 
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud (inputCloud);

  float trailingEdge = inputCloud->points[indexTrailingEdge].y;
  std::vector<int> pointIndexSearch(n);
  std::vector<float> pointDistanceSearch(n);
  Eigen::Vector3f normal (0,0,0);

  kdtree.nearestKSearch (searchPoint, 1, pointIndexSearch, pointDistanceSearch);
  searchPoint = inputCloud->points[pointIndexSearch[0]];

  for(int i = 0; i < iterate; i++) {
    kdtree.nearestKSearch (searchPoint, n, pointIndexSearch, pointDistanceSearch);
    for(int j = 1; j < n; j++) {
      if(direction < 0 && abs(inputCloud->points[pointIndexSearch[j]].y-trailingEdge) < abs(searchPoint.y-trailingEdge)) {
        searchPoint = inputCloud->points[pointIndexSearch[j]];
        normal+= Eigen::Vector3f(inputCloud->points[pointIndexSearch[j]].normal_x, inputCloud->points[pointIndexSearch[j]].normal_y, inputCloud->points[pointIndexSearch[j]].normal_z);
        break;
      }
      else if(direction > 0 && abs(inputCloud->points[pointIndexSearch[j]].y-trailingEdge) > abs(searchPoint.y-trailingEdge)) {
        searchPoint = inputCloud->points[pointIndexSearch[j]];
        normal+= Eigen::Vector3f(inputCloud->points[pointIndexSearch[j]].normal_x, inputCloud->points[pointIndexSearch[j]].normal_y, inputCloud->points[pointIndexSearch[j]].normal_z);
        break;
      }
    }
  }
  return normal;

}

pcl::PointXYZ GeometryExtractor::getOriginalPosition(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloud, pcl::PointNormal searchPoint) {
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud (inputCloud); 
    std::vector<int> pointIdxSearch;
    std::vector<float> pointDistanceSearch;
    kdtree.radiusSearch (searchPoint, 5, pointIdxSearch, pointDistanceSearch);
    int size = pointIdxSearch.size();
    pointIdxSearch.resize(size+10);
    pointDistanceSearch.resize(size+10);
    kdtree.nearestKSearch (searchPoint, size+10, pointIdxSearch, pointDistanceSearch);
    std::vector<double> x, y;
    for(int i = size; i  < pointIdxSearch.size(); i++) {
      x.push_back(inputCloud->points[pointIdxSearch[i]].y);
      y.push_back(inputCloud->points[pointIdxSearch[i]].z);
    }
    int degree = 3;
    double coeff[degree+1];
    getPolynomialCoeff(x, y, coeff, degree);
    double zPos = 0.0;
    for(int i = 0; i <= degree; i++) {
      zPos += coeff[i]*std::pow(searchPoint.y, i);
    }
    pcl::PointXYZ originalPos(inputCloud->points[pointIdxSearch[0]].x, inputCloud->points[pointIdxSearch[0]].y, zPos);
    return originalPos;    
}

bool GeometryExtractor::getPolynomialCoeff(std::vector<double> xIn, std::vector<double> y, double coeff[], int degree) {
    gsl_vector *x = gsl_vector_alloc(xIn.size());
    gsl_vector *b = gsl_vector_alloc(xIn.size());
    gsl_vector *work = gsl_vector_alloc(degree+1);
    gsl_matrix *A = gsl_matrix_alloc(xIn.size(), degree+1);
    gsl_matrix *T = gsl_matrix_alloc(degree+1, degree+1);
    for(int i = 0; i < xIn.size(); i++) {
        gsl_vector_set(b, i, y[i]);
        for(int j = 0; j <= degree; j++) {
            gsl_matrix_set(A, i, j, std::pow(xIn[i], j));
        }
    }
    gsl_linalg_QR_decomp_r(A, T);
    gsl_linalg_QR_lssolve_r(A, T, b, x, work);

    for(int i = 0; i <= degree; i++) {
        coeff[i] = gsl_vector_get(x, i);
    }
    gsl_vector_free(x);
    gsl_vector_free(b);
    gsl_vector_free(work);
    gsl_matrix_free(A);
    gsl_matrix_free(T);
    
    return 1;
}