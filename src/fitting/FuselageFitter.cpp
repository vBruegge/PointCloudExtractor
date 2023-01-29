#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Core>

#include "FuselageFitter.hpp"

FuselageFitter::FuselageFitter(Fuselage& section_, std::string sourceFolder) : section(section_){
    section = section_;
    io = IOHandler(sourceFolder);
}

void FuselageFitter::circularFit() {

    FuselageParameter params = section.getFuselageParameter();
    double radius = (params.disX + params.disY)/2;

    std::vector<Eigen::Vector2d> newPoints(360);
    for(int i = 0; i < 360; i++) {
        double degree = (double)i/180*M_PI;
        double xi = params.xM + radius * std::cos(degree);
        double yi = params.yM + radius * std::sin(degree);
        newPoints[i] = Eigen::Vector2d(xi, yi);
    }
    io.writingPointCloud(params.name, newPoints);
    params.disX = params.disY = radius;
    params.epsilon = 1.0;
    section.setAllFuselageParameter(params);
}

void FuselageFitter::ellipsoidalFit() {
    FuselageParameter params = section.getFuselageParameter();

    std::vector<Eigen::Vector2d> newPoints(360);
    for(int i = 0; i < 360; i++) {
        double degree = (double)i/180*M_PI;
        double xi = params.xM + params.disX * std::cos(degree);
        double yi = params.yM + params.disY * std::sin(degree);
        newPoints[i] = Eigen::Vector2d(xi, yi);
    }
    io.writingPointCloud(params.name, newPoints);
    params.epsilon = 1.0;
    section.setAllFuselageParameter(params);
}

FuselageParameter FuselageFitter::superellipsoidalFit() {
    FuselageParameter params = section.getFuselageParameter();
    std::vector<Eigen::Vector2d> points(section.getFuselage()->points.size());
    for(int i = 0; i < points.size(); i++) {
        points[i][0] = section.getFuselage()->points[i].x - params.xM;
        points[i][1] = section.getFuselage()->points[i].z - params.yM;
    }
    io.writingPointCloud(params.name + "_scan.txt", points);

    int iterate = 0;
    double fittedQuality;
    double boundaryRight = 50.0, boundaryLeft = 0.0;
    double epsilon = 4/(boundaryRight + boundaryLeft);

    do {
        fittedQuality = 0;
        double diff = 0;
        for(int i = 0; i < points.size(); i++) {
            double xSuperellipse = std::pow(abs(points[i][0])/params.disX,2/epsilon);
            double ySuperellipse = std::pow(abs(points[i][1])/params.disY,2/epsilon);
            fittedQuality += (xSuperellipse+ySuperellipse-1);
        }
        fittedQuality /= points.size();
        iterate++;

        if(fittedQuality < 0)
            boundaryRight = 2/epsilon;
        else
            boundaryLeft = 2/epsilon;

        epsilon = 4/(boundaryLeft + boundaryRight);    
        
    } while(iterate < 25 && abs(fittedQuality) > 1e-3);

    params.epsilon = epsilon;
    std::vector<Eigen::Vector2d> newPoints(360);
    for(int i = 0; i < 90; i++) {
        double degree = (double)i/180*M_PI;
        double xi = params.xM + params.disX * std::pow(std::cos(degree), epsilon);
        double yi = params.yM + params.disY * std::pow(std::sin(degree), epsilon);
        double yiNeg = params.yM - params.disY * std::pow(std::sin(degree), epsilon);
        newPoints[i] = Eigen::Vector2d(xi, yi);
        newPoints[180-i-1] = Eigen::Vector2d(-xi, yi);
        newPoints[180+i] = Eigen::Vector2d(-xi, yiNeg);
        newPoints[360-i-1] = Eigen::Vector2d(xi, yiNeg);
    }
    io.writingPointCloud(params.name, newPoints);
    section.setAllFuselageParameter(params);  

    return params;  
}