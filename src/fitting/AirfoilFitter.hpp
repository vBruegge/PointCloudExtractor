#ifndef AIRFOILFITTER_H
#define AIRFOILFITTER_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <string>

#include "Airfoil.hpp"
#include "IOHandler.hpp"

class AirfoilFitter {
public:
    AirfoilFitter(Airfoil& foil);

    void initiateFitting(std::string type = "bernsteinPolynomial");

    void sortUpperAndLowerHalves();

    void orientFoil();
    //orient foil

    std::vector<Eigen::Vector2d> splineInterpolation(std::vector<Eigen::Vector2d>& points);

    std::vector<Eigen::Vector2d> bernsteinPolynomialFit(std::vector<Eigen::Vector2d>& points, float trailingEdgeWidth);

    void replaceMorphedFlap(std::vector<Eigen::Vector2d>& referenceProfile);

private:
    void computeCompareValues(Airfoil& foil);
    void splitAirfoil(std::vector<Eigen::Vector2d> points);
    bool checkIfFoilUpsideDown();
    bool checkPositionLeadingEdge(std::vector<Eigen::Vector2d>& points);
    long binomialCoeff(int n, int r);
    bool getBernsteinPolynomialCoeff(double xDc[], double yDc[], double coeff [], long binCoeff[], int degree, int numPoints);
    double getBernsteinPolynomialValue(double x, double ySupports[], int degree, long binCoeff[], float trailingEdgeWidth);

    std::vector<Eigen::Vector2d> upper;
    std::vector<Eigen::Vector2d> lower;
    std::vector<Eigen::Vector2d> compare;
    std::string name;
    float trailingEdgeWidth;
    IOHandler io;
};

#endif