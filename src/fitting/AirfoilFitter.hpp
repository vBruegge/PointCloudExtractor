#ifndef AIRFOILFITTER_H
#define AIRFOILFITTER_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <string>

#include "Airfoil.hpp"
#include "IOHandler.hpp"

class AirfoilFitter {
public:
    /**
     * @brief Construct a new Airfoil Fitter object
     * 
     * @param foil airfoil section which is the fitted
     */
    AirfoilFitter(Airfoil& foil);

    /**
     * @brief executes all needed operations for a complete fitting
     * 
     * @param type type of fitting
     * possible types: spline, bernsteinPolynomial (default)
     */
    void initiateFitting(std::string type = "bernsteinPolynomial");

    /**
     * @brief sorts the upper and lower halves of the airfoil both increasingly
     * 
     */
    void sortUpperAndLowerHalves();

    /**
     * @brief orients the foil -> leading edge in the front, not upside down
     * 
     */
    void orientFoil();
    //orient foil

    /**
     * @brief interpolates given points with a spline
     * 
     * @param points points which should be fitted
     * @return std::vector<Eigen::Vector2d> new points of the fitting
     */
    std::vector<Eigen::Vector2d> splineInterpolation(std::vector<Eigen::Vector2d>& points);

    /**
     * @brief fits given points with a bernstein polynomial specified by Kulfan
     * 
     * @param points points which should be fitted
     * @param trailingEdgeWidth width of the trailing edge
     * @return std::vector<Eigen::Vector2d> new points of the fitting
     */
    std::vector<Eigen::Vector2d> bernsteinPolynomialFit(std::vector<Eigen::Vector2d>& points);

private:
    void computeCompareValues(Airfoil& foil);
    void splitAirfoil(std::vector<Eigen::Vector2d> points);
    bool checkIfFoilUpsideDown();
    bool checkPositionLeadingEdge(std::vector<Eigen::Vector2d>& points);
    long binomialCoeff(int n, int r);
    bool getBernsteinPolynomialCoeff(double xDc[], double yDc[], double coeff [], long binCoeff[], int degree, int numPoints);
    double getBernsteinPolynomialValue(double x, double ySupports[], int degree, long binCoeff[]);

    std::vector<Eigen::Vector2d> upper;
    std::vector<Eigen::Vector2d> lower;
    std::vector<Eigen::Vector2d> compare;
    std::string name;
    IOHandler io;
};

#endif