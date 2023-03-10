#include <gsl/gsl_math.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_linalg.h>

#include <pcl/filters/passthrough.h>
#include "AirfoilFitter.hpp"
#include "Airfoil.hpp"

bool sortIncrease(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
    return a[0]<b[0];
}

AirfoilFitter::AirfoilFitter(Airfoil& foil, std::string sourceFolder) {
    //copy y- and z-distance in array
    io = IOHandler(sourceFolder);
    std::vector<Eigen::Vector2d> points;
    points.resize((int) foil.getFoil()->size());
    for(int i = 0; i < foil.getFoil()->size(); i++) {
      points[i] = Eigen::Vector2d(foil.getFoil()->points[i].y, foil.getFoil() -> points[i].z);
    }
    AirfoilParameter params = foil.getAirfoilParameter();
    io.writingPointCloud(params.name + "_scan.txt", points);
    
    computeCompareValues(foil);
    splitAirfoil(points);

    name = params.name;
}

void AirfoilFitter::computeCompareValues(Airfoil& foil) {
    //calculate approximative skeleton line of the foil
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = foil.getFoil();
    std::vector<int> indexLeadingTrailingEdge = foil.findLeadingTrailingEdge();
    pcl::PointXYZ maxPt, minPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    float sectionDisX = 15;
    float dis = 5;
    if(abs(maxPt.y-minPt.y) < 1) {
        sectionDisX = 0.1;
        dis = 0.05;
    }
    const int iterator = abs((maxPt.y-minPt.y)) / sectionDisX + 2;
    std::vector<Eigen::Vector2d> compare_(iterator);
    pcl::PointXYZ leadingEdge = inputCloud->points[indexLeadingTrailingEdge[0]];
    pcl::PointXYZ trailingEdge = inputCloud->points[indexLeadingTrailingEdge[1]];

    float beginSection;
    if(leadingEdge.y < trailingEdge.y) {
      beginSection = leadingEdge.y;
    }
    else {
      beginSection = trailingEdge.y;
    }

    compare_[0] = Eigen::Vector2d(leadingEdge.y, leadingEdge.z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (inputCloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits(maxPt.y-dis/6, maxPt.y+dis/6);
    pass.filter(*cloudPassThrough);
    pcl::getMinMax3D(*cloudPassThrough, minPt, maxPt);
    compare_[compare_.size()-1] = Eigen::Vector2d(maxPt.y, (minPt.z+maxPt.z)/2);

    for(int i = 1; i < iterator-1; i++) {
      pass.setFilterLimits (beginSection + i*sectionDisX - dis, beginSection + i*sectionDisX + dis);
      pass.filter (*cloudPassThrough);

      pcl::getMinMax3D(*cloudPassThrough, minPt, maxPt);
      Eigen::Vector2d compareValue = Eigen::Vector2d((maxPt.y + minPt.y)/2, (maxPt.z + minPt.z)/2);
      compare_[i] = Eigen::Vector2d((maxPt.y + minPt.y)/2, (maxPt.z + minPt.z)/2);
    }
    compare = compare_;
}

void AirfoilFitter::splitAirfoil(std::vector<Eigen::Vector2d>& points) {
    int compareIndex = compare.size()-1;

    std::vector<Eigen::Vector2d> tmpUpper, tmpLower;
    //seperate the vector in two, one for the upper surface, one for the lower
    for(int i = 0; i < points.size(); i++){
        for(int j = 0; j < compare.size()-1; j++) {
            if(points[i][0] < compare[j+1][0]) {
                compareIndex = j;
                break;
            }            
        }
        //line from first point to second, then calculates the yAxis of the point
        // X = compare[j] + point[i].x * (compare[j+1]-compare[j])
        Eigen::Vector2d gradient = (compare[compareIndex+1]-compare[compareIndex]);
        float ycompare = compare[compareIndex][1]+(points[i][0]-compare[compareIndex][0])/gradient[0]*gradient[1];
        if(points[i][1]> ycompare)
            tmpUpper.push_back(points[i]);
        else
            tmpLower.push_back(points[i]);
    }
    lower = tmpLower;
    upper = tmpUpper;
}

void AirfoilFitter::sortUpperAndLowerHalves() {
    //sort upper (increasing) and lower (increasing) vector
    std::sort(upper.begin(), upper.end(), sortIncrease);
    std::sort(lower.begin(), lower.end(), sortIncrease);
}

void AirfoilFitter::orientFoil () {
    bool upsideDown = checkIfFoilUpsideDown();
    if(upsideDown == true) {
        //flip foil at y-Axis
        for(int i = 0; i < upper.size(); i++)
            upper[i][1] = -upper[i][1];
        for(int i = 0; i < lower.size(); i++)
            lower[i][1] = -lower[i][1];
    }
    bool posLeadingEdge;
    if(upsideDown == true)
        posLeadingEdge = checkPositionLeadingEdge(lower);
    else
        posLeadingEdge = checkPositionLeadingEdge(upper);
    //flip foil at x-Axis
    if(posLeadingEdge == false) {
        for(int i = 0; i < upper.size(); i++)
            upper[i][0] = -upper[i][0];
        for(int i = 0; i < lower.size(); i++)
            lower[i][0] = -lower[i][0];
        for(int i = 0; i < compare.size(); i++)
            compare[i][0] = -compare[i][0];
    }
}

bool AirfoilFitter::checkIfFoilUpsideDown() {
    //check if foil is upside down through curvature
    //returns true if the mid of the skeleton line is lower than the first point

    float min = compare[0][1], max = compare[0][1];
    for(int i = 1; i < compare.size()/2; i++) {
        if(abs(min - compare[i][1]) > abs(min-max))
            max = compare[i][1];
    }
    if(min > max) {
        return true;
    }
    else {
        return false;
    }
}

bool AirfoilFitter::checkPositionLeadingEdge(std::vector<Eigen::Vector2d>& points) {
    //check if foil is oriented leading edge to trailing edge, returns true if leading edge position < trailing edge position
    float frontY = abs(points[0][1]-points[20][1]);
    float backY = abs(points[points.size()-21][1]-points[points.size()-1][1]); //already sorted from smallest value to greatest to smallest
    if(frontY > backY)
        return true;
    else
        return false;
}

std::vector<Eigen::Vector2d> AirfoilFitter::splineInterpolation(std::vector<Eigen::Vector2d>& points) {
    //fit gsl spline on curve
    int size = points.size();
    double maxDis = abs(points[0][0] - points[size-1][0]);
    double x[size];
    double y[size];
    float max = points[0][1] / (maxDis/2);
    float min = max;
    for(int i  = 0 ; i < size; i++) {
        //scale to 2 and translate to -1
        x[i] = (points[i][0]-points[0][0]-maxDis/2)/(maxDis/2);
        if(x[i] == x[i-1])
            x[i] += 0.00001;
        y[i] = (points[i][1])/(maxDis/2);
    }

    gsl_interp_accel *acc = gsl_interp_accel_alloc();
    gsl_spline *spline = gsl_spline_alloc(gsl_interp_steffen, size);
    gsl_spline_init(spline, x, y, size);

    if(size < 800)
        size = 800; 
    
    double xi[size], yi[size];

    xi[0] = -1;
    yi[0]=gsl_spline_eval(spline, xi[0], acc);
    xi[size-1] = 1;
    yi[size-1]=gsl_spline_eval(spline, xi[size-1], acc);
    for (int i = 1; i < size-1; ++i)
    {
        //node calculation with chebyshev nodes for stability
        xi[i] = -cos((2*i-1)/(float)(2*size)*M_PI);
        yi[i] = gsl_spline_eval(spline, xi[i], acc);
    }    

    std::vector<Eigen::Vector2d> newPoints;
    int add = size / 70;
    //chose the optimal y distance of polynom, spline and original foil
    for(int i = 0; i < size; i+=add) {
        newPoints.push_back(Eigen::Vector2d(xi[i], yi[i]));
    }

    gsl_spline_free(spline);

    return newPoints;
}

long AirfoilFitter::binomialCoeff(const int n, const int k) {
      long double res = 1;
    for (int i = 1; i <= k; ++i)
        res = res * (n - k + i) / i;
    return (long)(res + 0.01);
}

double AirfoilFitter::getBernsteinPolynomialValue(double xDc, double coeff[], int degree, long binCoeff[]) {
    double yDc = 0;
    for(int i = 0; i <= degree; i++) {
        yDc += binCoeff[i]*std::pow(xDc,i)*std::pow(1-xDc, degree+1-i)*coeff[i]*std::sqrt(xDc);
    }
    return yDc;
}

std::vector<Eigen::Vector2d> AirfoilFitter::bernsteinPolynomialFit(std::vector<Eigen::Vector2d>& points) {
    //fit gsl spline on curve
    int size = points.size();
    int min = 0;
    int max = points.size()-1;
    float maxDis = abs(points[min][0]-points[max][0]);
    float trailingEdgeWidth = (points[max][1]-points[min][1])/maxDis;
    double yDc[size], xDc[size], yDcTranslated[size];

    //scale to 1
    for(int i  = 0 ; i < size; i++) {
        //scale to 1 and translate to (0,0)
        yDc[i] = (points[i][1]-points[min][1])/maxDis;
        yDcTranslated[i] = yDc[i]-trailingEdgeWidth;
        xDc[i] = (points[i][0]-points[min][0])/maxDis;
    }
    int degree = 8;

    //calculate binnomial coefficient for bernstein polynomial
    long binCoeff[degree+1];
    for(int i = 0; i <= degree; i++) {
        binCoeff[i] = binomialCoeff(degree, i);
    }

    //get the coefficients for the bernstein polynomal fitting function
    double coeff[degree+1];
    getBernsteinPolynomialCoeff(xDc, yDc, coeff, binCoeff, degree, size);
    double coeffTranslated[degree+1];
    getBernsteinPolynomialCoeff(xDc, yDcTranslated, coeffTranslated, binCoeff, degree, size);

    //saving smoothed points
    std::vector<Eigen::Vector2d> newPoints;
    size = 100;

    newPoints.push_back(Eigen::Vector2d(0,0));
    for(int i = 0; i < size; i++) {
        double xiDc = 0.5 - 0.5*cos((2*i-1)/(float)(2*size)*M_PI);
        double yiDc;
        if(abs(trailingEdgeWidth) > 1e-3 && xiDc > 0.5) {
            yiDc = getBernsteinPolynomialValue(xiDc, coeffTranslated, degree, binCoeff);
            yiDc += trailingEdgeWidth;
        }
        else {
            yiDc = getBernsteinPolynomialValue(xiDc, coeff, degree, binCoeff);
        }
        newPoints.push_back(Eigen::Vector2d(xiDc, yiDc));
    }
    if(abs(trailingEdgeWidth) > 5e-4) {
        newPoints.push_back(Eigen::Vector2d(1, getBernsteinPolynomialValue(1, coeffTranslated, degree, binCoeff)+trailingEdgeWidth));
    }
    else {
        newPoints.push_back(Eigen::Vector2d(1, getBernsteinPolynomialValue(1, coeffTranslated, degree, binCoeff)));
    }

    return newPoints;
}                                                                        

//get the coefficients for the bernstein polynomal fitting function
bool AirfoilFitter::getBernsteinPolynomialCoeff(double xDc[], double yDc[], double coeff[], long binCoeff[], int degree, int numPoints) {
    gsl_vector *x = gsl_vector_alloc(numPoints);
    gsl_vector *b = gsl_vector_alloc(numPoints);
    gsl_vector *work = gsl_vector_alloc(degree+1);
    gsl_matrix *A = gsl_matrix_alloc(numPoints, degree+1);
    gsl_matrix *T = gsl_matrix_alloc(degree+1, degree+1);
    for(int i = 0; i < numPoints; i++) {
        gsl_vector_set(b, i, yDc[i]);
        for(int j = 0; j <= degree; j++) {
            long double temp = binCoeff[j]*std::pow(1-xDc[i], degree+1-j)*std::pow(xDc[i], j)*std::sqrt(xDc[i]);
            gsl_matrix_set(A, i, j, temp);
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

void AirfoilFitter::initiateFitting(std::string type) {

    sortUpperAndLowerHalves();
    orientFoil();
    sortUpperAndLowerHalves();
    float disX = abs(upper[0][1]-lower[0][1]);
    if(disX > 1) {
        Eigen::Vector2d center = Eigen::Vector2d(std::min(upper[0][0],lower[0][0]), (upper[0][1]+lower[0][1])/2);
        lower.insert(lower.begin(), center);
        upper.insert(upper.begin(), center);
    }

    std::vector<Eigen::Vector2d> newUpper;
    std::vector<Eigen::Vector2d> newLower;
    if(type == "bernsteinPolynomial") {
        newUpper = bernsteinPolynomialFit(upper);
        newLower = bernsteinPolynomialFit(lower);
    }
    else if(type == "spline") {
        newUpper = splineInterpolation(upper);
        newUpper = splineInterpolation(lower);
    }

    std::vector<Eigen::Vector2d> foil;
    foil.insert(foil.end(), newUpper.rbegin(), newUpper.rend());
    foil.insert(foil.end(), newLower.begin(), newLower.end());

    io.writingPointCloud("../Results/" + name, foil);
    /*io.writingPointCloud("../Results/" + name + "_upper.txt", upper);
    io.writingPointCloud("../Results/" + name + "_newUpper.txt", newUpper);
    io.writingPointCloud("../Results/" + name + "_compare.txt", compare);
    io.writingPointCloud("../Results/" + name + "_lower.txt", lower);
    io.writingPointCloud("../Results/" + name + "_newLower.txt", newLower);*/
}

void AirfoilFitter::replaceMorphedFlap(std::vector<Eigen::Vector2d>& referenceProfile) {
    sortUpperAndLowerHalves();
    /*io.writingPointCloud("../Results/" + name + "_upper.txt", upper);
    io.writingPointCloud("../Results/" + name + "_lower.txt", lower);
    io.writingPointCloud("../Results/" + name + "_compare.txt", compare);*/
    if(checkIfFoilUpsideDown() == true) {
        for(int i = 0; i < upper.size(); i++) {
            upper[i][1] = -upper[i][1] + 2*upper[upper.size()-20][1];
        }
        for(int i = 0; i < lower.size(); i++) {
            lower[i][1] = -lower[i][1] + 2*upper[upper.size()-20][1];
        }
    }
    if(upper.size() > 100) {
        downsizeAirfoil(upper);
    }
    if(lower.size() > 100) {
        downsizeAirfoil(lower);
    }

    float maxX = std::min(upper[upper.size()-1][0], lower[lower.size()-1][0]);
    std::vector<Eigen::Vector2d> tmp;
    for(int i = 0; i < referenceProfile.size(); i++) {
        if(referenceProfile[i][0] > maxX && i < referenceProfile.size() / 2) {
            tmp.push_back(referenceProfile[i]);
        }
        else if(referenceProfile[i][0] > maxX && i > referenceProfile.size() / 2) {
            lower.push_back(referenceProfile[i]);
        }
    }
    std::vector<Eigen::Vector2d> foil;
    foil.insert(foil.end(), tmp.begin(), tmp.end());
    foil.insert(foil.end(), upper.rbegin(), upper.rend());
    foil.insert(foil.end(), lower.begin(), lower.end());

    io.writingPointCloud("../Results/" + name, foil);
}

void AirfoilFitter::downsizeAirfoil(std::vector<Eigen::Vector2d>& points) {
    int add = points.size()/100 + 10;
    std::vector<Eigen::Vector2d> newPoints;
    for(int i = 0; i < points.size(); i+=add) {
        if(points[i][0] < 0.1) {
            newPoints.insert(newPoints.end(), points.begin()+i, points.begin()+i+add);
        }
        else {
            newPoints.push_back(points[i]);
        }
    }
    points = newPoints;
}