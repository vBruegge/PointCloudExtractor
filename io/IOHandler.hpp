#ifndef IOHANDLER_H
#define IOHANDLER_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <fstream>

#include "Airfoil.hpp"
#include "Fuselage.hpp"

class IOHandler {
public:

    void writingPointCloud( const std::string& filename, const std::vector<Eigen::Vector2d>& points );

    void writingWingDataInCSV( std::ofstream& outStream, AirfoilParameter data[], std::string& sectionType, int length);

    void writingFuselageDataInCSV(std::ofstream& outStream, FuselageParameter data[], int length);

    void readSectionFile(std::string& filename, float splittingDistance, std::vector<float>& fuselageSections,
        std::vector<float>& wingSections, std::vector<float>& horizontalTailSections, std::vector<float>& verticalTailSections);

    void convertTXTToPCDFile(std::string& filename);

private:
    std::vector<float> readLineInVector(std::string& line);
};

#endif