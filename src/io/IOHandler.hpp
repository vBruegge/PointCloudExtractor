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

    /**
     * @brief writes an airfoil DAT file
     * 
     * @param filename name of the file
     * @param points points of the airfoil
     */
    void writingPointCloud( const std::string& filename, const std::vector<Eigen::Vector2d>& points );

    /**
     * @brief writes a CSV file with all airfoil parameters
     * 
     * @param outStream filestream
     * @param data exctracted geometries of the airfoil sections
     * @param sectionType (wing/horizontal_tail/vertical_tail)
     * @param length number of sections of given sectionType
     */
    void writingWingDataInCSV( std::ofstream& outStream, AirfoilParameter data[], std::string& sectionType, int length);

    /**
     * @brief writes a CSV file with all fuselage parameters
     * 
     * @param outStream filestream
     * @param data exctracted geometries of the airfoil sections
     * @param length number of sections of the fuselage
     */
    void writingFuselageDataInCSV(std::ofstream& outStream, FuselageParameter data[], int length);

    /**
     * @brief reads the sectioning file and copies the values in the corresponding vectors
     * 
     * @param filename name of the sectioning file
     * @param splittingDistance distance between the wing and tail for splitting aircraft in two
     * @param fuselageSections vector of fuselage sections
     * @param wingSections vector of wing sections
     * @param horizontalTailSections  vector of horizontal tail sections
     * @param verticalTailSections  vector of vertical tail section
     */
    void readSectionFile(std::string& filename, float& splittingDistance, std::vector<float>& fuselageSections,
        std::vector<float>& wingSections, std::vector<float>& horizontalTailSections, std::vector<float>& verticalTailSections);

    /**
     * @brief reads a point cloud of a TXT file and converts it to a PCD file
     * 
     * @param filename name of the TXT file
     */
    void convertTXTToPCDFile(std::string& filename);

private:
    void readLineInVector(std::string& line, std::string& type, std::vector<float>& sections);
};

#endif