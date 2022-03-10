#include <sstream>
#include <algorithm>
#include <iomanip>
#include <stdlib.h>

#include "IOHandler.hpp"

IOHandler::IOHandler(std::string sourceFolder_) : sourceFolder(sourceFolder_) {};

void IOHandler::writingPointCloud(const std::string& filename, const std::vector<Eigen::Vector2d>& points) {
    std::ofstream fout(sourceFolder + "/Results/" + filename);
    fout << std::setprecision(10);
	if( fout.fail()) {
		std::cout << "Error: Could not open file";
        exit(EXIT_FAILURE);
    }    

	for(int i = 0; i < points.size(); i++)
	{
		fout << points[i].x() << " " << points[i].y() << std::endl;
	}
	fout.close();
}

void IOHandler::writingWingDataInCSV( std::ofstream& outStream, AirfoilParameter data[], std::string& sectionType, int length) {
    std::stringstream ss;
    ss << std::setprecision(10);
    ss << "/TABLE; " << sectionType << "\n\r"
        << "/FIELDS\n\r"
        << "Filename; Sectioning Distance [mm]; Chord Length [mm]; Dihedral [deg]; Twist [deg]; "
        << "Flap Position [%]; Offset [mm]; Sweep [deg]; Trailing Edge Width [mm]; x-Position Leading Edge [mm]; "
        << "y-Position Leading Edge [mm]; z-Position Leading Edge [mm]\n\r";
    for(int i = 0; i < length; i++) {
        if(sectionType == "vertical_tail") {
            data[i].posLeadingEdge.z = data[i].posLeadingEdge.x;
            data[i].posLeadingEdge.x = 0;
        }
        ss << data[i].name << "; " << data[i].cuttingDistance << "; " << data[i].chordLength << "; " << data[i].dihedral << "; "
            << data[i].twist << "; " << data[i].flapPosition << "; " << data[i].offset << "; " << data[i].sweep << "; "
            << data[i].trailingEdgeWidth << "; " << data[i].posLeadingEdge.x << "; " << data[i].posLeadingEdge.y << "; "
            << data[i].posLeadingEdge.z << "\n\r";
    }
    ss << "/END; " << sectionType << "\n\r\n";
    outStream << ss.str();
}

void IOHandler::writingMorphingWingDataInCSV( std::ofstream& outStream, MorphingWingParameter data[], int length) {
    std::stringstream ss;
    ss << std::setprecision(10);
    ss << "/TABLE; MorphingWing\n\r"
        << "/FIELDS\n\r"
        << "Filename; Sectioning Distance [mm]; Scale []; rotationAngle [deg]\n\r";
    for(int i = 0; i < length; i++) {
        ss << data[i].name << "; " << data[i].cuttingDistance << "; " << data[i].scale << "; " << data[i].rotationAngle << "\n\r";
    }
    ss << "/END; MorphingWing\n\r\n";
    outStream << ss.str();
}

void IOHandler::writingFuselageDataInCSV( std::ofstream& outStream, FuselageParameter data[], int length) {
    std::stringstream ss;
    ss << std::setprecision(10);
    ss << "/TABLE; Fuselage\n\r"
        << "/FIELDS\n\r"
        << "Filename; Sectioning Distance [mm]; Half-axis X [mm]; Half-axis Y [mm]; Center X [mm]; "
        << "Center Y [mm]; epsilon []\n\r";
    for(int i = 0; i < length; i++) {
        ss << data[i].name << "; " << data[i].cuttingDistance << "; " << data[i].disX << "; " << data[i].disY << "; "
            << data[i].xM << "; " << data[i].yM << "; " << data[i].epsilon << "\n\r";
    }
    ss << "/END; Fuselage\n\r\n";
    outStream << ss.str();
}

void IOHandler::readSectionFile(std::string& filename, float& splittingDistance, std::vector<float>& fuselageSections,
    std::vector<float>& wingSections, std::vector<float>& horizontalTailSections, std::vector<float>& verticalTailSections) {

    std::string tailType;

    std::ifstream sectionFile(filename, std::ifstream::in);
    std::string line;
    std::string type;
    std::vector<float> sections;
    //reading section file
    if(!sectionFile.is_open()) {
        std::cout << "Error, no section file!\n";
        exit(EXIT_FAILURE);
    }    

    std::getline(sectionFile, tailType);

    std::getline(sectionFile, line);
    splittingDistance = std::stof(line);

    std::getline(sectionFile, line);
    readLineInVector(line, type, sections);
    fuselageSections = sections;

    sections = {};
    std::getline(sectionFile, line);
    readLineInVector(line, type, sections);
    if(type == "wing")
        wingSections = sections;
    else
        horizontalTailSections = sections;
    sections = {};

    std::getline(sectionFile, line);
    readLineInVector(line, type, sections);
    if(type == "wing")
        wingSections = sections;
    else
        horizontalTailSections = sections;
    sections = {};

    if(tailType == "h") {
        std::getline(sectionFile, line);
        readLineInVector(line, type, sections);
        verticalTailSections = sections;
    }
    
}

void IOHandler::readLineInVector(std::string& line, std::string& type, std::vector<float>& sections) {
    std::stringstream ss;
    ss << std::setprecision(2);
    ss << std::fixed;

    float tmp;
    ss << line;
    ss >> type;
    while( ss >> tmp) {
        sections.push_back(tmp);
    }
}

void IOHandler::convertTXTToPCDFile(std::string& filename) {
    std::ifstream fin(filename, std::ifstream::in);
    if(!fin.is_open()) {
        std::cout << "Error, no point cloud file!\n";
        exit(EXIT_FAILURE);
    }    
    std::stringstream buffer;
    buffer << fin.rdbuf();
    std::string file = buffer.str();

    size_t count = std::count(file.begin(), file.end(), '\n');
    std::ofstream fout;
    fout.open(filename+".pcd", std::ofstream::out);
    fout << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z\n"
        << "SIZE 4 4 4\n"
        << "TYPE F F F\n"
        << "COUNT 1 1 1\n"
        << "WIDTH " << count << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << count << "\n"
        << "DATA ascii\n";
    fout << file;
    fout.close();
}

std::vector<Eigen::Vector2d> IOHandler::readAirfoilDATFile(const std::string& filename) {
    std::ifstream airfoil(filename, std::ifstream::in);
    std::string line;
    //reading section file
    if(!airfoil.is_open()) {
        std::cout << "Error, no airfoil file!\n";
        exit(EXIT_FAILURE);
    }    
    
    std::vector<Eigen::Vector2d> points;
    std::stringstream ss;
    while(std::getline(airfoil, line)) {

        Eigen::Vector2d tmp;
        ss << line;
        ss >> tmp[0];
        ss >> tmp[1];

        ss.str("");
        ss.clear();

        points.push_back(tmp);
    }

    return points;
}
