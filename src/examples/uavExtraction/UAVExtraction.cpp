#include <string>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"
#include "AirfoilFitter.hpp"
#include "FuselageFitter.hpp"

int main (int argc, char** argv)
{
    std::string pointCloudFile = argv[1];
    IOHandler io;
    io.convertTXTToPCDFile(pointCloudFile);

    std::string fuselageWidth = argv[2];
    bool fuselageGreaterThanWing = false;
    if(fuselageWidth == "y") {
        fuselageGreaterThanWing = true;
    }
    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, fuselageGreaterThanWing);

    std::string sectionFilename = argv[3];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getPointCloudWithoutNormals());
        sectionFilename = "section-generation.txt";
    }
    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);

    pcl::PointCloud<pcl::PointNormal>::Ptr wing(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail(new pcl::PointCloud<pcl::PointNormal>); 
    pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage = op.getPointCloudWithoutNormals();

    op.splitCloudInWingAndTail(wing, horizontalTail, verticalTail, splittingDistance);

    GeometryExtractor extract;
    FuselageParameter dataFuselage[fuselageSections.size()];
    for(int i = 0; i < fuselageSections.size(); i++) {
        Fuselage section = extract.sectioningCloudY(fuselage, fuselageSections[i]);
        FuselageFitter fitFuselage(section);
        fitFuselage.superellipseFit();
        dataFuselage[i] = section.getFuselageParameter();
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
    io.writingFuselageDataInCSV(aircraftDataFile, dataFuselage, fuselageSections.size());

    int sectioningType = std::stoi(argv[4]);
    AirfoilParameter dataWing[wingSections.size()];
    std::string sectionType = "wing";
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(wing, wingSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge(section.getFoil())[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, 0, wingSections[i]);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataWing[0].offset, dataWing[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataWing[i] = section.getAirfoilParameter();
    }

    AirfoilParameter dataHTail[horizontalTailSections.size()];
    sectionType = "horizontal_tail";
    for(int i = 0; i < horizontalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(horizontalTail, horizontalTailSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge(section.getFoil())[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, 0, horizontalTailSections[i]);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataHTail[0].offset, dataHTail[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataHTail[i] = section.getAirfoilParameter();
    }

    AirfoilParameter dataVTail[verticalTailSections.size()];
    sectionType = "vertical_tail";
    for(int i = 0; i < verticalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudZ(verticalTail, verticalTailSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge(section.getFoil())[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, 0, verticalTailSections[i]);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataVTail[0].offset, dataVTail[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataVTail[i] = section.getAirfoilParameter();
    }
    return 0;
}