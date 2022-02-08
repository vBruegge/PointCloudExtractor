#include <string>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"
#include "AirfoilFitter.hpp"
#include "FuselageFitter.hpp"

int main (int argc, char** argv)
{
    //convert the given point cloud in a valid PCD file
    std::string pointCloudFile = argv[1];
    IOHandler io;
    io.convertTXTToPCDFile(pointCloudFile);

    // check if the fuselage is greater than the wing width -> additional rotation needed
    std::string fuselageWidth = argv[2];
    bool fuselageGreaterThanWing = false;
    if(fuselageWidth == "y") {
        fuselageGreaterThanWing = true;
    }
    // reads the converted pcd file, alignes the point cloud and computes normals
    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, fuselageGreaterThanWing);

    //start the section generation gui if the argument is "new", otherwise use a given sectioning file
    std::string sectionFilename = argv[3];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getPointCloudWithoutNormals());
        sectionFilename = "section-generation.txt";
    }

    //reads the given sectioning file !all vectors are needed even if there are no sections which should be generated 
    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);

    //generates all point clouds which should be sectioned
    pcl::PointCloud<pcl::PointNormal>::Ptr wing(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail(new pcl::PointCloud<pcl::PointNormal>); 
    pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage = op.getPointCloudWithoutNormals();
    op.splitCloudInWingAndTail(wing, horizontalTail, verticalTail, splittingDistance);

    //sectioning the fuselage
    GeometryExtractor extract;
    FuselageParameter dataFuselage[fuselageSections.size()];
    for(int i = 0; i < fuselageSections.size(); i++) {
        Fuselage section = extract.sectioningCloudY(fuselage, fuselageSections[i]);
        FuselageFitter fitFuselage(section);
        dataFuselage[i] = fitFuselage.superellipseFit();
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
    io.writingFuselageDataInCSV(aircraftDataFile, dataFuselage, fuselageSections.size());

    //sectioning the wing
    bool flapRotationNeeded = true;
    std::string flapRotated = argv[4];
    if(flapRotated == "n")
        flapRotationNeeded = false;
    AirfoilParameter dataWing[wingSections.size()];
    std::string sectionType ="wing";
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(wing, wingSections[i], flapRotationNeeded);
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

    io.writingWingDataInCSV(aircraftDataFile, dataWing, sectionType, wingSections.size());

    //sectioning the horizontal tail
    AirfoilParameter dataHTail[horizontalTailSections.size()];
    sectionType = "horizontal_tail";
    for(int i = 0; i < horizontalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(horizontalTail, horizontalTailSections[i], flapRotationNeeded);
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

    io.writingWingDataInCSV(aircraftDataFile, dataHTail, sectionType, horizontalTailSections.size());

    //sectioning the vertical tail
    AirfoilParameter dataVTail[verticalTailSections.size()];
    sectionType = "vertical_tail";
    for(int i = 0; i < verticalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudZ(verticalTail, verticalTailSections[i], flapRotationNeeded);
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

    io.writingWingDataInCSV(aircraftDataFile, dataVTail, sectionType, verticalTailSections.size());


    return 0;
}