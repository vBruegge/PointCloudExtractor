#include <string>
#include <boost/filesystem.hpp>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"
#include "AirfoilFitter.hpp"
#include "FuselageFitter.hpp"

int main (int argc, char** argv)
{
    //create result folder
    std::string sourceFolder = argv[1];
    if(boost::filesystem::is_directory(sourceFolder + "Results") == false)
        boost::filesystem::create_directory(sourceFolder + "/Results");

    //convert the given point cloud in a valid PCD file
    std::string pointCloudFile = sourceFolder + "/Scans/" + argv[2];
    IOHandler io(sourceFolder);
    io.convertTXTToPCDFile(pointCloudFile);

    // check if the fuselage is greater than the wing width -> additional rotation needed
    std::string fuselageWidth = argv[3];
    bool fuselageGreaterThanWing = false;
    if(fuselageWidth == "y") {
        fuselageGreaterThanWing = true;
    }
    // reads the converted pcd file, alignes the point cloud and computes normals
    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, fuselageGreaterThanWing, true);

    //start the section generation gui if the argument is "new", otherwise use a given sectioning file
    std::string sectionFilename = argv[4];
    if(sectionFilename == "new") {
        sectionFilename = pointCloudFile + "_section-generation.txt";
        sectionGenerationGUI(op.getPointCloudWithoutNormals(), sourceFolder, sectionFilename);
    }
    else
        sectionFilename = sourceFolder + "/Scans/" + sectionFilename;

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
        FuselageFitter fitFuselage(section, sourceFolder);
        dataFuselage[i] = fitFuselage.superellipsoidalFit();
        std::cout << "Writing fuselage complete...\n";
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open(sourceFolder + "/Results/aircraftDataFile.csv", std::fstream::out);
    io.writingFuselageDataInCSV(aircraftDataFile, dataFuselage, fuselageSections.size());

    //sectioning wing
    int sectioningType = std::stoi(argv[5]);
    AirfoilParameter dataWing[wingSections.size()];
    std::string sectionType = "wing";
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(wing, wingSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge()[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, section.getAirfoilParameter().posLeadingEdge);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataWing[0].posLeadingEdge);
        }
        AirfoilFitter fitAirfoil(section, sourceFolder);
        fitAirfoil.initiateFitting();
        dataWing[i] = section.getAirfoilParameter();
        std::cout << "Writing wing complete...\n";
    }

    io.writingWingDataInCSV(aircraftDataFile, dataWing, sectionType, wingSections.size());

    //sectioning the horizontal tail
    AirfoilParameter dataHTail[horizontalTailSections.size()];
    sectionType = "horizontal_tail";
    for(int i = 0; i < horizontalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(horizontalTail, horizontalTailSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge()[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, section.getAirfoilParameter().posLeadingEdge);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataHTail[0].posLeadingEdge);
        }
        AirfoilFitter fitAirfoil(section, sourceFolder);
        fitAirfoil.initiateFitting();
        dataHTail[i] = section.getAirfoilParameter();
        std::cout << "Writing horizontal tail complete...\n";
    }

    io.writingWingDataInCSV(aircraftDataFile, dataHTail, sectionType, horizontalTailSections.size());

    //sectioning the vertical tail
    AirfoilParameter dataVTail[verticalTailSections.size()];
    sectionType = "vertical_tail";
    for(int i = 0; i < verticalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudZ(verticalTail, verticalTailSections[i], sectioningType);
        extract.translateSection(section);
        extract.derotateSection(section);
        int indexTrailingEdge = section.findLeadingTrailingEdge()[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 0.5);
        if(i == 0) {
            section.generateMissingAirfoilParameter(sectionType, section.getAirfoilParameter().posLeadingEdge);
        }
        else {
            section.generateMissingAirfoilParameter(sectionType, dataVTail[0].posLeadingEdge);
        }
        AirfoilFitter fitAirfoil(section, sourceFolder);
        fitAirfoil.initiateFitting();
        dataVTail[i] = section.getAirfoilParameter();
        std::cout << "Writing vertical tail complete...\n";
    }

    io.writingWingDataInCSV(aircraftDataFile, dataVTail, sectionType, verticalTailSections.size());


    return 0;
}