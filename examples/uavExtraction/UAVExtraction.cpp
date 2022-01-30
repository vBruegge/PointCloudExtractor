#include <string>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"

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
    PointCloudOperator op(filename + ".pcd", fuselageGreaterThanWing);

    std::string sectionFilename = argv[3];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getCloudWithoutNormals());
        section = "section-generation.txt";
    }
    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);

    pcl::PointCloud<pcl::PointNormal>::Ptr wing(new pcl::PointCloud<pcl::PointNormal);
    pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail(new pcl::PointCloud<pcl::PointNormal); 
    pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail(new pcl::PointCloud<pcl::PointNormal);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage = op.getCloudWithoutNormals();

    op.splitCloudInWingAndTail(wing, horizontalTail, verticalTail, splittingDistance);

    GeometryExtractor extract;
    FuselageParameter dataFuselage[fuselageSections.size()];
    for(int i = 0; i < fuselageSections.size(); i++) {
        Fuselage section = extract.sectioningCloudY(fuselage, fuselageSections[i]);
        FuselageFitter fitFuselage(section);
        fitFuselage.superellipseFit();
        dataFuselage[i] = section.getFuselageParameter();
    }
    io.writingFuselageDataInCSV(dataFuselage);

    bool flapRotationNeeded = true;
    std::string flapRotated = argv[4];
    if(flapRotated == "n")
        flapRotationNeeded = false;
    AirfoilParameter dataWing[wingSections.size()];
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(wing, wingSections[i], flapRotationNeeded);
        extract.translateSection(section);
        extract.derotateSection(section);
        extract.deleteTrailingEdge(section);
        if(i == 0) {
            section.generateMissingAirfoilParameter("wing", 0, wingSection[i]);
        }
        else {
            section.generateMissingAirfoilParameter("wing", dataWing[0].offset, dataWing[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataWing[i] = section.getAirfoilParameter();
    }

    AirfoilParameter dataHTail[horizontalTailSections.size()];
    for(int i = 0; i < horizontalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(horizontalTail, horizontalTailSections[i], flapRotationNeeded);
        extract.translateSection(section);
        extract.derotateSection(section);
        extract.deleteTrailingEdge(section);
        if(i == 0) {
            section.generateMissingAirfoilParameter("horizontal_tail", 0, horizontalTailSections[i]);
        }
        else {
            section.generateMissingAirfoilParameter("horizontal_tail", dataHTail[0].offset, dataHTail[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataHTail[i] = section.getAirfoilParameter();
    }

    AirfoilParameter dataVTail[verticalTailSections.size()];
    for(int i = 0; i < verticalTailSections.size(); i++) {
        Airfoil section = extract.sectioningCloudZ(verticalTail, verticalTailSections[i], flapRotationNeeded);
        extract.translateSection(section);
        extract.derotateSection(section);
        extract.deleteTrailingEdge(section);
        if(i == 0) {
            section.generateMissingAirfoilParameter("vertical_tail", 0, verticalTailSections[i]);
        }
        else {
            section.generateMissingAirfoilParameter("vertical_tail", dataVTail[0].offset, dataVTail[0].cuttingDistance);
        }
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.initiateFitting();
        dataVTail[i] = section.getAirfoilParameter();
    }
    return 0;
}