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

    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, fuselageGreaterThanWing);

    std::string sectionFilename = argv[2];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getPointCloudWithoutNormals());
        sectionFilename = "section-generation.txt";
    }

    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);
    
    float positionFlap = argv[3];
    pcl::PointXYZ firstReference, secondReference;
    MorphingWingParameter data[wingSections.size()];
    GeometryExtractor extract;
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(op.getPointCloudWithNormals(), wingSections[i], 2);
        extract.derotateToReferencePoints(foil, firstReference, secondReference);
        data[i] = foil.getMorphingWingParameter();
        int indexTrailingEdge = section.findLeadingTrailingEdge(section.getFoil())[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, positionFlap);
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
    io.writingMorphingWingDataInCSV(aircraftDataFile, data, wingSections.size());
    aircraftDataFile.close();
}