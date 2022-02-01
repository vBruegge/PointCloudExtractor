#include <string>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"
#include "AirfoilFitter.hpp"

int main (int argc, char** argv)
{
    std::string pointCloudFile = argv[1];
    IOHandler io;
    io.convertTXTToPCDFile(pointCloudFile);

    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, false);

    std::string sectionFilename = argv[2];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getPointCloudWithoutNormals());
        sectionFilename = "section-generation.txt";
    }

    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);

    std::string filename = argv[3];
    std::vector<Eigen::Vector2d> reference = io.readAirfoilDATFile(filename);
    float xPosFirstReference = 0.4;
    float xPosSecondReference = 0.7;
    pcl::PointXYZ firstReference, secondReference;
    for(int i = reference.size()/2; i < reference.size(); i++) {
        if(abs(reference[i][0]-xPosFirstReference) < 0.01) {
            firstReference.x = 0;
            firstReference.y = reference[i][0];
            firstReference.z = reference[i][1];
        }
        if(abs(reference[i][0]-xPosSecondReference) < 0.01) {
            secondReference.x = 0;
            secondReference.y = reference[i][0];
            secondReference.z = reference[i][1];
        }
    }
    
    float positionFlap = std::stof(argv[3]);
    MorphingWingParameter data[wingSections.size()];
    GeometryExtractor extract;
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(op.getPointCloudWithNormals(), wingSections[i], 2);
        extract.derotateToReferencePoints(section, firstReference, secondReference);
        data[i] = section.getMorphingWingParameter();
        int indexTrailingEdge = section.findLeadingTrailingEdge(section.getFoil())[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, positionFlap);
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.replaceMorphedFlap(reference);
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
    io.writingMorphingWingDataInCSV(aircraftDataFile, data, wingSections.size());
    aircraftDataFile.close();
}