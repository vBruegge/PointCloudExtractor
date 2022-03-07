#include <string>

#include "PointCloudOperator.hpp"
#include "GeometryExtractor.hpp"
#include "IOHandler.hpp"
#include "DrawUAV.hpp"
#include "AirfoilFitter.hpp"

int main (int argc, char** argv)
{
    //converts the point cloud in a valid pcd file
    std::string pointCloudFile = argv[1];
    IOHandler io;
    io.convertTXTToPCDFile(pointCloudFile);

    //reads the pcd file, alignes the point cloud and computes normals
    pointCloudFile = pointCloudFile + ".pcd";
    PointCloudOperator op(pointCloudFile, true, false);

    //possibility to generate a new sectioning file with the argument "new"
    std::string sectionFilename = argv[2];
    if(sectionFilename == "new") {
        sectionGenerationGUI(op.getPointCloudWithoutNormals());
        sectionFilename = "section-generation.txt";
    }

    //reads the given sectioning file !all of the vectors are needed!
    std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
    float splittingDistance;
    io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);

    //extract the reference airfoil
    std::string filename = argv[3];
    std::vector<Eigen::Vector2d> reference = io.readAirfoilDATFile(filename);

    //computes the reference points at the given distances
    float xPosFirstReference = 0.8008; //CellSkin, use 0.801998 for Mono
    float xPosSecondReference = 0.418467; //CellSkin, use 0.399124 for Mono
    int indexBeforeFirstReference = 0;
    int indexBeforeSecondReference = 0;

    for(int i = reference.size()/2; i < reference.size(); i++) {
        if(reference[i][0] < xPosFirstReference) {
            indexBeforeFirstReference = i;
        }
        if(reference[i][0] < xPosSecondReference) {
            indexBeforeSecondReference = i;
        }
    }
    Eigen::Vector2d gradient = reference[indexBeforeFirstReference+1]-reference[indexBeforeFirstReference];
    pcl::PointXYZ firstReference(0, xPosFirstReference, reference[indexBeforeFirstReference][1] +
        gradient[1]/gradient[0]*(reference[indexBeforeFirstReference+1][0]-xPosFirstReference));
    gradient = reference[indexBeforeSecondReference+1]-reference[indexBeforeSecondReference];
    pcl::PointXYZ secondReference(0, xPosSecondReference, reference[indexBeforeSecondReference][1] +
    gradient[1]/gradient[0]*(reference[indexBeforeSecondReference+1][0]-xPosSecondReference));
    
    //morphing wing sectioning
    float positionFlap = std::stof(argv[4]);
    MorphingWingParameter data[wingSections.size()];
    GeometryExtractor extract;
    for(int i = 0; i < wingSections.size(); i++) {
        Airfoil section = extract.sectioningCloudX(op.getPointCloudWithNormals(), wingSections[i], 2);
        extract.derotateToReferencePoints(section, firstReference, secondReference);
        data[i] = section.getMorphingWingParameter();
        int indexTrailingEdge = section.findLeadingTrailingEdge()[1];
        extract.deleteTrailingEdge(section, indexTrailingEdge, 1-positionFlap);
        AirfoilFitter fitAirfoil(section);
        fitAirfoil.replaceMorphedFlap(reference);
    }

    std::ofstream aircraftDataFile;
    aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
    io.writingMorphingWingDataInCSV(aircraftDataFile, data, wingSections.size());
    aircraftDataFile.close();
}