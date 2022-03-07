# Morphing Wing Extraction

In this Readme, the functionalities of the morphing wing extraction are presented line by line and specified.

```
std::string pointCloudFile = argv[1];
```

The name of the point cloud file is saved in this line. Here, `argv[1]` is the first input argument.

```
IOHandler io;
io.convertTXTToPCDFile(pointCloudFile);
```
These lines convert the TXT point cloud file to a PCD file which can be used in the Point Cloud Library.

```
pointCloudFile = pointCloudFile + ".pcd";
PointCloudOperator op(pointCloudFile, true, false);
```
This operation creates a member of the PointCloudOperator class which is used for operations which affect the whole point cloud (e.g. aligning). This constructer loads the above created PCD file in the created object, aligns the point cloud and computes normal vectors on ervery point in the cloud. The alignment is defined such that the main axis points to the greatest length of the object oriented bounding box (OOBB). In the case of a conventional aircraft, the greatest length is defined by the wingspan. In the case of jets or as in this example a wing segment, the greatest length is the fuselage or the chord length, respectively. The second input argument of the constructor corrects this alignment and rotates the point again, so the x-axis points in wing direction. The third input argument is also a boolean which defines if the point cloud is a whole aircraft (false because the wing segment is not). The normal vectors are needed for the later sectioning process and the detection of the references of the morphing wing.

```
std::string sectionFilename = argv[2];
if(sectionFilename == "new") {
    sectionGenerationGUI(op.getPointCloudWithoutNormals());
    sectionFilename = "section-generation.txt";
}
```
If a section generation file was already defined (second execute argument), this name is saved. Otherwise, if a "new" was passed, the section generation GUI is opened where sections can be defined. Please note that this functionality was implemented especially for the complete UAV extraction, so some of the functionnalities have no usage in this scenario.

```
std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
float splittingDistance;
io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
        verticalTailSections);
```
Here the section generation file is read. All defined distances of the file are saved in the corresponding vector. Please note, that even if most of the sections won't be used in the extraction process, these vectors still have to be defined.
```
std::string filename = argv[3];
std::vector<Eigen::Vector2d> reference = io.readAirfoilDATFile(filename)
```
In this lines, the reference foil is read and saved as a vector of 2D points. The name of the reference foil is defined by the third input argument.
```
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
```

In this section the to reference points are computed. The first two lines define the (exact) x-position of the reference points. Since it cannot guaranteed that there is a point on the reference foil with this x-coordinates, a linear interpolation between the point before and after the position is done to approximate the reference point as exact as possible. This is needed because small differences in the reference position results in an inaccurate scaling factor.

```
float positionFlap = std::stof(argv[4]);
```
Here, the position of the flap (fourth input argument) is defined. At this position the foil coordinates to the trailing edge are deleted and replaced by the reference foil.

```
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
```
In this section the complete extraction is done. For every section defined in the section file of the wing following steps are executed:
- Sectioning the wing segment (op.getPointCloudWithNormals()) at the given position (wingSections[i]) and detect the reference points (2)
- derotate, scale and translate the morphing wing such that the two detected reference points are congruent.
- save morphing wing parameter to write them in a file later (e. g. scaling factor and rotation angle)
- delete the points from the trailing edge to the given position (in this case remove the complete flap)
- insert the reference foil for the deleted flap and save the point cloud

```
std::ofstream aircraftDataFile;
aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
io.writingMorphingWingDataInCSV(aircraftDataFile, data, wingSections.size());
aircraftDataFile.close();
```
Lastly, the saved morphing wing parameteres are saved in the "aircraftDataFile.csv".
