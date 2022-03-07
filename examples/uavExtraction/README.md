# UAV Extraction

In this Readme, the functionalities of the UAV extraction are presented line by line and specified.

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
std::string fuselageWidth = argv[2];
bool fuselageGreaterThanWing = false;
if(fuselageWidth == "y") {
    fuselageGreaterThanWing = true;
}
```
As second input argument, it is specified if the aircraft is a conventional aircraft or e. g. a jet with a fuselage greater than the wingspan.
```
pointCloudFile = pointCloudFile + ".pcd";
PointCloudOperator op(pointCloudFile, fuselageGreaterThanWing, false);
```
This operation creates a member of the PointCloudOperator class which is used for operations which affect the whole point cloud (e.g. aligning). This constructer loads the above created PCD file in the created object, aligns the point cloud and computes normal vectors on ervery point in the cloud. The alignment is defined such that the main axis points to the greatest length of the object oriented bounding box (OOBB). In the case of a conventional aircraft, the greatest length is defined by the wingspan. In the case of jets or as in this example a wing segment, the greatest length is the fuselage or the chord length, respectively. The second input argument of the constructor corrects this alignment if needed and rotates the point again, so the x-axis points in wing direction. The third input argument is also a boolean which defines if the point cloud is a whole aircraft. This is needed because it is possible that the OOBB alignment does not generate the minimal bounding box for some aircrafts. An additional rotation is defined to guarantee that the minimal box is found. The normal vectors are needed for the later sectioning process and the detection of flaps (if wanted).

```
std::string sectionFilename = argv[2];
if(sectionFilename == "new") {
    sectionGenerationGUI(op.getPointCloudWithoutNormals());
    sectionFilename = "section-generation.txt";
}
```
If a section generation file was already defined (second input argument), this name is saved. Otherwise, if a "new" was passed, the section generation GUI is opened where sections can be defined. Afterwards these sections are saved as TXT-file, so they can be used again in the next extraction process.

```
std::vector<float> fuselageSections, wingSections, horizontalTailSections, verticalTailSections;
float splittingDistance;
io.readSectionFile(sectionFilename, splittingDistance, fuselageSections, wingSections, horizontalTailSections,
    verticalTailSections);
```
Here the section generation file is read. All defined distances of the file are saved in the corresponding vector. Please note, that even if sections won't be used in the extraction process or there is no vertical tail, these vectors still have to be defined.

```
pcl::PointCloud<pcl::PointNormal>::Ptr wing(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr horizontalTail(new pcl::PointCloud<pcl::PointNormal>); 
pcl::PointCloud<pcl::PointNormal>::Ptr verticalTail(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr fuselage = op.getPointCloudWithoutNormals();
op.splitCloudInWingAndTail(wing, horizontalTail, verticalTail, splittingDistance);
```
In this part the point cloud is copied and split in the different parts which are later sectioned. On this occasion, the clouds for the horizontal tail and the wing are halved to reduce the computation time. The vertical tail consists of all points after the defined splitting distance.

```
GeometryExtractor extract;
FuselageParameter dataFuselage[fuselageSections.size()];
for(int i = 0; i < fuselageSections.size(); i++) {
    Fuselage section = extract.sectioningCloudY(fuselage, fuselageSections[i]);
    FuselageFitter fitFuselage(section);
    dataFuselage[i] = fitFuselage.superellipseFit();
    std::cout << "Writing fuselage complete...\n";
}
```
The fuselage is sectioned in this part. The operations are for every fuselage section:
- sectioning the fuselage at the given distance (fuselageSections[i])
- fit the fuselage with a superellipse (optional fits for circles and ellipses were also implemented)
- save the fuselage parameteres (e.g width height, center)

``
std::ofstream aircraftDataFile;
aircraftDataFile.open("../Results/aircraftDataFile.csv", std::fstream::out);
io.writingFuselageDataInCSV(aircraftDataFile, dataFuselage, fuselageSections.size());
```
Here, all saved fuselage parameters are written in the "aircraftDataFile.csv".

```
int sectioningType = std::stoi(argv[4]);
```
As fourth input argument, it is defined if the flaps were actuated and should be rotated or not.
```
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
    AirfoilFitter fitAirfoil(section);
    fitAirfoil.initiateFitting();
    dataWing[i] = section.getAirfoilParameter();
    std::cout << "Writing wing complete...\n";
}

io.writingWingDataInCSV(aircraftDataFile, dataWing, sectionType, wingSections.size());
```
Here, the wing is sectioned. The steps are the following:
- sectioning the wing at the given distance (wingSection[i])
- translating the section to the center of the coordinate system
- derotating the section about the twist angle and saving it
- deleting the trailing edge (important if the wing has a trailing edge width)
- compute missing airfoil parameteres (e. g. chord, offset...)
- fit a bernstein polynomial as CST parametrization on the airfoil (optional a spline fit was implemented)
- save the wing parameters and write them in the "aircraftDataFile.csv"

The horizontal and vertical tail is sectioned analogue. Keep in mind that the sectioning direction of the vertical tail is the z axis, so "sectioningCloudZ()" has to be used instead of "sectioningCloudX()".