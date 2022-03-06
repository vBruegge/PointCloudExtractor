# PointCloudExtractor_LLS

# Introduction
This is a library for extracting geometries of a 3D scan. The main focus lies on the extraction of aerospace structures like foil geometries, but in general is is applicable on all forms of 3D-scans. It is capable of orienting a point cloud and extracting before defined sections as well as post-processing the data for usage in low fidelity tools like XFLR5.

# Used librarys

- Point Cloud Library (PCL)
- GNU Scientific Library (GSL) v2.7
- Simple and Fast Multimedia Library (SFML)

# Examples
Two example cases were examined. First of all, an extraction of a complete aircraft was executed. In this case, fuselage, wing and tail geometries are extracted of the point cloud. In addition, geometric characteristics of the aircraft like the dihedral or the twist of the wing and tail is computed. All extracted foils are fitted with a CST parametrization using Bernstein polynomials and a shape function.

The second example is the extraction of a morphing wing. In this case, not the leading and trailing edge of the extracted foil can be used for rotation and aligning purposes. So, extra references have to be applied before the scanning process which are then used to fit the extracted wing on the reference airfoil. The references are deleted after this fitting.

The examples can be found in `src/examples`.

## uavExtraction

The code for the UAV extraction can be found in `src/examples/uavExtraction`.

The programm requires four inputs:
- the point cloud which should be sectioned as TXT-File in the build folder
- a boolean (y/n) if the fuselage is greater than the wing (e.g. jets)
- the name of the section generation file in the build folder or a "new"
- type of sectioning (0 for no flaps rotation, 1 for flap rotation)

`./src/examples/uavExtraction/build/uavExtraction 15tol.txt n section-generation.txt 0`

This would result in the extraction of the point cloud named "15tol.txt" which was copied in the build folder beforehand. It is a normal configuration, so the wingspan is greater than the fuselage. Therefore no extra rotation is needed (n). The sections are predefined in "section-generation.txt" (also in the build folder). The aircraft was scanned without the flaps actuated, therefore the rotation logic is skipped.

`./src/examples/uavExtraction/build/uavExtraction lizard.txt y new 1`

This would result of the point cloud named "lizard.txt". It was copied in the build folder and is a jet with a significant greater fuselage as the wingspan. Therefore a extra rotation is needed (y). There is no sectioning file yet, a new one has to be generated (new) using a GUI. The flaps were actuated in the scanning process and should be derotated (1).

## morphingWingExtraction

The code for the morphing wing extraction can be found in `src/examples/morphingWingExtraction`.

The programm requires four inputs:
- the point cloud which should be sectioned as TXT-File in the build folder
- the name of the section generation file in the build folder or a "new"
- the name of the reference foil in the build folder
- the position where the extracted foil should be replaced by the reference foil (e.g. flap position)

`./src/examples/morphingWingExtraction/build/morphingWingExtraction DemoMono.txt section-generation.txt B106_optairfoil.dat 0.84`

This results in the sectioning of the point cloud "DemoMono.txt". The position where to section are specified in the "section-generation.txt". The reference airfoil is named "B106_optairfoil.dat". The flap should be replaced by the reference foil, its position is 0.84.

# Nix
A nix flake was added to the repository. Nix is a package manager which has to be installed natively on your distribution. In this case, you can use nix to build all example projects. This has the advantage that no other dependency has to be installed on the machine.
To use the building operation of nix, change the directory of a terminal to your git and type `nix build`. The executables will be found in `result/bin`.
