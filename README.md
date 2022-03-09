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

The examples can be found in `examples`.

### uavExtraction

The code for the UAV extraction can be found in `examples/uavExtraction`.  
The following examples assume, that you have build the executable using "by foot", not using flake. If you have used flake, the executable is in the folder `result/bin` and this is where the point cloud is supposed to go (maybe you have to adjust the permissions to copy something inside).

The programm requires four inputs:
- the point cloud which should be sectioned as TXT-File in the build folder
- a boolean (y/n) if the fuselage is greater than the wing (e.g. jets)
- the name of the section generation file in the build folder or a "new"
- type of sectioning (0 for no flaps rotation, 1 for flap rotation)

`./examples/uavExtraction/build/uavExtraction 15tol.txt n section-generation.txt 0`

This would result in the extraction of the point cloud named "15tol.txt" which was copied in the build folder beforehand. It is a normal configuration, so the wingspan is greater than the fuselage. Therefore no extra rotation is needed (n). The sections are predefined in "section-generation.txt" (also in the build folder). The aircraft was scanned without the flaps actuated, therefore the rotation logic is skipped.

`./examples/uavExtraction/build/uavExtraction lizard.txt y new 1`

This would result of the point cloud named "lizard.txt". It was copied in the build folder and is a jet with a significant greater fuselage as the wingspan. Therefore a extra rotation is needed (y). There is no sectioning file yet, a new one has to be generated (new) using a GUI. The flaps were actuated in the scanning process and should be derotated (1).

### morphingWingExtraction

The code for the morphing wing extraction can be found in `examples/morphingWingExtraction`.

The programm requires four inputs:
- the point cloud which should be sectioned as TXT-File in the build folder
- the name of the section generation file in the build folder or a "new"
- the name of the reference foil in the build folder
- the position where the extracted foil should be replaced by the reference foil (e.g. flap position)

`./examples/morphingWingExtraction/build/morphingWingExtraction DemoMono.txt section-generation.txt B106_optairfoil.dat 0.84`

This results in the sectioning of the point cloud "DemoMono.txt". The position where to section are specified in the "section-generation.txt". The reference airfoil is named "B106_optairfoil.dat". The flap should be replaced by the reference foil, its position is 0.84.

# Nix
A nix flake was added to the repository. Nix is a package manager which has to be installed natively on your machine. In this case, you can use nix to build all example projects. This has the advantage that no other dependency is needed and no libraries have to be installed on the machine.  
If you haven't ascended to the realm of Arch yet, but living in the twilight of Ubuntu or even WSL (2) for Windows, please refer to this instruction for installation:  
https://ariya.io/2020/05/nix-package-manager-on-ubuntu-or-debian

To use the building operation of nix, there are two possible ways. Please be aware, that you can only have one build )(either the uavExtraction or the morphingWingExtraction. If you want to switch from one tool to the other, delete the *result* folder that is created by nix):
1. Clone the repository beforehand and then use nix for building:
    In this case, the repository is cloned to your machine using e.g. `git clone git@gitlab.lrz.de:000000000149A72A/pointcloudextractor_lls.git` (SSH). Then go to the root of the git repository in an Terminal and enter:
    - `nix build .#uavExtraction` for building the aircraft extraction example (if you get error messages about experimental features like "nix-command" and "flakes" being disabled, use the following command to build: `nix --extra-experimental-features nix-command --extra-experimental-features flakes build .#uavExtraction`).
    - `nix build .#morphingWingExtraction`for building the morphing wing extraction example (if you get error messages about experimental features like "nix-command" and "flakes" being disabled, use the following command to build: `nix --extra-experimental-features nix-command --extra-experimental-features flakes build .#morphingWingExtraction`).
2. Building the git repository without cloning:
    In this case, the repository is cloned by nix and builded in the same step. Enter:
    - `nix build git+https://gitlab.lrz.de/000000000149A72A/pointcloudextractor_lls.git?ref=main#uavExtraction` for building the aircraft extraction example
    - `nix build git+https://gitlab.lrz.de/000000000149A72A/pointcloudextractor_lls.git?ref=main#morphingWingExtraction` for building the morphing wing extraction example

The executeables can be found in `result/bin`.

# Section generation file
The section generation file has a specific, pre-defined structure.  Examples of files are presented in the following:
1. For a complete aircraft with a horizontal and vertical tail or a v-tail and a fin, the sectioning file is defined as:
    ```
    h
    -750
    fuselage -650 500 450 300 -75 -180 -350 -500 -750
    wing 110 350 675 1010 1150 1350 1425 1450
    horizontal_tail 50 150 275 360
    vertical_tail -40 0 50 125 145
    ```
    
    Here, the second line defines a distance between the wing and tail. The point cloud is split in two this distance to enable        sectioning the and tail separetely. The following lines define the sectioning distances dependening on the center of mass. For an ease of use, the GUI can be used to generate the file for the first time. Please note, that the lines for the wing and horizontal tail sections can be exchanged.
2. For a complete aircraft with a vertical tail, the sectioning file is defined as:
    ```
    v
    975 
    fuselage -1450 -925 -510 -95 600 1175 1485
    wing 220 550 1150 1180 1800 2350 2400 2450
    horizontal_tail 125 260 300 470 505
    ```
    The sectioning file has a line less as the one above. Since there is no vertical tail or fin, these sections must not be defined. Important is the replacement of the h to a v in the first line.
3. For a non complete aircraft (e.g. without tail, morphing wing...), the sectioning file is defined as:
    ```
    h
    -750
    fuselage -650 500 450 300 -75 -180 -350 -500 -750
    wing 110 350 675 1010 1150 1350 1425 1450
    horizontal_tail
    vertical_tail
    ```
    If no sections of this type is required, just leave the definition of the sectioning distances out. Important: the name is still necessary in the file! For configurations without a tail, the second line has no further significance and can contain any number (e.g. 0). Please note, that the first line defines the number of sectioning types and therefore has to be defined, even if there is no tail section needed.
