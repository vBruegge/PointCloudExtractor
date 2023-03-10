# PointCloudExtractor

# Introduction
This is a library for extracting geometries of a 3D scan. The main focus lies on the extraction of aerospace structures like foil geometries, but in general is is applicable on all forms of 3D-scans. It is capable of orienting a point cloud and extracting before defined sections as well as post-processing the data for usage in low fidelity tools like XFLR5.

# Used librarys

- Point Cloud Library (PCL)
- GNU Scientific Library (GSL) v2.7
- Simple and Fast Multimedia Library (SFML)

For the installation of the libraries on an Ubuntu machine use this:
```
sudo apt-get install libpcl-dev libgsl-dev libsfml-dev
```
Note: this is not necessary if you use the nix build option instead (see below).

# Examples
Two example cases were examined. First of all, an extraction of a complete aircraft was executed. In this case, fuselage, wing and tail geometries are extracted of the point cloud. In addition, geometric characteristics of the aircraft like the dihedral or the twist of the wing and tail is computed. All extracted foils are fitted with a CST parametrization using Bernstein polynomials and a shape function.

The second example is the extraction of a morphing wing. In this case, not the leading and trailing edge of the extracted foil can be used for rotation and aligning purposes. So, extra references have to be applied before the scanning process which are then used to fit the extracted wing on the reference airfoil. The references are deleted after this fitting.

The examples can be found in `examples/`.

### uavExtraction

The code for the UAV extraction can be found in `examples/uavExtraction`.  
The point cloud is supposed to be in a directery called "Scans" in the root directory of this repository. If you have generated a sectioning file beforehand, this should also be in this directory. Two examplary program execution are presented below, one if you have build the project with the nix flake and one without. If you have built the tool using the nix flake, your executeable can be found in `result/bin`.

The programm requires five inputs:
- the absolute path to the root directory of the repository (use $PWD if you execute the program from the root directory)
- the point cloud which should be sectioned as TXT-File in the "Scans" folder
- a boolean (y/n) if the fuselage is greater than the wing (e.g. jets)
- the name of the section generation file in the "Scans" folder or a "new"
- type of sectioning (0 for no flaps rotation, 1 for flap rotation)
```
./result/bin/uavExtraction $PWD 15tol.txt n section-generation.txt 0
```

This would result in the extraction of the point cloud named "15tol.txt" which was copied in the "Scans" folder beforehand. It is a normal configuration, so the wingspan is greater than the fuselage. Therefore no extra rotation is needed (n). The sections are predefined in "section-generation.txt" (also in the "Scans" folder). The aircraft was scanned without the flaps actuated, therefore the rotation logic is skipped.

**Important:** Unfortunately, there is a driver problem with glib which is a dependency of the GUI. If you want to use the GUI for defining the sections, you **can not** build your executable with nix. Instead you have to install the needed libraries (take care about the right versions) and build the project using cmake. In this case your workflow would be:
```
cd examples/uavExtraction
mkdir build
cd build
cmake ..
make
./uavExtraction ../../.. lizard.txt y new 1
```

This would result in the sectioning of the point cloud named "lizard.txt". It was copied in the "Scans" folder and is a jet with a significant greater fuselage as the wingspan. Therefore a extra rotation is needed (y). There is no sectioning file yet, a new one has to be generated (new) using a GUI. The flaps were actuated in the scanning process and should be derotated (1).

This problem can also occur on some distributions like Arch - testing of the GUI was done using Ubuntu 20.4. 

### morphingWingExtraction

**Preliminary:** For the extraction of a morphing wing, additional references has to be added on the wing (segment). These are rectangular profiles which will be attached to the bottom side of the wing. Two references are needed for the rotation and scaling processes. The x-position of the edges laying closest to the trailing edge will be used as reference/scaling positions.

The code for the morphing wing extraction can be found in `examples/morphingWingExtraction`.
The point cloud is supposed to be in a directery called "Scans" in the root directory of this repository. If you have generated a sectioning file beforehand, this should also be in this directory. Also the reference airfoil is saved in this folder. Two examplary program execution are presented below, one if you have build the project with the nix flake and one without. If you have built the tool using the nix flake, your executeable can be found in `result/bin`.

**Important:** Before building, please verify the position of the references in the 'examples/morphingWingExtraction/morphingWingExtraction.cpp'. They can be found in line 47 and 48.

The programm requires five inputs:
- the absolute path to the root directory of the repository (use $PWD if you execute the program from the root directory)
- the point cloud which should be sectioned as TXT-File in the Scans folder
- the name of the section generation file in the Scans folder or a "new"
- the name of the reference foil in the Scans folder
- the position where the extracted foil should be replaced by the reference foil (e.g. flap position)
```
./result/bin/morphingWingExtraction $PWD DemoMono.txt section-generation.txt B106_optairfoil.dat 0.84
```

This results in the sectioning of the point cloud "DemoMono.txt". The position where to section are specified in the "section-generation.txt". The reference airfoil is named "B106_optairfoil.dat". The flap should be replaced by the reference foil, its position is 0.84.

**Important:** Unfortunately, there is a driver problem with glib which is a dependency of the GUI. If you want to use the GUI for defining the sections, you **can not** build your executable with nix. Instead you have to install the needed libraries (take care about the right versions) and build the project using cmake. Please refer to the section above for an example.

# Nix
A nix flake was added to the repository. Nix is a package manager which has to be installed natively on your machine. In this case, you can use nix to build all example projects. This has the advantage that no other dependency is needed and no libraries have to be installed on the machine.

### Installation on Ubuntu
For installing the nix package manager run this
```
$ sh <(curl -L https://nixos.org/nix/install) --no-daemon
```
For being able to handle the nix flake, you have to install this package
```
nix-env -iA nixpkgs.nixFlakes
```
and add the following line to `~/.config/nix/nix.conf`
```
experimental-features = nix-command flakes
```
You can verify your setup with
```
nix flake --help
```
### Installation on other distribution
If you use another distribution, you sould have enough knowledge to get this done without an instruction, if not, then refer to your mighty friend google.

### Building with the nix flake
To use the building operation of nix, there are two possible ways. Please be aware, that you can only have one build (either the uavExtraction or the morphingWingExtraction). If you want to switch from one tool to the other, rebuild the wanted package.
1. Clone the repository beforehand and then use nix for building:
    In this case, the repository is cloned to your machine using e.g. `git clone git@gitlab.lrz.de:000000000149A72A/pointcloudextractor_lls.git` (SSH). Then go to the root of the git repository in an Terminal and run
    - for building the aircraft extraction example:
    ```
    nix build .#uavExtraction
    ```
    - for building the morphing wing extraction example
    ```
    nix build .#morphingWingExtraction
    ```
2. Building the git repository without cloning:
    In this case, the repository is cloned by nix and builded in the same step. Run
    - for building the aircraft extraction example:
    ```
    nix build git+https://gitlab.lrz.de/000000000149A72A/pointcloudextractor_lls.git?ref=main#uavExtraction
    ```
    - for building the morphing wing extraction example
    ```
    nix build git+https://gitlab.lrz.de/000000000149A72A/pointcloudextractor_lls.git?ref=main#morphingWingExtraction
    ```

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

# Disclaimer

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
