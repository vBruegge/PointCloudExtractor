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

# Nix
A nix flake was added to the repository. Nix is a package manager which has to be installed natively on your distribution. In this case, you can use nix to build all example projects. This has the advantage that no other dependency has to be installed on the machine.
To use the building operation of nix, change the directory of a terminal to your git and type `nix build`. The executables will be found in `result/bin`.
