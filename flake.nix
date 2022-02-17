{
  description = "Geometry extraction";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }: 
    let pkgs = nixpkgs.legacyPackages.x86_64-linux;
  	needed = with pkgs; [
          pcl
          boost
          gsl
          sfml
          eigen
        ];

    in rec {
      devShell.x86_64-linux = pkgs.mkShell rec {
        name = "pointcloudextractor_lls";
        packages = needed ++ [
	  pkgs.gcc
	  pkgs.cmake
          pkgs.pkg-config
	];
      };

      packages =
        let mkExample = xname: pkgs.stdenv.mkDerivation rec {
          name = xname;
	  src = self;
	  nativeBuildInputs = [ pkgs.cmake ];
	  buildInputs = needed;
	  phases = ["unpackPhase" "buildPhase" "installPhase" ];

	  buildPhase = ''
	    cd src/examples/${name}
	    [ -e build ] && rm -rf build
	    mkdir build
	    cd build
	    cmake ..
	    make
	  '';

	  installPhase = ''
	    mkdir -p $out/bin
	    cp ./${name} $out/bin/
	  '';
	};
        in {
          morphingWingExtraction = mkExample "morphingWingExtraction";
      };

      defaultPackage.x86_64-linux = packages.morphingWingExtraction;
    };
}
