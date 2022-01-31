{
  description = "Geometry extraction";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/21.05";
  };

  outputs = { self, nixpkgs }: 
    let pkgs = nixpkgs.legacyPackages.x86_64-linux;
    in {
      devShell.x86_64-linux = pkgs.mkShell rec {
        name = "pointcloudextractor_lls";

	shellHook = ''
          export LD_LIBRARY_PATH=$(nixGLIntel printenv LD_LIBRARY_PATH):$LD_LIBRARY_PATH
	  '';

        packages = with pkgs; [
          gcc
          cmake
          pcl
          boost
          gsl
          sfml
          pkg-config
          eigen
          ]))
        ];
      };
    };
}
