{
  description = "Dev shell for ACS-KM (Ant Colony System Java project)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
  let
    system = "x86_64-linux";
    pkgs = import nixpkgs { inherit system; };
  in {
    devShells.${system}.default = pkgs.mkShell {
      name = "acs-km-shell";

      packages = [
        pkgs.openjdk11
        pkgs.findutils
      ];

      shellHook = ''
        export CLASSPATH="$PWD/bin:$PWD/lib/*"
        export JAVA_TOOL_OPTIONS="-Xmx2G"

        alias build="javac -d bin -cp 'lib/*' \$(find src -name '*.java')"
        alias run="java -cp \"$CLASSPATH\" aco.Controller"

        if command -v java >/dev/null; then
          echo "Java: $(java -version 2>&1 | head -n 1)"
        fi

        echo "CLASSPATH: $CLASSPATH"
        echo "Commands:"
        echo "  build   -> compile sources"
        echo "  run     -> run aco.Controller"
      '';
    };
  };
}
