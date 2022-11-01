#!/bin/bash
help()
{
    echo "Build Botlab code."
    echo
    echo "Usage:"
    echo "    -h            Print help and exit."
    echo "    -l            Build for the laptop."
    echo "    -c            Clean up by deleting compiled files."
    echo "    -b            Basic build without creating a system compilation."
}

PACKAGE_SYSTEM=true
MBOT=true
CMAKE_FLAGS=""
ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

while getopts "hlcb" option; do
    case $option in
        h)  # Help
            help
            exit;;
        l)  # Build for the laptop.
            MBOT=false;;
        c)  # Cleanup
            rm -rf system_compilation
            rm -rf build
            exit;;
        b)  # Package
            PACKAGE_SYSTEM=false ;;
        \?) # Invalid.
            echo "Invalid option provided."
            help
            exit;;
    esac
done

if $MBOT ; then
    CMAKE_FLAGS=-DBUILD_ON_BOT=ON
    echo "Building for the MBot."
else
    CMAKE_FLAGS=-DBUILD_ON_BOT=OFF
    echo "Building for laptop."
fi

[ ! -d "build" ] && mkdir build
cd build
cmake $CMAKE_FLAGS .. && make
cd $ROOT_DIR

if $PACKAGE_SYSTEM ; then
    bash $ROOT_DIR/scripts/package_built_system.sh
fi