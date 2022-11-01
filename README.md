# MBot Omni Autonomy Code

## Usage

The simplest way to build is by running: `./build.sh`. To build on a laptop, use `./build.sh -l`.

To run:
```
cd system_compilation
./launch_botlab.sh
```

## Directories

- mbot
    - where all the base code resides

- data
    - where data needed to run parts of the assignment are located
    - log files and target files for SLAM and exploration are here
    - data logged or generated ought to reside in that directory

- docker
    - houses some possibly outdated docker files for building and running the code in a container

- scripts
    - contains all supplementary bash files as well as python scripts.


## Files

- CMakeLists.txt
    - compiles the mbot codebase code into executables

- build.sh
    - facilitates the system compilation. Different files require access to resources like lcm message compilations or specifically located bash files, this script deals with said organizations for you.
    - run `./build.sh` to generate a system_compilation self contained folder.
