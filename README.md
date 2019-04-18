# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Basic Build Instructions

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.
Visual Studio is also supported using vcpckg. 

### gcc

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Visual Studio

For setting up the environment for Visual Studio under Windows, follow these steps (credits go to [fkeidel](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/blob/master/VisualStudio/README.md)):

1. Install cmake
    * Download and run windows installer (see https://cmake.org/download/)

2. Install make
    * Download setup from   http://gnuwin32.sourceforge.net/packages/make.htm
    * Select 'Complete package, except sources - Setup'
    * Run downloaded setup

3. Clone and install vcpkg
    * The install script used in the next step will asume that you installed vckpgk in c:\\vcpkg. You can choose another location, but then you have to adapt VcPkgDir in line 13 in install-windows.bat
    * cd c:\\
    * git clone https://github.com/Microsoft/vcpkg.git
    * cd vcpkg
    * call bootstrap-vcpkg.bat

4. Adapt and call the install script for windows
    * cd to directory ide_profiles\\VisualStudio
    * Open install-windows.bat and adjust lines 5 to 7 to the   settings you will use when building your Visual Studio project    (platform, toolset, buildtype)
    * You could also pass these settings as command line arguments  to install-windows.bat
    * If you have more than one toolset installed, comment line 14  and uncomment line 15
    * call install-windows.bat
    * the install scipt will
        * set the build parameters for the libraries to install     (platform, toolset, buildtype)
        * use vcpkg to download, build and install uWebSockets
            * it will download the latest version of uWebSockets

5. Open solution and adapt toolset settings
    * Open Localisation.sln
    * Open project properties
    * Adapt target platform version and platform toolset (use the   same setting that you used in the install script)

6. Build project in Visual Studio
    * Build the project for the platform and buildtype you used in the install script
