# Project done by Abinav Anantharaman and Satwik Shridhar Bhandiwad
# 3 Time travel days used
# CS 5330 Spring 2023 --> Calibration and Augmented Reality @ NEU

## Operating System and IDE:

I have used Ubuntu 20.04.5 LTS OS and Visual Studio Code 1.74.2 for this project. 
The IDE doesn't really matters because I have compiled and built the workspace using cmake and make terminal commands

### Installation of Dependencies
*  cmake, gcc and g++ installation
```bash
sudo apt install cmake gcc g++
```

### Workspace Installation
* Clone this workspace
```bash
cd [$Your_Destination_Folder]
git clone https://github.com/Abinav2695/CameraCalibration_AugmentedReality.git
```
* Build workspace
```bash
cd CameraCalibration_AugmentedReality/
mkdir build
cd build
cmake -S .. -B .
make
```

### Running Executables
* To run application  
```bash
cd build
./bin/main {video source number} {camera model name}
example -> ./bin/main 2 my_webcam
```


# Usage
## To calibrate run following commands:  
 's' : save current image if corners are found in the image and increment counter

 'c' : call calibration function once required number of images have been collected

 'o' : keep playing video feed

## To run AR tasks run following commands:
 'p' : Task5: Project Outside Corners or 3D Axes

 'y' : Task6: Create a Virtual Object --> Pyramid

 'l' : Task6: Create a Virtual ObjectDataset  --> Spatula

 'a' : Extension 1 --> aruco marker detection  

 'b' : Extension 2 --> aruco board detection and print Orientation of camera

 'f' : Extension 3 --> draw cubes on multiple aruco markers
 
 'q' : quit command
