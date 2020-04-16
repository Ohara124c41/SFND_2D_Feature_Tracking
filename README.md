


# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Performance Analysis

| Detector + Descriptor | Num. KP | Matched KP |Time [ms]
|:--------:| :-------------:|:-------------:|:-------------:|
| AKAZE+AKAZE| 1670|1172| 845.583|
| AKAZE+BRIEF| 1670 |1087| 877.726|
| AKAZE+BRISK | 1670|1110| 874.885|
| AKAZE+FREAK| 1670|973| 804.114|
| AKAZE+ORB| 1670|918| 884.853|
| AKAZE+SIFT| 1670|1270| 839.757|
| BRISK+AKAZE|nul|nul|nul|
| BRISK+BRIEF|2762|1344|3783.89|
| BRISK+BRISK|2762|1298|3774.18|
| BRISK+FREAK|2762|1093|3778.11|
| BRISK+ORB|2762|933|3777.22|
| BRISK+SIFT|2762|1646|3766.72|
| FAST+AKAZE|nul|nul|nul|
| FAST+BRIEF|4094|2178|49.1297|
| FAST+BRISK|4094|1832|69.2639|
| FAST+FREAK|4094|1566|69.0111|
| FAST+ORB|4094|2061|48.4005|
| FAST+SIFT|4094|2782|52.0133|
| HARRIS+AKAZE|nul|nul|nul|
| HARRIS+BRIEF|248|141|199.716|
| HARRIS+BRISK|248|121|200.885|
| HARRIS+FREAK|248|123|168.128|
| HARRIS+ORB|248|145|194.823|
| HARRIS+SIFT|248|163|193.836|
| ORB+AKAZE|nul|nul|nul|
| ORB+BRIEF|1161|450|84.0515|
| ORB+BRISK|1161|649|84.7304|
| ORB+FREAK|1161|649|87.2563|
| ORB+ORB|1161|530|83.2771|
| ORB+SIFT|1161|763|83.0927|
| SHI-TOMASI+AKAZE|nul|nul|nul|
| SHI-TOMASI+BRIEF|1179|816|166.881|
| SHI-TOMASI+BRISK|1179|690|182.826|
| SHI-TOMASI+FREAK|1179|574|144.65|
| SHI-TOMASI+ORB|1179|768|167.014|
| SHI-TOMASI+SIFT|1179|927|155.641|
| SIFT+AKAZE|nul|nul|nul|
| SIFT+BRIEF|1386|597|1321.9|
| SIFT+BRISK|1386|536|1099.49|
| SIFT+FREAK|1386|506|1297.85|
| SIFT+ORB|memErr|memErr|memErr|
| SIFT+SIFT|1386|800|1045.31|


