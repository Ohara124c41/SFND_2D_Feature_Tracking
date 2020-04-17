


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

## Tasks
- [x] MP.0 Mid-Term Report
- [x] MP.1 Data Buffer Optimization
- [x] MP.2 Keypoint Detection
- [x] MP.3 Keypoint Removal
- [x] MP.4 Keypoint Descriptors
- [x] MP.5 Descriptor Matching
- [x] MP.6 Descriptor Distance Ratio
- [x] MP.7 Performance Evaluation 1
- [x] MP.8 Performance Evaluation 2
- [x] MP.9 Performance Evaluation 3

## Assignment
### MP.0 Mid-Term Report
Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

### MP.1 Data Buffer Optimization
Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

```c
DataFrame frame;
frame.cameraImg = imgGray;
if (dataBuffer.size() == dataBufferSize)
    dataBuffer.pop_front();
dataBuffer.push_back(frame);
```
### MP.2 Keypoint Detection
Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
```c
double t = (double) cv::getTickCount();

if (detectorType.compare("SHITOMASI") == 0) {
    detKeypointsShiTomasi(keypoints, imgGray, bVis);
}
else if (detectorType.compare("HARRIS") == 0) {
    detKeypointsHarris(keypoints, imgGray, bVis);
}
else {
    detKeypointsModern(keypoints, imgGray, detectorType, bVis);
}

totalTime += ((double) cv::getTickCount() - t) / cv::getTickFrequency();
```
### MP.3 Keypoint Removal
Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
```c
bool bFocusOnVehicle = true;
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle) {
    vector<cv::KeyPoint> filteredKeypoints;
    for (auto kp : keypoints) {
        if (vehicleRect.contains(kp.pt))
            filteredKeypoints.push_back(kp);
    }
    keypoints = filteredKeypoints;
}
```
### MP.4 Keypoint Descriptors

Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
```c
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType) {
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0) {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0) extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    else if (descriptorType.compare("ORB") == 0) extractor = cv::ORB::create();
    else if (descriptorType.compare("FREAK") == 0) extractor = cv::xfeatures2d::FREAK::create();
    else if (descriptorType.compare("AKAZE") == 0) extractor = cv::AKAZE::create();
    else if (descriptorType.compare("SIFT") == 0) extractor = cv::xfeatures2d::SIFT::create();
    else throw invalid_argument("Unknown descriptorType" + descriptorType);

```
### MP.5 Descriptor Matching
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
```c
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType,
                      std::string selectorType) {
    // Init Config
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    int normType;
    if (descriptorType.compare("DES_BINARY")) normType = cv::NORM_HAMMING;
    else if (descriptorType.compare("DES_HOG")) normType = cv::NORM_L2;
    else throw invalid_argument("Unknown descriptorType " + descriptorType);

    if (matcherType.compare("MAT_BF") == 0) matcher = cv::BFMatcher::create(normType, crossCheck);
    else if (matcherType.compare("MAT_FLANN") == 0){
        if(normType==cv::NORM_HAMMING)
        {
            const cv::Ptr<cv::flann::IndexParams>& indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
            matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams);
        }
        else matcher = cv::FlannBasedMatcher::create();
    }
    else throw invalid_argument("Unknown matcherType " + matcherType);

    // Implement optimal matching
    if (selectorType.compare("SEL_NN") == 0) { // Optimal NN

        matcher->match(descSource, descRef, matches); // Select optimal match
    } else if (selectorType.compare("SEL_KNN") == 0) {
        int k = 2;
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, k);
    }
}
```
### MP.6 Descriptor Distance Ratio
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
```c
// Utilize descriptor distance ratio to filter KNN result
double minDistRatio = 0.8;
for (auto matchPair : knn_matches) {
    if (matchPair[0].distance < minDistRatio * matchPair[1].distance) {
        matches.push_back(matchPair[0]);
    }
```
### MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

Please see the table below. "Num. KP" refers to the number of keypoints for all 10 images.

### MP.8 Performance Evaluation 2

Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

Please see the table below. "Match KP" refers to the number of matched keypoints for all 10 images.

### MP.9 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Please see the table below. "Time [ms]" refers to the time (in milliseconds) to perform keypoint detection and descriptor extraction.


## Performance Analysis

The required performance metrics have been documented below. Additionally, these values have been normalized for better insight into the performance for 'Detector + Descriptor' combinations. Three cases are normalized:


$\frac{KP_{matched}}{KP_{number}}  \ \ \quad (1)$

$\frac{KP_{number}}{KP_{maximum}} \quad (2)$

$\frac{t_{best}}{t_{actual}} \quad \quad \ \ \ (3)$

Equation $(1)$ takes the total matched keypoints and is divided by the number of keypoints in the combination (higher is better).

Equation $(2)$ takes the total number of keypoints and is divided by the maximum number of detected keypoints for the specific detector and descriptor combination (higher is better).

Equation $(3)$ takes the best time and is divided by the actual time for the specific detector and descriptor combination (higher is better).

With these three equations, a fourth equation is created to create a tradeoff space matrix:
$$[\frac{KP_{matched}}{KP_{number}}  * \frac{KP_{number}}{KP_{maximum}} *\frac{t_{best}}{t_{actual}}]*100(3)$$

This results in an objective evaluation for the performance values.

| Detector + Descriptor | Num. KP | Match KP |Time [ms]|Match KP /Num. KP|Num. KP/Max. KP|Best Time/Actual Time |Score*|
|:--------:| :-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|
| AKAZE+AKAZE| 1670|1172| 845.583|0.701|0.408|0.057|1.637
| AKAZE+BRIEF| 1670 |1087| 877.726|0.650|0.408|0.055|1.462
| AKAZE+BRISK | 1670|1110| 874.885|0.665|0.408|0.055|1.501
| AKAZE+FREAK| 1670|973| 804.114|0.583|0.408|0.060|1.432
| AKAZE+ORB| 1670|918| 884.853|0.550|0.408|0.055|1.227
| AKAZE+SIFT| 1670|1270| 839.757|0.760|0.408|0.058|1.787
| BRISK+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| BRISK+BRIEF|2762|1344|3783.89|0.487|0.675|0.013|0.420
| BRISK+BRISK|2762|1298|3774.18|0.470|0.675|0.013|0.407
| BRISK+FREAK|2762|1093|3778.11|0.396|0.675|0.013|0.342
| BRISK+ORB|2762|933|3777.22|0.338|0.675|0.013|0.292
| BRISK+SIFT|2762|1646|3766.72|0.596|0.675|0.013|0.517
| FAST+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| FAST+BRIEF|4094|2178|49.1297|0.532|1|0.985|52.410
| FAST+BRISK|4094|1832|69.2639|0.448|1|0.699|31.306
| FAST+FREAK|4094|1566|69.0111|0.383|1|0.701|26.861
| FAST+ORB|4094|2061|48.4005|0.503|1|1.000|50.300
| FAST+SIFT|4094|2782|52.0133|0.680|1|0.931|63.277
| HARRIS+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| HARRIS+BRIEF|248|141|199.716|0.569|0.061|0.242|0.841
| HARRIS+BRISK|248|121|200.885|0.488|0.061|0.241|0.717
| HARRIS+FREAK|248|123|168.128|0.496|0.061|0.288|0.871
| HARRIS+ORB|248|145|194.823|0.585|0.061|0.248|0.887
| HARRIS+SIFT|248|163|193.836|0.657|0.061|0.250|1.001
| ORB+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| ORB+BRIEF|1161|450|84.0515|0.388|0.284|0.576|6.345
| ORB+BRISK|1161|649|84.7304|0.559|0.284|0.571|9.069
| ORB+FREAK|1161|649|87.2563|0.559|0.284|0.555|8.806
| ORB+ORB|1161|530|83.2771|0.456|0.284|0.581|7.527
| ORB+SIFT|1161|763|83.0927|0.657|0.284|0.582|10.869
| SHI-TOMASI+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| SHI-TOMASI+BRIEF|1179|816|166.881|0.692|0.288|0.290|5.780
| SHI-TOMASI+BRISK|1179|690|182.826|0.585|0.288|0.265|4.460
| SHI-TOMASI+FREAK|1179|574|144.65|0.487|0.288|0.335|4.693
| SHI-TOMASI+ORB|1179|768|167.014|0.651|0.288|0.290|5.433
| SHI-TOMASI+SIFT|1179|927|155.641|0.786|0.288|0.311|7.039
| SIFT+AKAZE|nul|nul|nul|nul|nul|nul|nul|
| SIFT+BRIEF|1386|597|1321.9|0.431|0.339|0.037|0.535
| SIFT+BRISK|1386|536|1099.49|0.387|0.339|0.044|0.578
| SIFT+FREAK|1386|506|1297.85|0.365|0.339|0.037|0.461
| SIFT+ORB|memErr|memErr|memErr|memErr|memErr|memErr|memErr|
| SIFT+SIFT|1386|800|1045.31|0.577|0.339|0.046|0.906


## Detector + Descriptor Selection

As a result of the table above, the following combinations have been selected:


| Detector + Descriptor | Motivation |
|:--------:| :-------------|
| FAST+SIFT| Highest Tradeoff Space: 63.277 |
| FAST+ORB| Fastest Response Time: 48.400 |
| FAST+BRIEF| Robust and fulfils nonfunctional requirements well (COTS, ease of implementation) |
