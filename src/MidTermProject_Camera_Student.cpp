/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    double totalTime = 0.0;
    int totalKeyPoints = 0;
    int totalMatchedKeyPoints = 0;

    /* MAIN LOOP OVER ALL IMAGES */
    string detectorType = "SIFT"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() == dataBufferSize)
            dataBuffer.pop_front();
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        // cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */
        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType

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
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // Store KPs for only the preceding vehicle
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

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts) {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") ==
                0) {
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // Push KP && descriptor to end of buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        // cout << "#2 : DETECT KEYPOINTS done" << endl;
        totalKeyPoints += keypoints.size();

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors,
                      descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // Push descriptors to end of buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;
        // cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1)
        {

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorTypeCategory = "DES_BINARY"; // DES_BINARY, DES_HOG
            if (descriptorType.compare("SIFT")==0) descriptorTypeCategory = "DES_BINARY";
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            t = (double) cv::getTickCount();
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorTypeCategory, matcherType, selectorType);

            totalTime += ((double) cv::getTickCount() - t) / cv::getTickFrequency();
            //// EOF STUDENT ASSIGNMENT

            // Store matches in cached DF
            (dataBuffer.end() - 1)->kptMatches = matches;

            // cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            totalMatchedKeyPoints += matches.size();
            bVis = true;
            if (bVis) {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
                cv::imshow(windowName, matchImg);
                cv::imwrite(imgBasePath + "keypoints/"+detectorType+"_"+descriptorType+"_"+imgNumber.str()+imgFileType, matchImg);
                // cout << "Press key to continue to next image" << endl;
                cv::waitKey(1); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    cout << "detectorType+descriptorType,  " << detectorType << " + " << descriptorType << ",  Num.KP:" << totalKeyPoints << ",  Num.Match:" << totalMatchedKeyPoints << ",  Time:" << totalTime * 1000 / 1.0 << "ms" <<endl;



    return 0;
}
