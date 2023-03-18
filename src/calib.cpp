/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Calibration Functions
   Source file
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "calib.h"

using namespace std;



void generateKnownBoardPos(float sqEdgeLength, vector<cv::Point3f> &corners, cv::Size patternSize) {
    for(int i=0; i<patternSize.height; i++) {
        for(int j=0; j<patternSize.width; j++) {
            corners.push_back(cv::Point3f(j*sqEdgeLength, i*sqEdgeLength, 0));
        }

    }
}


int find_corners(cv::Mat &image, cv::Mat &image_corners, vector<cv::Point2f> &corners, cv::Size patternSize){
   image_corners = image.clone();
   bool found = cv::findChessboardCorners(image, patternSize, corners);
   cv::drawChessboardCorners(image_corners, patternSize, corners, found);
   return 0;
}



