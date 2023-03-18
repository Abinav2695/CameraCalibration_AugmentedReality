/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Calibration Header File
*/

#ifndef CALIB_H
#define CALIB_H


int find_corners(cv::Mat &image, cv::Mat &image_corners, std::vector<cv::Point2f> &corners, cv::Size patternSize);
void generateKnownBoardPos(float sqEdgeLength, std::vector<cv::Point3f> &corners, cv::Size patternSize);

#endif