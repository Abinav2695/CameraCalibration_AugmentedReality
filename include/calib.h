/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Calibration Header File
*/

#ifndef CALIB_H
#define CALIB_H


bool find_corners(cv::Mat &image, cv::Mat &drawFrame, std::vector<cv::Point2f> &corners, cv::Size boardSize, bool show_corners);
void generateKnownBoardPos(float sqEdgeLength, std::vector<cv::Point3f> &worldPoints, cv::Size boardSize);
int save_calibration(std::vector<cv::Mat> &calibImages, std::string cameraName, cv::Size boardSize, float sqEdgeLength, std::string fileName, bool printCalibrationData);
int write_camera_properties_to_file(std::string fileName, cv::Mat cameraMatrix, cv::Mat distCoeffs, 
                                    std::string cameraName, cv::Size cameraResolution, double reprojection_error, 
                                    int reset_file);
#endif