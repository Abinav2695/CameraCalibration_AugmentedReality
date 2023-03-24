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
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "calib.h"

using namespace std;


/**
 * Function to generate a vector of known world coordinates on the checkerboard
 *
 * This function takes the edge length of the checkerboard squares, vector of 3D world coordinates and Board dimensions as input
 *
 * @param sqEdgeLength edge length of the checkerboard squares
 * @param worldPoints  vector to store the world coordinates
 * @param boardSize mxn size of the board
 */
void generateKnownBoardPos(float sqEdgeLength, vector<cv::Point3f> &worldPoints, cv::Size boardSize) {
    for(int i=0; i<boardSize.height; i++) {
        for(int j=0; j<boardSize.width; j++) {
            worldPoints.push_back(cv::Point3f(j*sqEdgeLength, i*(-sqEdgeLength), 0));
        }
    }
}

/**
 * Function to find checkerboard corners in an image
 *
 * This function takes the edge length of the checkerboard squares, vector of 3D world coordinates and Board dimensions as input
 *
 * @param image input image
 * @param drawFrame  image on which to draw the checkerboard corners
 * @param corners vector of 2D image coordinates to store the board corners
 * @param boardSize mxn size of the board
 * @param show_corners draw the board corners if true else dont draw them
 * 
 * @return true if all corners are found else false
 */
bool find_corners(cv::Mat &image, cv::Mat &drawFrame, vector<cv::Point2f> &corners, cv::Size boardSize, bool show_corners){
    drawFrame = image.clone();
    bool found = cv::findChessboardCorners(image, boardSize, corners, 
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if(!found) return false; 
    if(show_corners) cv::drawChessboardCorners(drawFrame, boardSize, corners, found);
    return true;
}

/**
 * Function to write camera properties to a yaml file
 *
 * This function takes filepath, camera intrinsics, distortion coefficients , camera model, camera resolution and reprojection error as input
 * 
 * @param fileName yaml file path
 * @param cameraMatrix  camera intrinsics matrix
 * @param distCoeffs distortion coefficients vector
 * @param cameraName camera model
 * @param cameraResolution camera resolution cv::Size(w,h)
 * @param reprojection_error reprojection error of calibration
 * @param reset_file flag to reset the file while loading camera parameters
 * 
 * @return 0
 */
int write_camera_properties_to_file(std::string fileName, cv::Mat cameraMatrix, cv::Mat distCoeffs, 
                                    std::string cameraName, cv::Size cameraResolution, double reprojection_error, 
                                    int reset_file) {
    char buffer[256];
    char mode[8];
    FILE *fp;

    //std::string filename = cameraName + "_Calibration_Parameters.yaml";
    strcpy(mode, "a");
    if( reset_file ) {
    strcpy( mode, "w" );
    }

    fp = fopen( fileName.c_str(), mode );
    if(!fp) {
        printf("ERROR: Unable to open calib output file %s\n", fileName.c_str());
        return 0;
    }
    
    // if(mode == "a")
    // write the filename and the feature vector to the CSV file
    strcpy(buffer, "image_width:  ");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    strcpy(buffer, std::to_string(cameraResolution.width).c_str());
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "image_height:  ");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    strcpy(buffer, std::to_string(cameraResolution.height).c_str());
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "camera_name:  ");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    strcpy(buffer, cameraName.c_str());
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "reprojection_error:  ");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    strcpy(buffer, std::to_string(reprojection_error).c_str());
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "camera_matrix:");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "  rows:  3");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "  cols:  3");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "  data:  [");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    for(int row=0; row < cameraMatrix.rows ; ++row){
        double *row_ptr = cameraMatrix.ptr<double>(row);
        for(int col=0; col < cameraMatrix.cols ; ++col){
            char tmp[256];
            sprintf(tmp, "%.4f", row_ptr[col]);
            std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
            if(row==2 && col== 2) std::fwrite("]", sizeof(char), 1, fp); // EOL
            else std::fwrite(", ", sizeof(char), 1, fp); // comma separated
        }
    }
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "distortion_model:  plum_bob"); //distortion model
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    //distortion_coefficients ------------
    strcpy(buffer, "distortion_coefficients:");  
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "  rows:  1");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    strcpy(buffer, "  cols:  5");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    std::fwrite("\n", sizeof(char), 1, fp); // EOL
    
    strcpy(buffer, "  data:  [");
    std::fwrite(buffer, sizeof(char), strlen(buffer), fp );
    for(int row=0; row < distCoeffs.rows ; ++row){
        double *row_ptr = distCoeffs.ptr<double>(row);
        for(int col=0; col < distCoeffs.cols ; ++col){
            char tmp[256];
            sprintf(tmp, "%.4f", row_ptr[col]);
            std::fwrite(tmp, sizeof(char), strlen(tmp), fp );
            if(row==0 && col== 4) std::fwrite("]", sizeof(char), 1, fp); // EOL
            else std::fwrite(", ", sizeof(char), 1, fp); // comma separated
        }
    }
    std::fwrite("\n", sizeof(char), 1, fp); // EOL

    fclose(fp);
    return 0;
}


/**
 * Function to perform calibration of the camera
 *
 * This function performs the calibration of the camera and then calls write_camera_properties_to_file function to save the calibration results to yaml file
 * 
 * @param calibImages vector of images for calibration
 * @param cameraName  camera model
 * @param boardSize mxn size of the checkered board
 * @param sqEdgeLength length of the edge of the checkered board squares
 * @param fileName yaml file path to store calibration results
 * @param printCalibrationData prints the calibration results if set
 * 
 * @return 0
 */
int save_calibration(std::vector<cv::Mat> &calibImages, 
                        std::string cameraName, cv::Size boardSize, 
                        float sqEdgeLength, std::string fileName, 
                        bool printCalibrationData){

    vector<cv::Point3f> point_set;
    vector<std::vector<cv::Point3f> > point_list;
    vector<std::vector<cv::Point2f> > corner_list;
    generateKnownBoardPos(sqEdgeLength, point_set, boardSize);
    cv::Mat frame = calibImages[0];

    
    for (auto image : calibImages) {
        
        vector<cv::Point2f> corners;
        if(!find_corners(image, image, corners, boardSize, false)) {
            printf("ERROR: Could not find any corners in image\n");
            continue;
        }
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
        corner_list.push_back(corners);
        point_list.push_back(point_set);
    }

    
    cv::Mat cameraMatrix,distCoeffs,R,T;
    cameraMatrix = (cv::Mat_<float>(3,3) << 1.0f, 0.f , (float)frame.cols/2.0f,
                                               0.f, 1.0f , (float)frame.rows/2.0f,
                                               0.f, 0.f, 1.0f);
    
    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    double reprojectionError = cv::calibrateCamera(point_list, corner_list, cv::Size(frame.cols,frame.rows), cameraMatrix, distCoeffs, R, T, 
                                                                                                                    CV_CALIB_FIX_ASPECT_RATIO);
    
    if(printCalibrationData){
        std::cout << "Reprojection Error: " << reprojectionError << std::endl;
        std::cout << "Camera Matrix : " << cameraMatrix << std::endl;
        std::cout << "Dist Coeffs : " << distCoeffs << std::endl;
        std::cout << "Rotation vector : " << R << std::endl;
        std::cout << "Translation vector : " << T << std::endl;
    }

    write_camera_properties_to_file(fileName, cameraMatrix, distCoeffs, cameraName, cv::Size(frame.cols,frame.rows), reprojectionError,1);
    return 0;
}


