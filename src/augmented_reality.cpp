/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   AR Functions
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
#include <utility>  
#include "augmented_reality.h"

void readCameraParameters(std::string filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs){
    // Load camera param YAML file
    YAML::Node yaml_config = YAML::LoadFile(filename);

    // Access all camera parameters
    std::vector<double> intrinsic_values = yaml_config["camera_matrix"]["data"].as<std::vector<double>>();
    int rows = yaml_config["camera_matrix"]["rows"].as<int>();
    int cols = yaml_config["camera_matrix"]["cols"].as<int>();

    cameraMatrix = cv::Mat::zeros(cv::Size(rows, cols), CV_64FC1);
    // cv::Mat cameraMatrix(rows, cols, CV_64FC1);
    
    for (int row=0; row<rows; ++row){
        double *row_ptr = cameraMatrix.ptr<double>(row);
        for (int col=0; col<cols; ++col){
            row_ptr[col] = intrinsic_values[row*cols + col];
        }
    }

    std::vector<double> dist_values = yaml_config["distortion_coefficients"]["data"].as<std::vector<double>>();
    rows = yaml_config["distortion_coefficients"]["rows"].as<int>();
    cols = yaml_config["distortion_coefficients"]["cols"].as<int>();

    distCoeffs = cv::Mat::zeros(cv::Size(rows, cols), CV_64FC1);
    // cv::Mat distCoeffs(rows, cols, CV_64FC1);

    for (int row=0; row<rows; ++row){
        double *row_ptr = distCoeffs.ptr<double>(row);
        for (int col=0; col<cols; ++col){
            row_ptr[col] = dist_values[row*cols + col];
        }
    }
}

void project_world_points_to_image_plane(cv::Mat &image, ProjectPoints2DConfig *config){
    
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat R,T;
    readCameraParameters(config->cameraIntrinsicsFileName,cameraMatrix,distCoeffs);
    cv::solvePnP(config->worldCornerPoints, config->imageCornersPoints, cameraMatrix, distCoeffs, R, T);
    std::cout << "Rotation Matrix : " << R << std::endl;
    std::cout << "Translation Matrix : " << T << std::endl;
    
    cv::projectPoints(config->pointsToProjectOnImagePlane, R, T, cameraMatrix, distCoeffs, config->outputImagePoints);

    // Draw the 3D axes on the image
    cv::Mat rot_mat;
    cv::Rodrigues(R, rot_mat);
    cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rot_mat, T, 0.07);

}




void draw_pyramid(cv::Mat &image, ProjectPoints2DConfig *config){

    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[13]);
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[22]);
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[30]);
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[32]);
    for (int i=0; i< config->pointsToProjectOnImagePlane.size();++i){
        config->pointsToProjectOnImagePlane[i].z = 0.050;
    }
    cv::Point3f top_point = config->worldCornerPoints[22];
    top_point.z = 0.100;
    config ->pointsToProjectOnImagePlane.push_back(top_point);

    project_world_points_to_image_plane(image, config);
    for (auto point :config->outputImagePoints) cv::circle(image, point, 5, cv::Scalar(0, 0, 255), cv::FILLED);
    std::vector<std::pair<int,int>> linePoints = {{0,2},{0,3},{2,3},{0,4},{2,4},{3,4},{1,4}};
    for (auto indeices : linePoints){
        cv::line(image, config->outputImagePoints[indeices.first], config->outputImagePoints[indeices.second],cv::Scalar(255, 0, 0), 2,4 );
    }

}

void draw_spatula(cv::Mat &image, ProjectPoints2DConfig *config){

    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[18]); //0
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[27]); //1
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[23]); //2
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[32]); //3
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[14]); //4
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[41]); //5
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[17]); //6
    config ->pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[44]); //7

    // std::vector<cv::Point3f> top_points;
    int total_points = config->pointsToProjectOnImagePlane.size();
    for(int i = 0; i< total_points; i++){

        cv::Point3f point = config->pointsToProjectOnImagePlane[i];
        point.z = 0.125;
        config->pointsToProjectOnImagePlane.push_back(point);
        config->pointsToProjectOnImagePlane[i].z = 0.100;
    }

    project_world_points_to_image_plane(image, config);
    
    
    for (auto point :config->outputImagePoints) cv::circle(image, point, 5, cv::Scalar(0, 0, 255), cv::FILLED);

    for (int i = 0; i< total_points; i++){
        cv::line(image, config->outputImagePoints[i], config->outputImagePoints[total_points+i],cv::Scalar(255, 0, 0), 2,4 );
    }

    std::vector<std::pair<int,int>> linePoints = {{0,1},{0,2},{1,3},{2,4},{3,5},{4,6},{5,7},{6,7}};
    int total_lines = linePoints.size();
    for (int i = 0; i < total_lines; ++i){
        linePoints.push_back(std::make_pair(linePoints[i].first + total_points, linePoints[i].second+total_points));
    }
    for (auto indeices : linePoints){
        cv::line(image, config->outputImagePoints[indeices.first], config->outputImagePoints[indeices.second],cv::Scalar(255, 0, 0), 2,4 );
    }

}


