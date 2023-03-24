/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   ARUCO Source File
   Functions for Aruco marker detection and aruco board detection
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>


#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include "aruco_markers.h"

/**
 * Function to detect aruco markers and draw ids and bounding boxes on the image
 *
 * This function takes an image and the ArucoDetectionConfig type config variable as inputs
 * @param image cv::Mat type input image
 * @param config  ArucoDetectionConfig configuration variables for the detection
 *
 */
void detect_aruco_markers(cv::Mat &image, ArucoDetectionConfig *config){

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary;

    switch(config->family){
        case FAMILY_25h9: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9); break;
        case FAMILY_36h10: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10); break;
        case FAMILY_36h11: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
        default: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
    } 
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    std::vector<std::vector<cv::Point2f>> rejected;
    detector.detectMarkers(image, config->corners, config->ids, rejected);

    std::cout << "Total Markers Found:" << config->ids.size() << std::endl;
    if(config->ids.size() > 0 ){
        if(config->draw_markers)cv::aruco::drawDetectedMarkers(image, config->corners, config->ids);
        
    }

}

/**
 * Function to detect camera pose based of Aruco Board detection
 *
 * This function takes an image and the ArucoDetectionConfig type config variable as inputs
 *
 * @param image cv::Mat type input image
 * @param config  ArucoDetectionConfig configuration variables for the detection
 *
 */
void detect_camera_pose_aruco(cv::Mat& image, ArucoDetectionConfig *config){

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary;

    switch(config->family){
        case FAMILY_25h9: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9); break;
        case FAMILY_36h10: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10); break;
        case FAMILY_36h11: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
        default: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
    }
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    cv::Ptr<cv::aruco::GridBoard> board = new cv::aruco::GridBoard(cv::Size(7,5), 0.04, 0.01, dictionary);

    // Detect markers
    detector.detectMarkers(image, config->corners, config->ids);

    // If at least one marker detected
    if (config->ids.size() > 0) {
        if(config->draw_markers)cv::aruco::drawDetectedMarkers(image, config->corners, config->ids);
        cv::Vec3d rvec, tvec;

        // Get object and image points for the solvePnP function
        cv::Mat objPoints, imgPoints;
        board->matchImagePoints(config->corners, config->ids, objPoints, imgPoints);
        
        // Find pose
        cv::solvePnP(objPoints, imgPoints, config->cameraMatrix, config->distCoeffs, rvec, tvec);
        cv::Mat rotMat, eulerAngles;
        cv::Rodrigues(rvec, rotMat); //rotation vector to rotation matrix
        cv::Rodrigues(rotMat, eulerAngles, cv::noArray()); //rotation matrix to euler angles
        eulerAngles*= 180/CV_PI;

        std::cout << "Aruco Marker Rotation (Euler Angles): " << eulerAngles << std::endl;
        std::cout << "Aruco Marker Translation Vector: " << tvec << std::endl;

        // If at least one board marker detected
        int markersOfBoardDetected = (int)objPoints.total() / 4;
        if(markersOfBoardDetected > 0 && config->draw_markers)
            cv::drawFrameAxes(image, config->cameraMatrix, config->distCoeffs, rvec, tvec, 0.1);
        

        std::vector<cv::Point3f> pointsToProjectOnImagePlane;
        std::vector<cv::Point2f> outputImagePoints;
        pointsToProjectOnImagePlane.push_back(cv::Point3f(0.050f,0.05f,-0.100f));
        cv::projectPoints(pointsToProjectOnImagePlane, rvec, tvec, config->cameraMatrix, config->distCoeffs, outputImagePoints);
        for (auto point : outputImagePoints) cv::circle(image, point, 4, cv::Scalar(0,0,255), -1);
        
    }

}   

/**
 * Function to draw a 3D cube on aruco markers anchored at the center
 *
 * This function takes an image, colour of the cube and image points required to draw cube
 *
 * @param image cv::Mat type input image
 * @param color  cv::Scalar type colour value for drawing cube
 * @param imagePoints Image points required to draw the cube
 */
void draw_cube(cv::Mat &image, cv::Scalar color, std::vector<cv::Point2f> imagePoints){


    cv::line(image, imagePoints[2], imagePoints[6],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[3], imagePoints[7],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[4], imagePoints[8],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[5], imagePoints[9],color, 2, cv::LINE_AA);


    cv::line(image, imagePoints[2], imagePoints[4],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[2], imagePoints[5],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[3], imagePoints[4],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[3], imagePoints[5],color, 2, cv::LINE_AA);


    cv::line(image, imagePoints[6], imagePoints[8],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[6], imagePoints[9],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[7], imagePoints[8],color, 2, cv::LINE_AA);
    cv::line(image, imagePoints[7], imagePoints[9],color, 2, cv::LINE_AA);
}


/**
 * Function to draw a 3D cube on multiple aruco markers of multiple families
 *
 * This function takes an image and ArucoDetectionConfig type config variable as inputs
 *
 * @param image cv::Mat type input image
 * @param config  ArucoDetectionConfig config variables
 */
void draw_shape_on_multiple_targets(cv::Mat& image, ArucoDetectionConfig *config){

    for (auto family : config->multiFamilies){
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary;

        switch(family){
            case FAMILY_25h9: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9); break;
            case FAMILY_36h10: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10); break;
            case FAMILY_36h11: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
            default: dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11); break;
        }
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        // Detect markers
        detector.detectMarkers(image, config->corners, config->ids);

        // If at least one marker detected
        if (config->ids.size() > 0) {
            if(config->draw_markers)cv::aruco::drawDetectedMarkers(image, config->corners, config->ids);
            cv::Mat rotation_vectors, translation_vectors;
            cv::aruco::estimatePoseSingleMarkers(config->corners, 0.06f, config->cameraMatrix, config->distCoeffs, rotation_vectors, translation_vectors);
            
            // std::cout << "Total Rotation Vectors: " << rotation_vectors.size() << std::endl;
            // std::cout << "Translation Vector: " << translation_vectors.size() << std::endl;
            // std::cout << " Total Image Corners: " << config->corners.size() << std::endl;


            for (int i = 0; i < config->corners.size(); ++i){
                
                cv::Vec3d rvec,tvec;
                rvec = rotation_vectors.at<cv::Vec3d>(0,i);
                tvec = translation_vectors.at<cv::Vec3d>(0,i);
                // Converting rotation vector to matrix to euler angles in degrees
                cv::Mat rotMat, eulerAngles;
                cv::Rodrigues(rvec, rotMat);
                cv::Rodrigues(rotMat, eulerAngles, cv::noArray());
                eulerAngles*= 180/CV_PI;

                std::cout << "Rotation Vector: " << rvec << std::endl;
                std::cout << "Translation Vector: " << tvec << std::endl;
                // if(config->draw_markers) cv::drawFrameAxes(image, config->cameraMatrix, config->distCoeffs, rvec, tvec, 0.1);


                // World coordinates required to draw cube relative to aruco marker center    
                std::vector<cv::Point3f> pointsToProjectOnImagePlane;
                std::vector<cv::Point2f> outputImagePoints;

                pointsToProjectOnImagePlane.push_back(cv::Point3f(0,0,0.02f));  //Circle center
                pointsToProjectOnImagePlane.push_back(cv::Point3f(0,0,0.07f));  
                float sideLength = 0.02f;
                pointsToProjectOnImagePlane.push_back(cv::Point3f(sideLength,sideLength,0.02f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(-sideLength,-sideLength,0.02f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(sideLength,-sideLength,0.02f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(-sideLength,sideLength,0.02f));

                pointsToProjectOnImagePlane.push_back(cv::Point3f(sideLength,sideLength,0.05f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(-sideLength,-sideLength,0.05f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(sideLength,-sideLength,0.05f));
                pointsToProjectOnImagePlane.push_back(cv::Point3f(-sideLength,sideLength,0.05f));

                // Finding corresponding image coordinates of the world 3D points
                cv::projectPoints(pointsToProjectOnImagePlane, rvec, tvec, config->cameraMatrix, config->distCoeffs, outputImagePoints);
                // for (auto point : outputImagePoints) cv::circle(image, point, 4, cv::Scalar(0,0,255), -1);
                cv::Scalar color;
                color = family == FAMILY_25h9 ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
                draw_cube(image, color,outputImagePoints); //draw cube
            }
        }
    }
    
}
