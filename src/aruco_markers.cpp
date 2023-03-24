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
    // cv::Mat img1;
    // board->generateImage(cv::Size(640,480), img1, 5);
    // cv::imshow("Board", img1);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

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

        cv::Rodrigues(rvec, rotMat);
        
        cv::Rodrigues(rotMat, eulerAngles, cv::noArray());
        eulerAngles*= 180/CV_PI;

        std::cout << "Rotation Angles: " << eulerAngles << std::endl;
        std::cout << "Tvec: " << tvec << std::endl;

        // If at least one board marker detected
        int markersOfBoardDetected = (int)objPoints.total() / 4;
        if(markersOfBoardDetected > 0 && config->draw_markers)
            cv::drawFrameAxes(image, config->cameraMatrix, config->distCoeffs, rvec, tvec, 0.1);
        

        std::vector<cv::Point3f> pointsToProjectOnImagePlane;
        std::vector<cv::Point2f> outputImagePoints;
        pointsToProjectOnImagePlane.push_back(cv::Point3f(0.050f,0.05f,-0.100f));
        // pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[22]);
        // pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[30]);
        // pointsToProjectOnImagePlane.push_back(config->worldCornerPoints[32]);
        // for (int i=0; i< pointsToProjectOnImagePlane.size();++i){
        //     config->pointsToProjectOnImagePlane[i].z = 0.050;
        // }
        // cv::Point3f top_point = config->worldCornerPoints[22];
        // top_point.z = 0.100;
        // config ->pointsToProjectOnImagePlane.push_back(top_point);
        cv::projectPoints(pointsToProjectOnImagePlane, rvec, tvec, config->cameraMatrix, config->distCoeffs, outputImagePoints);
        for (auto point : outputImagePoints) cv::circle(image, point, 4, cv::Scalar(0,0,255), -1);
        
    }

}   

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
        // cv::Ptr<cv::aruco::GridBoard> board = new cv::aruco::GridBoard(cv::Size(7,5), 0.04, 0.01, dictionary);
        // cv::Mat img1;
        // board->generateImage(cv::Size(640,480), img1, 5);
        // cv::imshow("Board", img1);
        // cv::waitKey(0);
        // cv::destroyAllWindows();

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

                cv::projectPoints(pointsToProjectOnImagePlane, rvec, tvec, config->cameraMatrix, config->distCoeffs, outputImagePoints);
                // for (auto point : outputImagePoints) cv::circle(image, point, 4, cv::Scalar(0,0,255), -1);
                cv::Scalar color;
                color = family == FAMILY_25h9 ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
                draw_cube(image, color,outputImagePoints);
                
            }
        }
    }
    
}

// int main()
// {
//     // Set the dictionary to use
//     cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9 );

//     // Define the grid board
//     cv::Size2f square_size(0.06f, 0.06f); // size of each square in meters
//     cv::Size2i board_size(4, 2); // number of squares in each row and column
//     float spacing = 0.03f; // spacing between squares in meters
//     cv::Ptr<cv::aruco::GridBoard> gridboard = cv::aruco::GridBoard::create(4, 2, 0.06f, 0.02f, dictionary);

//     cv::Mat boardImage;
//     gridboard->draw(cv::Size(1280, 720), boardImage, 10, 1);

//     // Print the board information
//     std::cout << "Grid board information:" << std::endl;
//     std::cout << " - number of markers: " << gridboard->getMarkerLength() << std::endl;

//     cv::imshow("Frame", boardImage);
//     cv::imwrite("36h11_markers.png", boardImage);
//     char key = cv::waitKey(0);
//     cv::destroyAllWindows();
//     return 0;
// }



// cv::VideoCapture inputVideo;
// inputVideo.open(0);
// cv::Mat cameraMatrix, distCoeffs;
// // You can read camera parameters from tutorial_camera_params.yml
// readCameraParameters(filename, cameraMatrix, distCoeffs);  // This function is implemented in aruco_samples_utility.hpp
// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
// // To use tutorial sample, you need read custom dictionaty from tutorial_dict.yml
// readDictionary(filename, dictionary); // This function is implemented in opencv/modules/objdetect/src/aruco/aruco_dictionary.cpp
// cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
// cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
// cv::aruco::ArucoDetector detector(dictionary, detectorParams);
// while (inputVideo.grab()) {
//     cv::Mat image, imageCopy;
//     inputVideo.retrieve(image);
//     image.copyTo(imageCopy);
//     std::vector<int> ids;
//     std::vector<std::vector<cv::Point2f> > corners;
//     // Detect markers
//     detector.detectMarkers(image, corners, ids);
//     // If at least one marker detected
//     if (ids.size() > 0) {
//         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
//         cv::Vec3d rvec, tvec;
//         // Get object and image points for the solvePnP function
//         cv::Mat objPoints, imgPoints;
//         board->matchImagePoints(corners, ids, objPoints, imgPoints);
//         // Find pose
//         cv::solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
//         // If at least one board marker detected
//         markersOfBoardDetected = (int)objPoints.total() / 4;
//         if(markersOfBoardDetected > 0)
//             cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
//     }
//     cv::imshow("out", imageCopy);
//     char key = (char) cv::waitKey(waitTime);
//     if (key == 27)
//         break;
// }