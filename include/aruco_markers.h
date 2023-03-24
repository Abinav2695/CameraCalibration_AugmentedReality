/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   ARUCO Header File
*/

#ifndef ARUCO_MARKERS_H
#define ARUCO_MARKERS_H

typedef enum{

    FAMILY_25h9 =0,
    FAMILY_36h10,
    FAMILY_36h11
} ArucoMarkersFAMILY;

typedef struct{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    ArucoMarkersFAMILY family;
    std::vector<ArucoMarkersFAMILY> multiFamilies;
    bool draw_markers;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
} ArucoDetectionConfig; 

void detect_aruco_markers(cv::Mat &image, ArucoDetectionConfig *config);
void detect_camera_pose_aruco(cv::Mat& image, ArucoDetectionConfig *config);
void draw_shape_on_multiple_targets(cv::Mat& image, ArucoDetectionConfig *config);
void draw_cube(cv::Mat &image, cv::Scalar color, std::vector<cv::Point2f> imagePoints);

#endif
