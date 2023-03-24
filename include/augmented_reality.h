/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   AR Header File
*/

#ifndef AR_H
#define AR_H


typedef struct{
   std::string cameraIntrinsicsFileName;
   std::vector<cv::Point2f> imageCornersPoints;
   std::vector<cv::Point3f> worldCornerPoints;
   std::vector<cv::Point3f> pointsToProjectOnImagePlane;
   std::vector<cv::Point2f> outputImagePoints;
}ProjectPoints2DConfig;

void readCameraParameters(std::string filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
void project_world_points_to_image_plane(cv::Mat &image, ProjectPoints2DConfig *config);
void draw_pyramid(cv::Mat &image, ProjectPoints2DConfig *config);
void draw_spatula(cv::Mat &image, ProjectPoints2DConfig *config);

#endif