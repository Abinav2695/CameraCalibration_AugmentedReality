/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Main File
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


// Video source variables
int vid_src = 0;
cv::VideoCapture *cap;
bool play_video = true;
cv::Mat frame; //input frame buffer
cv::Mat filtered_frame; //filtered frame
int play_mode = 0; //play original frame


std::string IMAGE_FOLDER_PATH = "/CameraCalibration_AugmentedReality/calib_images/";
int image_number[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};

std::string IMAGE_FOLDER_PATH_2 = "/CameraCalibration_AugmentedReality/images/";
int image_number_2[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};

std::string file_path = __FILE__;
std::string PROJECT_FOLDER_PATH = file_path.substr(0, file_path.rfind("/CameraCalibration_AugmentedReality"));


std::vector<cv::Point3f> point_set;
std::vector<std::vector<cv::Point3f> > point_list;
std::vector<std::vector<cv::Point2f> > corner_list;
cv::Size boardSize(9,6);

int main(int argc, char *argv[])
{
    if(argc<2) //check if any arguments are passed 
    {
        std::cout << "Please input the Video Device ID!!!" << std::endl;
        std::cout << "Usage : " << argv[0] << " {Video Device ID}" <<std::endl;
        std::cout << "Example : " << argv[0] << " 0/1/2/3/..." <<std::endl;
        return 0;
    }
    vid_src  = atoi(argv[1]);
    cap = new cv::VideoCapture(vid_src);

    //Check if the camera is accessible else exit program
    if (!cap->isOpened())
    {
        std::cerr << "ERROR: Cannot open video capture device" << std::endl;
        return 0;
    }

    // Get the size of the input image from the camera
    cv::Size refS( (int) cap->get(cv::CAP_PROP_FRAME_WIDTH ), (int) cap->get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << "INFO: Camera Calibration and Augmented Reality" << std::endl;
   
    // for(auto i:image_number)
    for(auto i:image_number_2)
    {
        // cap->read(frame); //get new frame

        // frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH + std::to_string(i)+ ".jpeg");
        frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH_2 + std::to_string(i)+ ".jpg");
        if(frame.empty()){
            printf("ERROR: Failed to read image");
            continue;
        }
        std::vector<cv::Point2f> corners;
        find_corners(frame,filtered_frame,corners, boardSize);
        corner_list.push_back(corners);

        // for (auto each_corn :corners){
        //     //printf("%f,%f \n", each_corn.x, each_corn.y);
        // }
        //printf("-------------------------------------------\n");
        char key_pressed = cv::waitKey(10); 
        switch(key_pressed)
        {
            case 'q': play_video = false; break;  //quit command
            default: break;
        }

        cv::imshow("Frame", filtered_frame);
        
    }
    cv::destroyAllWindows();
    cap->release();


    generateKnownBoardPos(0.03, point_set, boardSize);
    for (int i = 0; i < corner_list.size(); i++){
        point_list.push_back(point_set);
    }

    cv::Mat cameraMatrix,distCoeffs,R,T;
 
    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    double error = cv::calibrateCamera(point_list, corner_list, cv::Size(frame.rows,frame.cols), cameraMatrix, distCoeffs, R, T);
    
    std::cout << "Error: " << error << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    // std::cout << "Rotation vector : " << R << std::endl;
    // std::cout << "Translation vector : " << T << std::endl;
    return 0;
}

