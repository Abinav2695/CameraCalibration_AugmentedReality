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

//Local Header Files
#include "calib.h"
#include "augmented_reality.h"
#include "aruco_markers.h"

// Video source variables
int vid_src = 0;
cv::VideoCapture *cap;
bool play_video = true;
cv::Mat frame; //input frame buffer
cv::Mat drawFrame; //filtered frame
int play_mode = 0; //play original frame
std::string camera_model; //camera name


std::string IMAGE_FOLDER_PATH = "/CameraCalibration_AugmentedReality/calib_images/";
int image_number[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};

std::string IMAGE_FOLDER_PATH_2 = "/CameraCalibration_AugmentedReality/images/";
int image_number_2[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};

std::string CALIB_FOLDER_PATH = "/CameraCalibration_AugmentedReality/calib_params/";
std::string file_path = __FILE__;
std::string PROJECT_FOLDER_PATH = file_path.substr(0, file_path.rfind("/CameraCalibration_AugmentedReality"));

// Checker Board Settings
std::vector<cv::Mat> calib_images;
int imageCount = 0;
cv::Size boardSize(9,6);
float squareSize = 0.034;
std::string yamlFileName;

int main(int argc, char *argv[])
{
    if(argc<3) //check if any arguments are passed 
    {
        std::cout << "Please input the Video Device ID!!!" << std::endl;
        std::cout << "Usage : " << argv[0] << " {Video Device ID}  {Camera Name}" <<std::endl;
        std::cout << "Example : " << argv[0] << " 0/1/2/3/ MyCam" <<std::endl;
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
    // Print camera model
    // camera_model = cap->getBackendName();
    camera_model = argv[2];
    std::cout << "INFO: Camera model ->" << camera_model << std::endl;
    std::cout << "INFO: Camera Calibration and Augmented Reality" << std::endl;
    std::cout << "INFO: Camera Resolution ->" << refS << std::endl;
    


    while(play_video){

        cap->read(frame); //get new frame
        if(frame.empty()){ //If no frame then continue
            printf("WARNING: Failed to read image\n");
            continue;
        }

        char key_pressed = cv::waitKey(10); 
        switch(key_pressed)
        {
            case 's': //Save image for calibration
            {
                //Save Image only if all corners are found else throw error message
                std::vector<cv::Point2f> corners;
                if(find_corners(frame, drawFrame, corners, boardSize, true)) {
                    calib_images.push_back(frame.clone());
                    imageCount++;
                    printf("Total image count: %ld\n", calib_images.size());
                    cv::imwrite("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_4/CameraCalibration_AugmentedReality/image_corners/Corners_Image_"+ std::to_string(imageCount) + ".jpg", drawFrame);
                }
                else printf("WARNING: Cannot find corners in image ... Dropping current image\n");
                break;
            }
            
            case 'c': //Call calibration function
            {
                yamlFileName = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
                if(calib_images.size() > 5) {
                    save_calibration(calib_images, camera_model, boardSize, squareSize, yamlFileName, true);
                }
                else printf("WARNING: Image count is too low for calibration.. add more images\n");
                break;
            }

            case 'd': //Take images from database and calibrate
            {
                std::vector<cv::Mat> savedImages;
                for(auto i:image_number_2)
                {
                    frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH_2 + std::to_string(i)+ ".jpg");
                    if(frame.empty()){
                        printf("ERROR: Failed to read image\n");
                        continue;
                    }
                    savedImages.push_back(frame);
                }
                yamlFileName = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
                if(savedImages.size() > 5) save_calibration(savedImages, camera_model, boardSize, squareSize, yamlFileName, true);
                else printf("WARNING: Image count is too low for calibration.. add more images\n");
                break;
            }

            case 'o': play_mode = 0; break; //original frame
            case 'p': play_mode = 1; break; //projecting points
            case 'y': play_mode = 2; break; //pyrammid
            case 'l': play_mode = 3; break; //spatula
            case 'a': play_mode = 4; break; //aruco detection
            case 'b': play_mode = 5; break; //aruco pose estimation
            case 'f': play_mode = 6; break; //draw images of multiple targets
            case 'q': play_video = false; break;  //quit command

            default: break;
        }

        if(play_mode==1){
            ProjectPoints2DConfig config;
            if(find_corners(frame, drawFrame, config.imageCornersPoints, boardSize, false)) {
                generateKnownBoardPos(squareSize, config.worldCornerPoints, boardSize);
                config.pointsToProjectOnImagePlane.push_back(config.worldCornerPoints[0]);
                config.pointsToProjectOnImagePlane.push_back(config.worldCornerPoints[8]);
                config.pointsToProjectOnImagePlane.push_back(config.worldCornerPoints[45]);
                config.pointsToProjectOnImagePlane.push_back(config.worldCornerPoints[53]);
                for (auto point:config.pointsToProjectOnImagePlane) {
                    point.z = 0.020f;
                    config.pointsToProjectOnImagePlane.push_back(point);
                }
                config.cameraIntrinsicsFileName = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
                project_world_points_to_image_plane(frame, &config);
                
                for( auto point: config.outputImagePoints) {
                    cv::circle(frame, point, 2, cv::Scalar(0,0,255), 2);
                    
                }
            } else {
                printf("ERROR: No corners found in image!!!\n");
            } 
        } else if(play_mode ==2){   //Draw Pyramid
            ProjectPoints2DConfig config;
            if(find_corners(frame, drawFrame, config.imageCornersPoints, boardSize, false)) {
                generateKnownBoardPos(squareSize, config.worldCornerPoints, boardSize);
                config.cameraIntrinsicsFileName = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
                draw_pyramid(frame, &config);
            } else printf("ERROR: No corners found in image!!!\n");


        } else if(play_mode ==3){   //Draw Spatula
            ProjectPoints2DConfig config;
            if(find_corners(frame, drawFrame, config.imageCornersPoints, boardSize, false)) {
                generateKnownBoardPos(squareSize, config.worldCornerPoints, boardSize);
                config.cameraIntrinsicsFileName = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
                draw_spatula(frame, &config);
            } else printf("ERROR: No corners found in image!!!\n");

        } else if (play_mode == 4){ //Aruco detection
            ArucoDetectionConfig config;
            config.draw_markers = true;
            // config.family = FAMILY_25h9;
            // config.family = FAMILY_36h11;
            config.family = FAMILY_36h10;

            detect_aruco_markers(frame, &config);

        } else if (play_mode == 5){ //Camera Pose Detection using Aruco Board
            ArucoDetectionConfig config;
            config.draw_markers = true;
            config.family = FAMILY_25h9;
            std::string filename = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
            readCameraParameters(filename, config.cameraMatrix, config.distCoeffs);
            detect_camera_pose_aruco(frame, &config);
        
        } else if (play_mode == 6){ //draw shapes on multiple markers
            ArucoDetectionConfig config;
            config.draw_markers = true;
            config.multiFamilies.push_back(FAMILY_36h11);
            config.multiFamilies.push_back(FAMILY_25h9);
            std::string filename = PROJECT_FOLDER_PATH + CALIB_FOLDER_PATH + camera_model + "_Calibration_Parameters.yaml";
            readCameraParameters(filename, config.cameraMatrix, config.distCoeffs);
            draw_shape_on_multiple_targets(frame, &config);

        }
        cv::imshow("Frame", frame);
        
    }

    cv::destroyAllWindows();
    cap->release();  
    return 0;
}










//  // for(auto i:image_number)
//     for(auto i:image_number_2)
//     {
//         cap->read(frame); //get new frame

//         // frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH + std::to_string(i)+ ".jpeg");
//         // frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH_2 + std::to_string(i)+ ".jpg");
//         if(frame.empty()){
//             printf("ERROR: Failed to read image");
//             continue;
//         }
//         std::vector<cv::Point2f> corners;
//         find_corners(frame,filtered_frame,corners, boardSize);
//         corner_list.push_back(corners);

//         // for (auto each_corn :corners){
//         //     //printf("%f,%f \n", each_corn.x, each_corn.y);
//         // }
//         //printf("-------------------------------------------\n");
//         char key_pressed = cv::waitKey(10); 
//         switch(key_pressed)
//         {
//             case 'q': play_video = false; break;  //quit command
//             default: break;
//         }

//         cv::imshow("Frame", filtered_frame);
        
//     }
//     cv::destroyAllWindows();
//     cap->release();


//     generateKnownBoardPos(0.034, point_set, boardSize);
//     for (int i = 0; i < corner_list.size(); i++){
//         point_list.push_back(point_set);
//     }

//     cv::Mat cameraMatrix,distCoeffs,R,T;
//     // cameraMatrix = (cv::Mat_<float>(5,5) << 1, 0 , frame.rows/2,
//     //                                            0, 1 , frame.cols/2,
//     //                                            0, 0, 1);
//     /*
//     * Performing camera calibration by 
//     * passing the value of known 3D points (objpoints)
//     * and corresponding pixel coordinates of the 
//     * detected corners (imgpoints)
//     */
//     double error = cv::calibrateCamera(point_list, corner_list, cv::Size(frame.rows,frame.cols), cameraMatrix, distCoeffs, R, T);
    
//     std::cout << "Error: " << error << std::endl;
//     std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
//     std::cout << "distCoeffs : " << distCoeffs << std::endl;
//     std::cout << "Rotation vector : " << R << std::endl;
//     // std::cout << "Translation vector : " << T << std::endl;

//     cv::Mat R_new,T_new;
//     bool ret = cv::solvePnP(point_list[0], corner_list[0], cameraMatrix, distCoeffs, R_new, T_new);
//     if(ret){
//         std::cout << "Rotation vector : " << R_new << std::endl;
//         std::cout << "Translation vector : " << T_new << std::endl;

//     }

