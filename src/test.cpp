/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition Test Code File
   Source file
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "rtor.h"


// Video source variables
int vid_src = 0;
cv::VideoCapture *cap;
bool play_video = true;
cv::Mat frame; //input frame buffer
cv::Mat filtered_frame; //filtered frame
int play_mode = 0; //play original frame

//Variables to store the directory information and file paths
char filename[255];

std::string PROJECT_FOLDER = "/Assgn_3";
std::string IMAGE_FOLDER = "/Assgn_3/images/";

std::string file_path = __FILE__;
std::string PROJECT_FOLDER_PATH = file_path.substr(0, file_path.rfind(PROJECT_FOLDER));
std::string CSV_FILE_PATH = PROJECT_FOLDER_PATH + "/Assgn_3/csv_files/dataset.csv";
std::string TRAINING_DATA_FOLDER_PATH[14]  = {"/Assgn_3/training_dataset/Knife/", "/Assgn_3/training_dataset/Spatula1/", "/Assgn_3/training_dataset/Spatula2/",
                                            "/Assgn_3/training_dataset/Pan/", "/Assgn_3/training_dataset/Cup/", "/Assgn_3/training_dataset/Car/",
                                            "/Assgn_3/training_dataset/Allen_Key/", "/Assgn_3/training_dataset/Watch/", "/Assgn_3/training_dataset/Hanger/",
                                            "/Assgn_3/training_dataset/Perfume_Bottle/", "/Assgn_3/training_dataset/Eyeglasses_Case/",
                                            "/Assgn_3/training_dataset/BodyLotion_Dispenser/", "/Assgn_3/training_dataset/Soap_Box/",
                                            "/Assgn_3/training_dataset/Banana/"};


int main(int argc, char *argv[]){

    if(argc<2) //check if any arguments are passed 
    {
        std::cout << "Please input the Video Device ID!!!" << std::endl;
        std::cout << "Usage : " << argv[0] << " {Video Device ID}" <<std::endl;
        std::cout << "Example : " << argv[0] << " 0/1/2/3/..." <<std::endl;
        return 0;
    }
    vid_src  = atoi(argv[1]);
    cap = new cv::VideoCapture(vid_src);
    
    int input_series = 0;
    // std::cout << "Enter Image Series: " << std::endl;
    // std::cin >> input_series;

    //Check if the camera is accessible else exit program
    if (!cap->isOpened())
    {
        std::cerr << "ERROR: Cannot open video capture device" << std::endl;
        return 0;
    }

    // Get the size of the input image from the camera
    cv::Size refS( (int) cap->get(cv::CAP_PROP_FRAME_WIDTH ),
                       (int) cap->get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "INFO: Real Time Object 2-D Recongnition" << std::endl;
    
    // Video Loop
    while(play_video)
    {
        // cap->read(frame); //get new frame
        frame  = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/images/img3P3.png");
        // frame  = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/training_dataset/Hanger/image_3.jpg");
        // frame  = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/training_dataset/Allen_Key/image_2.jpg");
        // frame  = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/training_dataset/Spatula1/image_16986711.jpg");
        // frame  = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/training_dataset/Spatula1/image_16986716.jpg");

        // frame = cv::imread("/home/exmachina/NEU/SEM-2/PRCV/Assignment/ProjectX_CaptureImages/Imgs1/1678762548295.dng");
        

        if (frame.empty())
        {
            std::cout << "WARNING: Frame is empty" << std::endl;
            continue;
        }
        //wait for 10ms until a key is pressed
        //if a key event is registered check the event else continue
        char key_pressed = cv::waitKey(10); 
        switch(key_pressed)
        {
            case 'q': play_video = false; break;  //quit command
            case 't': cv::destroyAllWindows(); play_mode = 1; break;
            case 'o': cv::destroyAllWindows(); play_mode = 0; break;
            case 'd': cv::destroyAllWindows(); play_mode = 2; break;
            case 'f': play_mode =3; break;
            case 's': cv::destroyAllWindows(); play_mode =4; break;
            case 'm': cv::destroyAllWindows(); play_mode =5; break;
            case 'n': cv::destroyAllWindows(); play_mode =6; break;
            case 'v': {
                        std::stringstream ss;
                        ss << input_series;
                        std::string filename = "/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_3/training_dataset/Banana/image_" +  
                                                ss.str() + ".jpg" ;
                        std::cout << filename << std::endl;
                        cv::imwrite(filename,frame);  
                        input_series+=1;
                        break;}
            
            case 'a': {
                        std::stringstream ss;
                        ss << input_series;
                        std::string filename = PROJECT_FOLDER_PATH + IMAGE_FOLDER+ "thresh_image_" +  ss.str() + ".jpg" ;
                        std::cout << filename << std::endl;
                        cv::imwrite(filename,filtered_frame);  
                        input_series+=1;
                        break;}



            default: break;
        }
        if(play_mode == 1){

            darkenImage(frame, filtered_frame);
            threshold(filtered_frame, filtered_frame , kThreshOtsu);
            // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            // //Perform opening operation
            // morphologyEx(filtered_frame, filtered_frame, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2);
            // morphologyEx(filtered_frame, filtered_frame, cv::MORPH_OPEN, element, cv::Point(-1, -1), 2);

            GTMorph(filtered_frame, 3, kClosingMode);
            GTMorph(filtered_frame, 3, kOpeningMode);
            cv::imshow("Thresh", filtered_frame);

        } else if(play_mode == 2){
            darkenImage(frame, filtered_frame);
            cv::cvtColor(filtered_frame, filtered_frame, cv::COLOR_BGR2GRAY);
            cv::imshow("Darkened", filtered_frame);

        } else if(play_mode == 3){
            std::vector< std::vector<float>> components;
            std::vector< std::vector<float>> feature_vector;
            findConnectedComponents(filtered_frame, components);
            compute_features(filtered_frame, components, feature_vector);
            
            std::cout << std::endl;
            play_mode = 1;

        } else if(play_mode == 4){
            std::string label;
            std::cout << "Enter object Label: " << std::endl;
            std::cin >> label;
            save_new_feature(frame, (char *)CSV_FILE_PATH.c_str(), (char *)label.c_str());
            play_mode = 0;

        } else if (play_mode == 5){
            std::vector< std::vector<float>> components;
            std::vector<std::vector<float>> feature_set;

            darkenImage(frame, filtered_frame);
            threshold(filtered_frame, filtered_frame , kThreshOtsu);
            GTMorph(filtered_frame, 2, kClosingMode);
            GTMorph(filtered_frame, 2, kOpeningMode);
            findConnectedComponents(filtered_frame, components);
            

            if(components.size()>0) {

                printf("Total Components: %ld\n", components.size());
                compute_features(filtered_frame,components,feature_set);
                for(int i=0; i<feature_set.size(); i++){
                                   
                    // int cx = components[0][5];
                    // int cy = components[0][6];
                    // int top = components[0][1];
                    // int left = components[0][0];
                    
                    // std::vector<float> feature_set;
                    // extract_feature(frame, feature_set);
                    // float alpha  = feature_set[6];
                    // cv::line(frame,cv::Point2d(cx, cy), cv::Point2d(cx +(int)(400*cos(1.571 - alpha)), cy +(int)(400*sin(1.571 - alpha))),cv::Scalar(0,0,255), 2);
                    // cv::line(frame,cv::Point2d(cx, cy), cv::Point2d(cx -(int)(400*cos(1.571 - alpha)), cy -(int)(400*sin(1.571 - alpha))),cv::Scalar(0,0,255), 2);
                    
                    std::string Final_match = feature_match((char *)CSV_FILE_PATH.c_str(), feature_set[i], KNearestNeighbour, 3);
                    std::cout << "Got a match :: " << Final_match << std::endl;
                    cv::rectangle(frame, cv::Rect(components[i][0], components[i][1], components[i][2], components[i][3]), cv::Scalar(50, 255, 50), 2);
                    cv::putText(frame, Final_match, cv::Point(components[i][0], components[i][1] - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, (50, 255, 50), 2);
                }

            }
            
            // play_mode = 0;

        } else if (play_mode == 6) {

            int csv_file_count = 0;
            std::string label;
            std::cout << "Enter csv file number and Label: " << std::endl;
            std::cin >> csv_file_count >> label;
            std::string folder = PROJECT_FOLDER_PATH + TRAINING_DATA_FOLDER_PATH[csv_file_count];
            std::cout << folder << std::endl;
            // std::cout << label << std::endl;
            train_folder((char *)folder.c_str(), (char *)CSV_FILE_PATH.c_str(), (char *)label.c_str());
            play_mode = 0;
        }
        cv::imshow("Video", frame);
    }

    std::cout << "Exiting..." << std::endl;
    cv::destroyAllWindows();

    std::vector<int> nums;
    return 0;

}

// int main(int argc, char *argv[]){

//     cv::Mat image = (cv::Mat_<uchar>(5,5) << 1, 2, 3, 4, 5,
//                                             6, 7, 8, 9, 10,
//                                             11, 12, 13, 14, 15,
//                                             6, 7, 8, 9, 10,
//                                             1, 2, 3, 4, 5);
//     cv::copyMakeBorder(image, image, 1,1,1,1,cv::BORDER_REFLECT);
//     for (int i = 0; i < image.rows; i++){
//         uchar *ptr = image.ptr<uchar>(i);
//         for (int j = 0; j < image.cols; j++){
//             // std::cout << (int)ptr[j] << "  ";
//         }
//         // std::cout << std::endl;
//     }
//     // std::cout << image.size() << std::endl;

//     // std::cout << atan2(1,1) << std::endl;

//     int a,b;
//     std::cin >> a >> b;
//     std::cout << a << "       " << b << std::endl;
// }



