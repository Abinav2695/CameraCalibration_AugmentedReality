/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition Save Feature Functions
   Source file
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <dirent.h>

#include "csv_utils.h"
#include "rtor.h"


int save_new_feature(cv::Mat &frame, char *filename, char *label){
    
    std::vector<float> feature_set;
    extract_feature(frame, feature_set);
    append_image_data_csv(filename, label, feature_set,0);
    return 0;
}

int train_folder(char *foldername, char *csv_filename, char *label) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (foldername)) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            //std::string filename = ent->d_name;
            char image_filename[300];
            strcpy(image_filename, ent->d_name);
            if (strlen(image_filename) > 4) {
                char absolute_path[500];
                strcpy(absolute_path, foldername);
                strcat(absolute_path,image_filename);                
                //cout<<absolute_path<<endl;

                //extract features of each image
                std::vector<float> feature_set;
                cv::Mat image = cv::imread(absolute_path);
                extract_feature(image, feature_set);
                append_image_data_csv(csv_filename, label, feature_set,0);

            }

        }
        printf(" Finished processing all images in %s\n", foldername);
        closedir (dir);
    }


    return 0;
}

int extract_feature(cv::Mat &frame, std::vector<float> &feature_set){

    cv::Mat filtered_frame;
    darkenImage(frame, filtered_frame);
    threshold(filtered_frame, filtered_frame , kThreshOtsu);
    GTMorph(filtered_frame, 2, kClosingMode);
    GTMorph(filtered_frame, 2, kOpeningMode);

    std::vector< std::vector<float>> components;
    std::vector< std::vector<float>> feature_vector;
    findConnectedComponents(filtered_frame, components);
    compute_features(filtered_frame, components, feature_vector);
    
    for(int i = 0; i < feature_vector[0].size(); ++i){
        feature_set.push_back(feature_vector[0][i]);
    }

    return 0;
}

