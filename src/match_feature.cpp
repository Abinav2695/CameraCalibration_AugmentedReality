/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition Match Feature Functions
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

using namespace std;
using namespace cv;


void calculate_standard_deviation(std::vector<char *> labels, vector<vector<float> > feature_set, map<string, vector<float>> &label_standard_deviation_map){

  map<string, vector<vector<float>>> label_featureset_map;

  for(int i=0; i< labels.size(); i++) {
    //if key present in map, add to the current feature set
    if (label_featureset_map.find(labels[i]) != label_featureset_map.end()) {
      //add to the current feature set
      vector<vector<float>>& found_value = label_featureset_map[labels[i]];
      found_value.push_back(feature_set[i]);
    } else {
      vector<vector<float>> new_feature_set;
      new_feature_set.push_back(feature_set[i]);
      label_featureset_map[labels[i]] = new_feature_set;
    }
  }


  //calculate standard deviation for each label
  //map<string, vector<float>> label_standard_deviation_map;

   for (auto pair : label_featureset_map) {
        vector<float> stddev_values;
        string key = pair.first;
        std::vector<std::vector<float>> value = pair.second;
        //create 6 column vectors for each
        vector<float> col1, col2,col3,col4,col5,col6;
        for (std::vector<float> inner_vec : value) {
           col1.push_back(inner_vec[0]);
           col2.push_back(inner_vec[1]);
           col3.push_back(inner_vec[2]);
           col4.push_back(inner_vec[3]);
           col5.push_back(inner_vec[4]);
           col6.push_back(inner_vec[5]);
        }

        Mat column1(col1), column2(col2), column3(col3), column4(col4), column5(col5), column6(col6);
        Scalar mean1, mean2, mean3, mean4,mean5,mean6;
        Scalar stddev1, stddev2, stddev3, stddev4, stddev5, stddev6;

        meanStdDev(column1, mean1, stddev1);
        meanStdDev(column2, mean2, stddev2);
        meanStdDev(column3, mean3, stddev3);
        meanStdDev(column4, mean4, stddev4);
        meanStdDev(column5, mean5, stddev5);
        meanStdDev(column6, mean6, stddev6);
        stddev_values.push_back(stddev1[0]);
        stddev_values.push_back(stddev2[0]);
        stddev_values.push_back(stddev3[0]);
        stddev_values.push_back(stddev4[0]);
        stddev_values.push_back(stddev5[0]);
        stddev_values.push_back(stddev6[0]);

        label_standard_deviation_map[key] = stddev_values;
   }

}



string feature_match(char *filename, vector<float> &target_feature, MATCH_CLASSIFIER mode, int K /* =3 */) {
    //read data
    vector<char *> objects;
    vector<vector<float> > feature_set;
    //read feature set from csv data file
    read_image_data_csv(filename,objects,feature_set,0);

    //calculate std devn for each class
    map<string, vector<float>> label_standard_deviation_map;
    calculate_standard_deviation(objects, feature_set,label_standard_deviation_map);

    map<float, string> target_distance;

    if (mode == NearestNeighbour){
        for(int i=0; i< objects.size(); i++) {

          // retrieve the deviation for that class
          vector<float> feature = feature_set[i];
          string label(objects[i]);
          vector<float> std_dev = label_standard_deviation_map[label];

          //calculate distance
          float distance = 0;
          for(int j=0; j< feature.size(); j++) {
            float difference = target_feature[j] - feature[j];
            distance += (difference * difference)/ (std_dev[j] * std_dev[j]);
          }

          target_distance[distance] = label;
          // std::cout << label << " : " << distance << std::endl;
        }
    }

    else if (mode == KNearestNeighbour){

        vector<string> class_labels;
        vector<map<float, string>> distance_map;
        

        for(auto it = label_standard_deviation_map.begin(); it != label_standard_deviation_map.end(); ++it){
            // std::cout << it->first << std::endl;
            class_labels.push_back(it->first);
            map<float, string> temp;
            temp[99999] = it->first;
            distance_map.push_back(temp);
        }

        for(int i=0; i< objects.size(); i++) {

          // retrieve the deviation for that class
  
          string label(objects[i]);
          vector<float> std_dev = label_standard_deviation_map[label];
          std::size_t index;
          auto it = std::find(class_labels.begin(), class_labels.end(), label);

          // Check if the string was found
          if (it != class_labels.end()) {
              // Get the index of the matching element
              index = std::distance(class_labels.begin(), it);
          }
          //calculate distance
          float distance = 0;
          for(int j=0; j< feature_set[i].size(); j++) {
            float difference = target_feature[j] - feature_set[i][j];
            distance += (difference * difference)/ (std_dev[j] * std_dev[j]);
          }
          //put distance value in appropriate class in distance map based on label string
          distance_map[index][distance] = label;
        }

        for(int i=0; i< distance_map.size(); ++i){
            int count = 0;
            float temp_dist = 0;
            for(auto it = distance_map[i].begin(); it != distance_map[i].end() && count < K; ++it, ++count){

                temp_dist += it->first;
            }
            target_distance[temp_dist] = class_labels[i];
            if(temp_dist >= 99999)temp_dist = 99999;
            std::cout << temp_dist << " " ;
        }
    }

 
  std::cout << std::endl;
  auto first_pair = *target_distance.begin();
  if(first_pair.first >= 20){
    return "Unknown";
  } else{
    return first_pair.second;
  }
  
}






