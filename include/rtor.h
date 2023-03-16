/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition
   Include file
*/

#ifndef RTOR_H
#define RTOR_H

#define DEBUG 1

// Threshold modes ENUM
typedef enum{
   kThreshBinary,
   kThreshHSV,
   kThreshOtsu,
   kThreshOtsuHSV
} THRESH_MODE;

// MORPHOLOGICAL Operation ENUM
typedef enum{
   kOpeningMode,
   kClosingMode
} MORPH_OP;

// Classifier type ENUM
typedef enum{
   NearestNeighbour,
   KNearestNeighbour
} MATCH_CLASSIFIER;


// Image Threshold
int threshold(cv::Mat &src, cv::Mat &dst, THRESH_MODE mode, int threshold = 130, int debug =0);
// Increase color value in image
int darkenImage(cv::Mat &src, cv::Mat &dst);

// Image Clean Functions
int erosion(cv::Mat &src, cv::Mat &dst);
int dilation(cv::Mat &src, cv::Mat &dst);
int opening(cv::Mat &src, cv::Mat &dst);
int closing(cv::Mat &src, cv::Mat &dst);
int jaggedEdges(cv::Mat &src, cv::Mat&dst);
int grassfireTransform(cv::Mat &src, cv::Mat&dst, uint8_t mode);
int GTMorph(cv::Mat &src, int threshold, MORPH_OP morph_mode);

// Finding Connected Components in the Image
int findConnectedComponents(cv::Mat &src, std::vector<std::vector <float>> &component_stats);
// Compute Feature Functions
int compute_features(cv::Mat &src, std::vector<std::vector <float>> &component_stats, std::vector<std::vector <float>> &fv);
float compute_moments(cv::Mat &img, uint8_t p, uint8_t q, float cx, float cy, float area);
float compute_special_moments(cv::Mat &img, float cx, float cy, float alpha, float area);
float compute_aplha(float U11, float U20, float U02);


//Match functions
std::string feature_match(char *filename, std::vector<float> &target_feature, MATCH_CLASSIFIER mode = KNearestNeighbour, int K = 3);

//Save Feature functions
int save_new_feature(cv::Mat &frame, char *filename, char *label);

//Train Database Functions
int extract_feature(cv::Mat &frame, std::vector<float> &feature_set);
int train_folder(char *foldername, char *csv_filename, char *label);


#endif