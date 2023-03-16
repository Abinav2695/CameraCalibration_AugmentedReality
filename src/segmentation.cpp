/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition Segmentation Functions
   Source file
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>

#include "rtor.h"


int findConnectedComponents(cv::Mat &src, std::vector<std::vector <float>> &component_stats){
// int findConnectedComponents(cv::Mat &src, cv::Mat &dst, int N){

    // Compute the connected components
    cv::Mat labels, stats, centroids;
    int num_components = cv::connectedComponentsWithStats(src, labels, stats, centroids);

    printf("Total number of components : %d\n", num_components);

    // Display the results
    if(num_components >=2){

        for (int i = 2; i <= num_components; ++i) {
        // Retrieve the statistics for component i

            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            int area = stats.at<int>(i, cv::CC_STAT_AREA); //M00
            int cx = centroids.at<double>(i, 0);
            int cy = centroids.at<double>(i, 1);

            if(area >=5000 && area <= 200000 && cx!=0 && cy!=0){
                // Draw a bounding box around component i
                // cv::rectangle(dst, cv::Rect(left, top, width, height), cv::Scalar(50, 255, 50), 2);
                // Print the centroid coordinates and area of component i
                
                printf("Component %d: centroid = (%d, %d),  width, height = (%d, %d), area = %d\n", i, cx, cy, width, height, area);
                std::vector<float> temp;
                temp.push_back(left);
                temp.push_back(top);
                temp.push_back(width);
                temp.push_back(height);
                temp.push_back(area);
                temp.push_back(cx);
                temp.push_back(cy);

                component_stats.push_back(temp);
            }
        }
    }
    std::cout << labels.size() << std::endl;
    // cv::Mat labeled_image = cv::Mat::zeros(labels.size(),CV_8UC3);
    // for (int i = 0; i < labels.rows; ++i){
    //     int *src_row_ptr = labels.ptr<int>(i);
    //     cv::Vec3b *dst_row_ptr = labeled_image.ptr<cv::Vec3b>(i);
    //     for (int j = 0; j < labels.cols; ++j){
    //         if(src_row_ptr[j] == 1){
    //             // printf(" %d \n",src_row_ptr[j]);
    //             dst_row_ptr[j][0] = 200;
    //             dst_row_ptr[j][1] = 0;
    //             dst_row_ptr[j][2] = 0;
    //         } else if(src_row_ptr[j] == 2){
                
    //             dst_row_ptr[j][0] = 0;
    //             dst_row_ptr[j][1] = 200;
    //             dst_row_ptr[j][2] = 0;
    //         } else if(src_row_ptr[j] == 3){
    //             dst_row_ptr[j][0] = 0;
    //             dst_row_ptr[j][1] = 0;
    //             dst_row_ptr[j][2] = 200;
    //         } else if(src_row_ptr[j] == 4){
    //             dst_row_ptr[j][0] = 150;
    //             dst_row_ptr[j][1] = 150;
    //             dst_row_ptr[j][2] = 0;
    //         } else if(src_row_ptr[j] == 5){
    //             dst_row_ptr[j][0] = 150;
    //             dst_row_ptr[j][1] = 0;
    //             dst_row_ptr[j][2] = 150;
    //         } else if(src_row_ptr[j] == 6){
    //             dst_row_ptr[j][0] = 0;
    //             dst_row_ptr[j][1] = 150;
    //             dst_row_ptr[j][2] = 150;
    //         } else if(src_row_ptr[j] == 7){
    //             dst_row_ptr[j][0] = 100;
    //             dst_row_ptr[j][1] = 100;
    //             dst_row_ptr[j][2] = 100;
    //         } else if(src_row_ptr[j] == 8){
    //             dst_row_ptr[j][0] = 150;
    //             dst_row_ptr[j][1] = 250;
    //             dst_row_ptr[j][2] = 150;
    //         } else if(src_row_ptr[j] == 9){
    //             dst_row_ptr[j][0] = 50;
    //             dst_row_ptr[j][1] = 100;
    //             dst_row_ptr[j][2] = 150;
    //         } else if(src_row_ptr[j] == 10){
    //             dst_row_ptr[j][0] = 20;
    //             dst_row_ptr[j][1] = 250;
    //             dst_row_ptr[j][2] = 20;
    //         } else if(src_row_ptr[j] == 11){
    //             dst_row_ptr[j][0] = 250;
    //             dst_row_ptr[j][1] = 250;
    //             dst_row_ptr[j][2] = 80;
    //         } else if(src_row_ptr[j] == 12){
    //             dst_row_ptr[j][0] = 50;
    //             dst_row_ptr[j][1] = 50;
    //             dst_row_ptr[j][2] = 250;
    //         } else{
    //             dst_row_ptr[j][0] = 0;
    //             dst_row_ptr[j][1] = 0;
    //             dst_row_ptr[j][2] = 0;
    //         }
    //     }
    // }
    // cv::imshow("Labeled_Image", labeled_image);
    // cv::waitKey(0);
    return 0;
}


int compute_features(cv::Mat &src, std::vector<std::vector <float>> &component_stats, std::vector<std::vector <float>> &fv){

    for (auto &component : component_stats){
        cv::Rect roi(component[0], component[1], component[2], component[3]);
        cv::Mat new_image = src(roi);

        float area_bounding_box = component[2] * component[3];
        float percentage_filled = component[4]/ area_bounding_box;  //Feature 1
        float h_w_ratio = component[3]/ component[2];  // Feature 2

        float M00 = compute_moments(new_image, 0, 0, 0,0,1);
        float M10 = compute_moments(new_image, 1, 0, 0,0,1);
        float M01 = compute_moments(new_image, 0, 1, 0,0,1);

        float centroid_x = M10/ M00;
        float centroid_y = M01/ M00;
        
        float U11 =  compute_moments(new_image, 1, 1, centroid_x, centroid_y, M00); //central moments (translation and scale invariant)
        float U20 =  compute_moments(new_image, 2, 0, centroid_x, centroid_y, M00); //central moments (translation and scale invariant)
        float U02 =  compute_moments(new_image, 0, 2, centroid_x, centroid_y, M00); //central moments (translation and scale invariant)
        float U30 = compute_moments(new_image, 3, 0, centroid_x, centroid_y, M00);  //central moments (translation and scale invariant)
        float U03 = compute_moments(new_image, 0, 3, centroid_x, centroid_y, M00);
        float U12 = compute_moments(new_image, 1, 2, centroid_x, centroid_y, M00);
        float U21 = compute_moments(new_image, 2, 1, centroid_x, centroid_y, M00);

        float alpha  = compute_aplha(U11, U20, U02); //orientation_angle
        
        float H0 = U20 + U02; //Feature 4
        float H1 = ((U20 - U02) * (U20 - U02)) + 4*U11*U11; //Feature 5
        float H2 = ((U30 - 3*U12) *  (U30 - 3*U12)) + ((U21 - 3*U03) *  (U21 - 3*U03)); //Feature 6
        float H3 = ((U30 + U12) *  (U30 + U12)) + ((U21 + U03) *  (U21 + U03)); //Feature 7
        // float U22_alpha = compute_special_moments(new_image, centroid_x, centroid_y, alpha, M00); //Feature 7

        std::vector<float> temp_fv;
        temp_fv.push_back(percentage_filled);
        temp_fv.push_back(h_w_ratio);
        
        temp_fv.push_back(H0);
        temp_fv.push_back(H1);
        temp_fv.push_back(H2);
        temp_fv.push_back(H3);
        // temp_fv.push_back(alpha);

        fv.push_back(temp_fv);
       
    }
    return 0;
}

float compute_moments(cv::Mat &img, uint8_t p, uint8_t q, float cx, float cy, float area){

    double moment = 0.0f;
    for (int row=0; row < img.rows; ++row){
        uchar *row_ptr = img.ptr<uchar>(row);
        for (int col=0; col < img.cols; ++col){
            moment += (pow((row - cx),p) * pow((col - cy),q) * (row_ptr[col]/255));  
        }
    }
    moment = moment / pow(area , (0.5*(p+q) + 1));  //Normalize the moments
    return moment;
}

float compute_special_moments(cv::Mat &img, float cx, float cy, float alpha, float area){

    double moment = 0.0f;
    for (int row=0; row < img.rows; ++row){
        uchar *row_ptr = img.ptr<uchar>(row);
        for (int col=0; col < img.cols; ++col){
            moment += pow(((col-cy)*cos(alpha) + (row - cx)*sin(alpha)) , 2);  
        }
    }
    moment = moment / area;
    return moment;
}

float compute_aplha(float U11, float U20, float U02){

    float alpha = 0.0f;
    alpha = 0.5 * atan2((2*U11), (U20 - U02));
    return alpha;
}