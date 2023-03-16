/* Abinav Anantharaman and Satwik Bhandiwad
   CS 5330 Spring 2023
   Real Time Object Recognition Morph Operation Functions
   Source file
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "rtor.h"

// Running a 5x5 kernel with full of ones
// A pixel in the original image (either 1 or 0) will be considered 1 only if all the pixels under the kernel is 1, 
// otherwise it is eroded (made to zero)

int erosion(cv::Mat &src, cv::Mat &dst){

    cv::Mat temp;
    cv::copyMakeBorder(src, temp, 2, 2, 2, 2, cv::BORDER_REFLECT101); //padding of 2 on all sides

    for (int row = 2; row < temp.rows-2; ++row){
        uchar *temp_row_ptr = temp.ptr<uchar>(row);
        uchar *temp_row_ptr_m1 = temp.ptr<uchar>(row-1); //row -1
        uchar *temp_row_ptr_m2 = temp.ptr<uchar>(row-2); //row -2
        uchar *temp_row_ptr_p1 = temp.ptr<uchar>(row+1); //row +1
        uchar *temp_row_ptr_p2 = temp.ptr<uchar>(row+2); //row +2

        uchar *dst_ptr = dst.ptr<uchar>(row-2);
        for (int col = 2; col < temp.cols-2; ++col){
            uchar erode = 0xFF;
            for (int k = col - 2; k <= col + 2; ++k){
                erode = erode & temp_row_ptr[k];
                erode = erode & temp_row_ptr_m1[k];
                erode = erode & temp_row_ptr_m2[k];
                erode = erode & temp_row_ptr_p1[k];
                erode = erode & temp_row_ptr_p2[k];
            }

            if(!erode){
                dst_ptr[col-2] = 0;
            } else {
                dst_ptr[col-2] = 255;
            }
        }
    }

    return 0;
}

int dilation(cv::Mat &src, cv::Mat &dst){

    cv::Mat temp;
    cv::copyMakeBorder(src, temp, 2, 2, 2, 2, cv::BORDER_REFLECT101); //padding of 2 on all sides

    for (int row = 2; row < temp.rows-2; ++row){
        uchar *temp_row_ptr = temp.ptr<uchar>(row);
        uchar *temp_row_ptr_m1 = temp.ptr<uchar>(row-1); //row -1
        uchar *temp_row_ptr_m2 = temp.ptr<uchar>(row-2); //row -2
        uchar *temp_row_ptr_p1 = temp.ptr<uchar>(row+1); //row +1
        uchar *temp_row_ptr_p2 = temp.ptr<uchar>(row+2); //row +2

        uchar *dst_ptr = dst.ptr<uchar>(row-2);
        for (int col = 2; col < temp.cols-2; ++col){
            uchar erode = 0x00;
            for (int k = col - 2; k <= col + 2; ++k){
                erode = erode | temp_row_ptr[k];
                erode = erode | temp_row_ptr_m1[k];
                erode = erode | temp_row_ptr_m2[k];
                erode = erode | temp_row_ptr_p1[k];
                erode = erode | temp_row_ptr_p2[k];
            }

            if(!erode){
                dst_ptr[col-2] = 0;
            } else {
                dst_ptr[col-2] = 255;
            }
        }
    }
    return 0;
}

int opening(cv::Mat &src, cv::Mat &dst){

    if(src.channels() > 1){
        printf("[WARNING] Input image should be a greyscale image.\n");
        return 1;
    }

    cv::Mat erode = cv::Mat::zeros(src.size(), CV_8UC1);
    erosion(src,erode);
    dilation(erode, dst); 
    return 0;

}

int closing(cv::Mat &src, cv::Mat &dst){

    if(src.channels() > 1){
        printf("[WARNING] Input image should be a greyscale image.\n");
        return 1;
    }

    cv::Mat dilate = cv::Mat::zeros(src.size(), CV_8UC1);
    dilation(src,dilate);
    erosion(dilate, dst); 
    
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // // Perform opening operation
    // //cv::Mat closed_image;
    // morphologyEx(src, dst, cv::MORPH_OPEN, element);
    return 0;


}

int jaggedEdges(cv::Mat &src, cv::Mat&dst){
    //2 erosions followed by 2 dilations followed by 2 erosions

    cv::Mat temp = cv::Mat::zeros(src.size(), CV_8UC1);
    erosion(src, temp);
    erosion(temp, temp);
    for (int i = 0; i < 4; ++i) dilation(temp, temp);

    erosion(temp, temp);
    erosion(temp, dst);

    return 0;
}



int grassfireTransform(cv::Mat &src, cv::Mat&dst, uint8_t mode){
    
    cv::Mat temp;
    cv::copyMakeBorder(src, temp,1,1,1,1,cv::BORDER_CONSTANT, cv::Scalar(0)); //padding of 1 on all sides
    dst = cv::Mat::zeros(temp.size(),CV_16UC1);

    uint8_t value_to_compare = 0; //Foregroud under consideration 
    if(!mode) value_to_compare = 255; // Backgroud under consideration
    // 1st pass
    for (int row = 1; row < temp.rows -1; ++row){
        uint16_t *dst_row_ptr = dst.ptr<uint16_t>(row);
        uint16_t *dst_row_ptr_p1 = dst.ptr<uint16_t>(row -1); //previous row
        uchar *temp_row_ptr = temp.ptr<uchar>(row);

        for(int col = 1; col< temp.cols -1; ++col){
            if(temp_row_ptr[col] == value_to_compare){
                //4 connected 
                dst_row_ptr[col] = 0;
            } else{
                dst_row_ptr[col] = 1 +  (dst_row_ptr[col-1] < dst_row_ptr_p1[col] ? dst_row_ptr[col-1] : dst_row_ptr_p1[col]); // 1+ min(west and north pixel)
            }
        }
    }

    //2nd pass
    for (int row = dst.rows -1 ; row > 0; --row){
        uint16_t *dst_row_ptr = dst.ptr<uint16_t>(row);
        uint16_t *dst_row_ptr_p1 = dst.ptr<uint16_t>(row+1);
        for(int col = dst.cols - 1; col > 0; --col){
            if(dst_row_ptr[col]>0){
                //4 connected 
                int temp = 1 +  (dst_row_ptr[col+1] < dst_row_ptr_p1[col] ? dst_row_ptr[col+1] : dst_row_ptr_p1[col]); // 1+ min(east and south pixel)
                dst_row_ptr[col] = dst_row_ptr[col] < temp ? dst_row_ptr[col] : temp;
            }
        }
    }

    cv::Rect roi(1, 1,src.cols,src.rows);
    dst = dst(roi);
    return 0;
}

int GTMorph(cv::Mat &src, int threshold, MORPH_OP morph_mode){

    cv::Mat temp;

    if (morph_mode == kOpeningMode){  //opening (erosion followed by dilation)
        grassfireTransform(src, temp, 1); //for erosion
        for (int i = 0; i < src.rows; i++){
            uchar *dst_row_ptr = src.ptr<uchar>(i);
            uint16_t *temp_row_ptr = temp.ptr<uint16_t>(i);
            for(int j = 0; j < src.cols; j++){
                if(temp_row_ptr[j] <= threshold) {
                    dst_row_ptr[j] = 0;
                }
            }
        }

        grassfireTransform(src, temp, 0); //for dilation
        for (int i = 0; i < src.rows; i++){
            uchar *dst_row_ptr = src.ptr<uchar>(i);
            uint16_t *temp_row_ptr = temp.ptr<uint16_t>(i);
            for(int j = 0; j < src.cols; j++){
                if(temp_row_ptr[j] <= threshold) {
                    dst_row_ptr[j] = 255;
                }
            }
        }
    } else if(morph_mode == kClosingMode){//closing (dilation followed by erosion)

        grassfireTransform(src, temp, 0); //for dilation
        for (int i = 0; i < src.rows; i++){
            uchar *dst_row_ptr = src.ptr<uchar>(i);
            uint16_t *temp_row_ptr = temp.ptr<uint16_t>(i);
            for(int j = 0; j < src.cols; j++){
                if(temp_row_ptr[j] <= threshold) {
                    dst_row_ptr[j] = 255;
                }
            }
        }

        grassfireTransform(src, temp, 1); //for dilation
        for (int i = 0; i < src.rows; i++){
            uchar *dst_row_ptr = src.ptr<uchar>(i);
            uint16_t *temp_row_ptr = temp.ptr<uint16_t>(i);
            for(int j = 0; j < src.cols; j++){
                if(temp_row_ptr[j] <= threshold) {
                    dst_row_ptr[j] = 0;
                }
            }
        }

    }
    return 0;
}