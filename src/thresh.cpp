/* Abinav Anantharaman and Satwik Bhandiwad

   CS 5330 Spring 2023

   CPP functions for thresholding images based on a threshold value
   Following Thresholding techniques can be used:
    1. Simple Inverted Binary Thresholding 
        Formula:
            dst(x,y) = 0                if src(x,y) > thresh
                       max_value        otherwise
    
    2. Binary Thresholding based on saturation values of HSV image 
        Formula:
            dst(x,y) = maxValue  if src(x,y)[saturation_channel] > thresh
                        0        otherwise

    3. Otsu Threshold Method --> Binary Thresholding based on a auto threshold value found from the pixel intensities 
        Formula:
            dst(x,y) = 0                if src(x,y) > autothresh
                       mav_value        otherwise
    
    4. Modified Otsu Threshold Method
    
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>


#include "rtor.h"

int threshold(cv::Mat &src, cv::Mat &dst, THRESH_MODE mode, int threshold /*= 130*/, int debug /*= 0*/){

    
    if(src.channels() == 3){
        cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    } 

    if(mode == kThreshBinary) 
    {
        for (int i = 0; i < src.rows; i++){
            uchar *src_row_ptr  = src.ptr<uchar>(i);
            uchar *dst_row_ptr  = dst.ptr<uchar>(i);  
            for (int j = 0; j < src.cols; j++){
                dst_row_ptr[j]  = src_row_ptr[j] > threshold ? 0 : 255;
            }
        }

    }

    else if(mode == kThreshHSV) 
    {
        if(src.channels() < 3){
            printf("[WARNING] Input image should be a color image.\n");
            return 1;
        }
        cv::Mat temp;
        cv::cvtColor(src, temp, cv::COLOR_BGR2HSV);

        for (int i = 0; i < dst.rows; i++){
            uchar *row_ptr  = dst.ptr<uchar>(i); 
            cv::Vec3b *temp_row_ptr = temp.ptr<cv::Vec3b>(i);
            for (int j = 0; j < dst.cols; j++){
                if(temp_row_ptr[j][1] > threshold){
                    row_ptr[j]  =  255;
                } else{
                    row_ptr[j] =  0;
                }
            }
        }
    }

    else if(mode == kThreshOtsu)
    {
        // Caluculate histogram for 256 bins / 256 intensity values

        cv::GaussianBlur(src, src, cv::Size(7,7), 0, 0, cv::BORDER_REFLECT);

        int histData[256] = {0};
        for(int row=0; row < src.rows; row++){
            uchar *row_ptr =  src.ptr<uchar>(row); 
            for(int col=0; col < src.cols; col++){
                uchar temp = row_ptr[col];
                histData[temp] += 1;
            }
        }

        int totalPixels = dst.rows * dst.cols;
        float totalSum=0;
        float sumBackground=0; // Background
        float weightBackground =0;
        float weightForeground = 0;  //Background and Foreground weights
        float varMax=0;
        int autoThreshold =0; 
        
        for (int t=0 ; t<256 ; t++){
            totalSum += t * histData[t]; // weighted sum
            // printf("Count Value for %d = %d \n",t,histData[t]);
        } 

        // Loop through all intensity value to find the right intersity to be used for thresholding
        for ( int t=0 ; t<256 ; t++){
            weightBackground+= histData[t]; // No need to divide by pixel count
            if(weightBackground== 0) continue;  //This intensity cannot be the mid value

            weightForeground = totalPixels - weightBackground;
            //This means remaining intensity values have to be zero so, i.e. any intensity value beyond this 't' will have 0 pixel counts. 
            //break loop with already calculated var_max/auto_threshold value and go ahead
            if(weightForeground== 0) break;


            sumBackground += (float)(t * histData[t]);  // weighted sum calculated in the same way as total_sum.
            float meanBackground = sumBackground / weightBackground;  //simple mean
            float meanForeground = (totalSum  - sumBackground) / weightForeground;
        
            float varBetweenClass = weightForeground * weightBackground * (meanBackground - meanForeground) * (meanBackground -  meanForeground); //

            // Check if new maximum found
            if (varBetweenClass > varMax){
                varMax = varBetweenClass;
                autoThreshold = t; // The auto threshold value to be used for binary thresholding
            }
        }

        if(debug){
            printf("[INFO] PRINTING ALL VALUES ................\n");
            printf("WB : %f \n", weightBackground);
            printf("WF: %f \n", weightForeground);
            printf("Total Pixels: %d \n", totalPixels);
            printf("Auto Threshold Value: %d\n", autoThreshold);
        }
        //compute binary image
        for (int row = 0; row < dst.rows; row++){
            uchar *src_row_ptr  = src.ptr<uchar>(row);
            uchar *dst_row_ptr  = dst.ptr<uchar>(row); 

            for (int col = 0; col < dst.cols; col++){
                dst_row_ptr[col]  = (src_row_ptr[col] > autoThreshold ? 0 : 255);
            }
        }
    }



    else if(mode == kThreshOtsuHSV)
    {
        // Caluculate histogram for 256 bins / 256 intensity values

        cv::Mat temp;
        cv::cvtColor(src, temp, cv::COLOR_BGR2HSV);

        // cv::GaussianBlur(temp, temp, cv::Size(7,7), 0, 0, cv::BORDER_REFLECT);

        int histData[256] = {0};
        int s_bins = 16;
        int v_bins = 16;
        int s_index, v_index = 0;

        for(int row=0; row < temp.rows; row++){
            cv::Vec3b *row_ptr =  temp.ptr<cv::Vec3b>(row); 
            for(int col=0; col < temp.cols; col++){
                s_index = row_ptr[col][1]/ s_bins;
                v_index = row_ptr[col][2]/ v_bins;
                int temp_val = v_index * 16 + s_index;  // linearising a 2D histogram
                histData[temp_val] += 1;
            }
        }

        int totalPixels = dst.rows * dst.cols;
        float totalSum=0;
        float sumBackground=0; // Background
        float weightBackground =0;
        float weightForeground = 0;  //Background and Foreground weights
        float varMax=0;
        int autoThreshold =0; 
        
        for (int t=0 ; t<256 ; t++){
            totalSum += t * histData[t]; // weighted sum
            // printf("Count Value for %d = %d \n",t,histData[t]);
        } 

        // Loop through all intensity value to find the right intersity to be used for thresholding
        for ( int t=0 ; t<256 ; t++){
            weightBackground+= histData[t]; // No need to divide by pixel count
            if(weightBackground== 0) continue;  //This intensity cannot be the mid value

            weightForeground = totalPixels - weightBackground;
            //This means remaining intensity values have to be zero so, i.e. any intensity value beyond this 't' will have 0 pixel counts. 
            //break loop with already calculated var_max/auto_threshold value and go ahead
            if(weightForeground== 0) break;


            sumBackground += (float)(t * histData[t]);  // weighted sum calculated in the same way as total_sum.
            float meanBackground = sumBackground / weightBackground;  //simple mean
            float meanForeground = (totalSum  - sumBackground) / weightForeground;
        
            float varBetweenClass = weightForeground * weightBackground * (meanBackground - meanForeground) * (meanBackground -  meanForeground); //

            // Check if new maximum found
            if (varBetweenClass > varMax){
                varMax = varBetweenClass;
                autoThreshold = t; // The auto threshold value to be used for binary thresholding
            }
        }

        printf("[INFO] PRINTING ALL VALUES ................\n");
        printf("WB : %f \n", weightBackground);
        printf("WF: %f \n", weightForeground);
        printf("Total Pixels: %d \n", totalPixels);
        printf("Auto Threshold Value: %d\n", autoThreshold);

        int threshold_v_index = autoThreshold/(int) 16;
        int threshold_s_index = autoThreshold % (int) 16;

        //compute binary image
        for (int row = 0; row < temp.rows; row++){
            uchar *dst_row_ptr  = dst.ptr<uchar>(row);
            cv::Vec3b *temp_row_ptr =  temp.ptr<cv::Vec3b>(row); 
            for (int col = 0; col < dst.cols; col++){
                dst_row_ptr[col]  = (temp_row_ptr[col][1] >= threshold_s_index*16 || temp_row_ptr[col][2] < threshold_v_index*16 ? 255 : 0);
            }
        }
    }
    return 0;
}




int darkenImage(cv::Mat& src, cv::Mat& dst){

    cv::Mat hsv_image;
    cv::cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);

    for(int row = 0; row < hsv_image.rows; row++){
        cv::Vec3b *row_ptr = hsv_image.ptr<cv::Vec3b>(row);
        for(int col = 0; col < hsv_image.cols ; ++col){
            // If color saturation is greater than 150 reduce pixel intensity
            if(row_ptr[col][1] >= 70){
                row_ptr[col][2] = row_ptr[col][2] *0.5;  // darkening pixel with high saturation value by reducing the intesity/value by 2 times
            } 
            // else if(row_ptr[col][1] <= 40){
            //     row_ptr[col][2] = 255;
            // }
        }
    }
    // Convert back to RGB color space
    cv::cvtColor(hsv_image, dst, cv::COLOR_HSV2BGR);
    return 0;
}