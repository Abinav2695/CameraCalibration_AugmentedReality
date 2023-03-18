#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
//#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

const float sqDimension = 0.03f;
const Size boardDimension = Size(9,6);


// std::string IMAGE_FOLDER_PATH = "/CameraCalibration_AugmentedReality/calib_images/";
std::string IMAGE_FOLDER_PATH = "/CameraCalibration_AugmentedReality/images/";
// int image_number[26] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};
int image_number[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};

std::string file_path = __FILE__;
std::string PROJECT_FOLDER_PATH = file_path.substr(0, file_path.rfind("/CameraCalibration_AugmentedReality"));


void generateKnownBoardPos(Size boardSize, float sqEdgeLength, vector<Point3f> &corners) {
    for(int i=0; i<boardSize.height; i++) {
        for(int j=0; j<boardSize.width; j++) {
            corners.push_back(Point3f(j*sqEdgeLength, i*sqEdgeLength, 0));
        }

    }
}

void getBoardCorners(vector<Mat> &images, vector<vector<Point2f> > &foundCorners, bool showRes) {

    for(auto img:images) {
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(img, boardDimension, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found) {
            foundCorners.push_back(pointBuf);
        }
        if(showRes) {
            drawChessboardCorners(img, boardDimension,  pointBuf, found);
            imshow("corners", img);
            waitKey(0); 
        }
    }

}

void cameraCalibration(vector<Mat> &calibrationImages, Size boardSize, float sqEdgeLength, Mat &cameraMatrix, Mat &distCoefficients){

    vector<vector<Point2f>>  boardImageSpacePoints;

    getBoardCorners(calibrationImages, boardImageSpacePoints, true);

    vector<vector<Point3f>> worldSpaceCornerPoints(1);

    generateKnownBoardPos(boardSize, sqEdgeLength, worldSpaceCornerPoints[0]);

    worldSpaceCornerPoints.resize(boardImageSpacePoints.size(),worldSpaceCornerPoints[0]);

    vector<Mat> rVecs, tVecs;

    //    distCoefficients = Mat::zeros(8, 1, CV_64F);

    //CV_CALIB_ZERO_TANGENT_DIST
    double rms = calibrateCamera(worldSpaceCornerPoints, boardImageSpacePoints, calibrationImages[0].size(), cameraMatrix, distCoefficients, rVecs, tVecs);
    std::cout << "Error: " << rms << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoefficients << std::endl;
}

bool saveCameraCalibration(string filename, Mat cameraMatrix, Mat distCoefficients) {

    ofstream outStream(filename);
    if(outStream) {
        uint16_t rows = cameraMatrix.rows;
        uint16_t cols = cameraMatrix.cols;

        for(int i=0; i<rows; i++) {
            for(int j=0; j<cols; j++) { 

                double val = cameraMatrix.at<double>(i,j);

                outStream << val << endl;
            } 
        }

        rows = distCoefficients.rows;
        cols = distCoefficients.cols;

        for(int i=0; i<rows; i++) {
            for(int j=0; j<cols; j++) { 

                double val = distCoefficients.at<double>(i,j);

                outStream << val << endl;
            } 
        }

        outStream.close();
        return true;
    }
    return false;

}


int main() {
    Mat frame;
    Mat drawToFrame;

    // Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
    Mat cameraMatrix;

    Mat distanceCoefficients;

    vector<Mat> savedImages;

    //vector<vector<Point2f> > markerCorners, rejectedCandidates;


    VideoCapture vid(0);

    if(!vid.isOpened()) {
        cerr << "ERROR: Cannot open webcam" << endl;
        return -1;
    }

    int framesPerSecond = 20;

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE );

    int count = 1;

    while(true) {
        if(!vid.read(frame)) {
            break;
        }

        vector<Vec2f> foundPoints;
        bool found = false;

        // found = findChessboardCorners(frame, boardDimension, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        // frame.copyTo(drawToFrame);
        // drawChessboardCorners(drawToFrame , boardDimension, foundPoints, found);

        if(found) {
            imshow("Webcam", drawToFrame);
        } else {
            imshow("Webcam", frame);
        }

        char key = waitKey(10);

        switch (key) {

            case 's':
            //save frame for calibration
            if(found) {
               Mat tmp;
               frame.copyTo(tmp);
               savedImages.push_back(tmp);
               //save the image
               string filename = to_string(count) + ".jpg";
               imwrite(filename, tmp);
               count++;
               cout << savedImages.size() << " Frame saved" << endl;
            }
            break;

            case 'c':
            if(savedImages.size() >= 10) {

                //Initialise cameraMatrix
                
                cameraMatrix.at<double>(0, 0) = 1;
                cameraMatrix.at<double>(0, 1) = 0;
                cameraMatrix.at<double>(0, 2) = frame.cols/2;

                cameraMatrix.at<double>(1, 0) = 0;
                cameraMatrix.at<double>(1, 1) = 1;
                cameraMatrix.at<double>(1, 2) = frame.rows/2;

                cameraMatrix.at<double>(2, 0) = 0;
                cameraMatrix.at<double>(2, 1) = 0;
                cameraMatrix.at<double>(2, 2) = 1;
                

                cameraCalibration(savedImages, boardDimension, sqDimension, cameraMatrix, distanceCoefficients);
                saveCameraCalibration("CameraCalibParams", cameraMatrix, distanceCoefficients);
                break;
            } else {
                cout << "Not enough images to calibrate. Save 10 or more to start calibration." << endl;
                break;
            }
                
            case 'd':
            {
                std::vector<cv::Mat> savedImages;
                for(auto i:image_number)
                {
                    // cap->read(frame); //get new frame
                    
                    frame = cv::imread(PROJECT_FOLDER_PATH + IMAGE_FOLDER_PATH + std::to_string(i)+ ".jpg");
                    if(frame.empty()){
                        printf("ERROR: Failed to read image");
                        continue;
                    }
                    savedImages.push_back(frame);
                }
                cameraCalibration(savedImages, boardDimension, sqDimension, cameraMatrix, distanceCoefficients);
                saveCameraCalibration("CameraCalibParams", cameraMatrix, distanceCoefficients);
                break;
            }
            case 'q':
            //quit
            return 0;
            break;
        }

    }

    return 0;
}