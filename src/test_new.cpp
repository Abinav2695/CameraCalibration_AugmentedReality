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

const float sqDimension = 0.034f;
const Size boardDimension = Size(6,9);

void generateKnownBoardPos(Size boardSize, float sqEdgeLength, vector<Point3f> &corners) {
    for(int i=0; i<boardSize.height; i++) {
        for(int j=0; j<boardSize.width; j++) {
            corners.push_back(Point3f(j*sqEdgeLength, i*sqEdgeLength, 0));
        }

    }
}

void getBoardCorners(vector<Mat> images, vector<vector<Point2f> > &foundCorners, bool showRes) {

    for(vector<Mat>::iterator it = images.begin(); it!= images.end(); it++) {
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*it, Size(9,6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found) {
            foundCorners.push_back(pointBuf);
        }
        if(showRes) {
            drawChessboardCorners(*it, Size(6,9),  pointBuf, found);
            imshow("corners", *it);
            waitKey(0); 
        }
    }

}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float sqEdgeLength, Mat &cameraMatrix, Mat &distCoefficients){

   vector<vector<Point2f>>  boardImageSpacePoints;

   getBoardCorners(calibrationImages, boardImageSpacePoints, false);

   vector<vector<Point3f>> worldSpaceCornerPoints(1);

   generateKnownBoardPos(boardSize, sqEdgeLength, worldSpaceCornerPoints[0]);

   worldSpaceCornerPoints.resize(boardImageSpacePoints.size(),worldSpaceCornerPoints[0]);

   vector<Mat> rVecs, tVecs;

   distCoefficients = Mat::zeros(8, 1, CV_64F);

   //CV_CALIB_ZERO_TANGENT_DIST
   double rms = calibrateCamera(worldSpaceCornerPoints, boardImageSpacePoints, boardSize, cameraMatrix, distCoefficients, rVecs, tVecs, CV_CALIB_FIX_ASPECT_RATIO );
   cout << "Reprojection error: " << rms << endl;
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

    Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);

    Mat distanceCoefficients;

    vector<Mat> savedImages;

    //vector<vector<Point2f> > markerCorners, rejectedCandidates;


    VideoCapture vid(2);

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

        found = findChessboardCorners(frame, boardDimension, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame , boardDimension, foundPoints, found);

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
                
            case 'q':
            //quit
            return 0;
            break;
        }

    }

    return 0;
}