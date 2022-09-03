// main for aruco tracking


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/types_c.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdio>
#include <time.h>

using namespace cv;
using namespace std;

static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

int startWebcam(float markerDim)
{
    Mat frame;// holds the frame from a webcam
    vector<int> markerIds;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);

    vector<Vec3d> rotationVectors, translationVectors;
    namedWindow("webcam", WINDOW_AUTOSIZE);

    vector<cv::String> fn;
    glob("/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/kinect", fn, false);
    vector<Mat> images;
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i = 0; i < count; i++)
    {
        images.push_back(imread(fn[i]));
        // imshow("webcam", imread(fn[i]));
        // waitKey(1000);
    }

    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector<bool> detected;
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(1, 1, markerDim, 1e-10, markerDictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();
    Size imgSize;

    for(int i = 0; i < count; i++)
    {
    	frame = images[i];
		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);

		vector<int> markerIds_refined;
		vector<vector<Point2f>> markerCorners_refined, rejected;
		aruco::refineDetectedMarkers(frame, board, markerCorners_refined, markerIds_refined, rejected);

		if(markerIds.size() > 0)
		{
			allCorners.push_back(markerCorners);
            allIds.push_back(markerIds);
            imgSize = frame.size();
            detected.push_back(true);
		}
		else
			detected.push_back(false);
	}
	cout << "Calibrate Now:" << endl;

	Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    int calibrationFlags = 0;
    // calibrate camera
    repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                           markerCounterPerFrame, board, imgSize, cameraMatrix,
                                           distCoeffs, rvecs, tvecs, calibrationFlags);

    cout << "Reprojection Error: " << repError << endl;

    saveCameraParams("kinect-camParameters", imgSize, 1.0, calibrationFlags, cameraMatrix,
                                   distCoeffs, repError);

	cout << "Draw Now:" << endl;

	int detected_count = 0;
    FILE *fp = fopen("/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/kinect-aruco-poses.txt", "w+");
    for(int i = 0; i < count; i++)
    {
    	frame = images[i];
		// aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);

		if(detected[i])
		{
			markerCorners = allCorners[detected_count];
    		markerIds = allIds[detected_count];
    		++detected_count;
	    	// get the pose
	      	aruco::estimatePoseSingleMarkers(markerCorners, markerDim, cameraMatrix, distCoeffs, rotationVectors, translationVectors);
	      	if(rotationVectors.size() == 1 && translationVectors.size() == 1)
	      	{
	      		if(!(isfinite(rotationVectors[0][0]) || isfinite(rotationVectors[0][1]) || isfinite(rotationVectors[0][2])))
	      		{
	      			break;
	      		}
	      		if(!(isfinite(translationVectors[0][0]) || isfinite(translationVectors[0][1]) || isfinite(translationVectors[0][2])))
	      		{
	      			break;
	      		}
				cout << i << " R :" << rotationVectors[0] << " T :" << translationVectors[0] << endl;

            	fprintf(fp, "%d\t%5.8f\t%5.8f\t%5.8f\t%5.8f\t%5.8f\t%5.8f\n", 1, 
            		         rotationVectors[0][0], rotationVectors[0][1], rotationVectors[0][2],
            	             translationVectors[0][0], translationVectors[0][1], translationVectors[0][2]);
	      		for (int i = 0; i < markerIds.size(); ++i)
	      		{
		  			aruco::drawAxis(frame, cameraMatrix, distCoeffs, rotationVectors, translationVectors, 0.03f);
	      		}
	    		aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
	    	}
	    	else
	    	{
	    		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\n", 0, 0, 0, 0, 0, 0, 0);
	    	}
		}
		else
	    {
	    	fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\n", 0, 0, 0, 0, 0, 0, 0);
	    }
		char name_str[2048];
		sprintf(name_str, "/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/kinect-aruco/%05d.jpg", i);
		string name(name_str);
		imwrite(name, frame);
		// imshow("webcam", frame);
		// waitKey(1000);
    }
    fclose(fp);

    return 1;
}

bool loadCameraCalibration( string name, Mat& cameraMatrix, Mat& distortionCoefficients )
{
    ifstream inStream(name);
    if(!inStream)
		return false;

    uint16_t rows;
    uint16_t cols;

    inStream >> rows;
    inStream >> cols;

    // camera matrix
    cameraMatrix = Mat(Size(cols, rows), CV_64F);

    for (int r = 0; r < rows; ++r)
    {
		for (int c = 0; c < cols; ++c)
		{
	    	double tmp = 0.0f;
	    	inStream >> tmp;
	    	cameraMatrix.at<double>(r, c) = tmp;
		}
    }

    // distortionCoefficients
    inStream >> rows;
    inStream >> cols;

    distortionCoefficients = Mat::zeros(rows, cols, CV_64F);

    for (int r = 0; r < rows; ++r)
    {
		for (int c = 0; c < cols; ++c)
		{
	    	double tmp = 0.0f;
	    	inStream >> tmp;
	    	distortionCoefficients.at<double>(r, c) = tmp;
		}
    }

    inStream.close();

    return true;
}



    
int main(int argc, char *argv[])
{
    namedWindow("webcam", WINDOW_AUTOSIZE);

    vector<cv::String> fn;
    glob("/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/hand-eye-color", fn, false);
    vector<Mat> images;
    size_t count = fn.size(); //number of png files in images folder
    std::ofstream outputx("/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/hand-eye-color/realsense-corners_x.txt");
    std::ofstream outputy("/Users/zarathustra/Desktop/Research Works/PubMATProgs/IMUDataSets/hand-eye-robot-world-tencent6/hand-eye-color/realsense-corners_y.txt");

    for (size_t i = 0; i < count; i++)
    {
    	Mat img = imread(fn[i]);
    	Mat gray;
    	cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        images.push_back(gray);
        waitKey(1);

        std::vector<cv::Point2f> corners;
        bool found = findChessboardCorners(gray, cv::Size(8, 11), corners, cv::CALIB_CB_ADAPTIVE_THRESH);
        if(found)
        {
        	cornerSubPix(gray, corners, Size(8, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        	outputx << i << "\t";
        	outputy << i << "\t";
        	for(int j = 0; j < 88; ++j)
        	{
        		outputx << corners[j].x << "\t";
        		outputy << corners[j].y << "\t";
        	}
        	outputx << std::endl;
        	outputy << std::endl;
        }

        drawChessboardCorners(gray, cv::Size(8, 11), Mat(corners), found);
        std::cout << corners << std::endl;
        imshow("result", gray);
    }

    return 0;
}
