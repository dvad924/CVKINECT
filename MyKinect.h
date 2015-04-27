#pragma once
#include <Kinect.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/tuple/tuple.hpp>
#include <opencv2/opencv.hpp>
#include "MyGnuPlot.h"
typedef void(*infdepthbodCallback)(IInfraredFrame*, IDepthFrame*,IBodyFrame*);
/*
	Class to Represent how interfacing and connections will
	be done with the kinect sensor.
	Initial objective is to use Infrared and Depth frames
*/
using namespace::std;
#define BODYCOUNT 6
#define XCOOR 0
#define YCOOR 1
#define DIST 1
#define ANGLE 0
class MyKinect
{
	const string title = "Infrared";
	IKinectSensor* m_sensor;
	HRESULT m_hResult;
	IMultiSourceFrameReader* m_reader;
	ICoordinateMapper * pCoordinateMapper;
	cv::Mat infraredMat;
	cv::Mat depthMat;
	cv::Mat handMat;
	cv::Mat morphFilter;
	cv::Mat zoom;
	cv::Mat borderMat;
	cv::Point start;
	vector<boost::tuple<int, int>> ptsvector;
	vector<boost::tuple<double, double>> objvector;
	MyGnuPlot plotter;
public:
	MyKinect();
	~MyKinect();
	bool check(string str);
	IKinectSensor* getSensor();
	void processFrames();
	void handleFrames(IInfraredFrame*, IDepthFrame*, IBodyFrame*);
	HRESULT getRightHandPoints(DepthSpacePoint dsp[], IBodyFrame*, int * length);	
	bool findNextBPoint(int cx, int cy, int w1, int w2, cv::Mat img, int *newx, int* newy);
	void getContour(cv::Point center, cv::Mat objFrame, vector<boost::tuple<int, int>> * ptsvector);
	void getContourBFS(cv::Point center, cv::Mat objFrame, vector<boost::tuple<int, int>> * ptsvector);
	void calcAnglesAndDistances(cv::Point center, vector<boost::tuple<int, int>>*ptsvector, vector<boost::tuple<double,double>>*objvector);
};

