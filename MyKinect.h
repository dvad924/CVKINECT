#pragma once
#include <Kinect.h>
#include <string>
#include <opencv2/opencv.hpp>
typedef void(*infdepthbodCallback)(IInfraredFrame*, IDepthFrame*,IBodyFrame*);
/*
	Class to Represent how interfacing and connections will
	be done with the kinect sensor.
	Initial objective is to use Infrared and Depth frames
*/
using namespace::std;
#define BODYCOUNT 6
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
	vector<vector<cv::Point>> contours;

public:
	MyKinect();
	~MyKinect();
	bool check(string str);
	IKinectSensor* getSensor();
	void processFrames();
	void handleFrames(IInfraredFrame*, IDepthFrame*, IBodyFrame*);
	HRESULT getRightHandPoints(DepthSpacePoint dsp[], IBodyFrame*, int * length);	
	
};

