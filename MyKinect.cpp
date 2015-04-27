#include "stdafx.h"
#include "MyKinect.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <queue>
using namespace std;
const double PI = 3.1415926535897;
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

MyKinect::MyKinect() :
infraredMat(424, 512, CV_8UC1),
depthMat(424, 512, CV_8UC1),
handMat(424, 512, CV_32FC1),
zoom(424, 512, CV_8UC1),
borderMat(424, 512, CV_32FC1),
morphFilter(cv::Mat::ones(cv::Size(3,3),CV_8UC1)),
pCoordinateMapper(nullptr)

{
	m_hResult = GetDefaultKinectSensor(&m_sensor); //upon creation set the sensor;
	if (!check("Getting Sensor"))
		throw - 1;
	m_hResult = m_sensor->Open();		//attempt to open a connection
	if (!check("Opening Sensor"))
		throw - 1;
	//Open the MultiSource Stream watching for Infrared, Depth and Body frames
	m_hResult = m_sensor->OpenMultiSourceFrameReader(FrameSourceTypes_Infrared | FrameSourceTypes_Depth |FrameSourceTypes_Body  ,&m_reader);
	if (!check("Opening Stream"))
		throw - 1;
	m_hResult = m_sensor->get_CoordinateMapper(&pCoordinateMapper);
	if (!check("Retrieving Coordinate Mapper"))
		throw - 1;
	cv::namedWindow("Infrared");
	cv::namedWindow("Depth");
	//cv::namedWindow("Transform");
	cv::namedWindow("Zoom");
	//cv::namedWindow("Border");
}

MyKinect::~MyKinect()
{
	SafeRelease(m_reader);
	if (m_sensor != nullptr)
		m_sensor->Close();
	SafeRelease(m_sensor);
}


bool MyKinect::check(string str)
{
	if (FAILED(m_hResult))
	{
		cerr << " Error : " + str << endl;
		int dummy;
		cin >> dummy;
		return false;
	}
	return true;
}

IKinectSensor* MyKinect::getSensor()
{
	return m_sensor;
}

void MyKinect::processFrames()
{
	try{
		while (1){
					IMultiSourceFrame *msf;
					m_hResult = m_reader->AcquireLatestFrame(&msf);
					if (SUCCEEDED(m_hResult)){
						HRESULT inf, dep, bod;
						IInfraredFrameReference* infRef = nullptr;
						IDepthFrameReference* dfRef = nullptr;
						IBodyFrameReference * bfRef = nullptr;
						inf = msf->get_InfraredFrameReference(&infRef);
						dep = msf->get_DepthFrameReference(&dfRef);
						bod = msf->get_BodyFrameReference(&bfRef);
						if (SUCCEEDED(inf) && SUCCEEDED(dep) && SUCCEEDED(bod)){
							IInfraredFrame* iframe = nullptr;
							IDepthFrame* dframe = nullptr;
							IBodyFrame* bframe = nullptr;
							inf = infRef->AcquireFrame(&iframe);
							dep = dfRef->AcquireFrame(&dframe);
							bod = bfRef->AcquireFrame(&bframe);
							if (SUCCEEDED(inf) && SUCCEEDED(dep) && SUCCEEDED(bod))
							{
								handleFrames(iframe, dframe, bframe);
							}//process frames
							SafeRelease(iframe); SafeRelease(dframe); SafeRelease(bframe);
						}//mult refs
						SafeRelease(infRef); SafeRelease(dfRef); SafeRelease(bfRef);
					}//frame
					SafeRelease(msf);
					cv::imshow("Infrared", infraredMat);
					cv::imshow("Depth", depthMat);
					//cv::imshow("Transform", handMat);
					cv::imshow("Zoom", zoom);
					//cv::imshow("Border", borderMat);
					if (cv::waitKey(30) == VK_ESCAPE){
						break;
					}
			
		}//while

	}
	catch (int escape){
		cout << "Ending Subscription" << escape << endl;
		return;
	}
} 

void MyKinect::handleFrames(IInfraredFrame*inf, IDepthFrame*dep, IBodyFrame*bod)
{
	
	unsigned int irbuffersize = 0, depbuffersize = 0;
	unsigned short* irbuff = nullptr, *depbuff = nullptr;

	HRESULT irRes, depRes;
	irRes = inf->AccessUnderlyingBuffer(&irbuffersize, &irbuff);
	depRes = dep->AccessUnderlyingBuffer(&depbuffersize, &depbuff);
	if (SUCCEEDED(irRes) && SUCCEEDED(depRes)){
		for (int y = 0; y < 424; y++){
			for (int x = 0; x < 512; x++){
				unsigned int index = y * 512 +x;
				infraredMat.at<unsigned char>(y, x) = irbuff[index] >> 8;
				depthMat.at<unsigned char>(y, x) = depbuff[index] >> 5;		//max of 8000mm so we only care about first 13 bits
			}
		}
		DepthSpacePoint handPoints[BODYCOUNT];
		int numpoints;
		m_hResult = getRightHandPoints(handPoints, bod, &numpoints);
		if (SUCCEEDED(m_hResult))
		{
			
				int x = static_cast<int>(handPoints[0].X);
				int y = static_cast<int>(handPoints[0].Y);
				if ((x >= 0) && (x < 512) && (y >= 0) && (y < 424))
				{
					unsigned int trackpoint = static_cast<unsigned int>(depthMat.at<unsigned char>(y, x));
					cout << "hand depth: " << trackpoint << endl;
					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					cv::Size dims(depthMat.size());
					int ymin = max(y - 49, 0);
					int ymax = min(y + 50, dims.height);
					int xmin = max(x - 49, 0);
					int xmax = min(x + 50, dims.width);
					depthMat(cv::Rect_<int>(0, 0, dims.width, ymin)) = 0;					
					depthMat(cv::Rect_<int>(0, ymin, xmin, dims.height - ymin)) = 0;
					depthMat(cv::Rect_<int>(xmax, ymin, dims.width - xmax, dims.height - ymin)) = 0;
					depthMat(cv::Rect_<int>(xmin, ymax, xmax-xmin, dims.height - ymax)) = 0;
					//////////////////////////////////////////////Clear everything not in the 100x100 frame about the hand /////////////
					for (int i = ymin; i < ymax; i++)
					{
						for (int j = xmin; j < xmax; j++)
						{
								unsigned int cpoint = depbuff[i * 512 + j] >> 5;
								depthMat.at<unsigned char>(i, j) = ((cpoint >= trackpoint - 4) && (cpoint <= trackpoint + 1)) ? 255 : 0;	
						}						
					}
				
					cv::distanceTransform(depthMat, handMat, CV_DIST_L1, 3); //Use L2 norm with a 3x3 to compute distance transform
					cv::threshold(handMat, borderMat, 1, 0, CV_THRESH_TOZERO_INV);
					cv::threshold(zoom, zoom, 0, 0, CV_THRESH_TOZERO_INV);
					//cv::morphologyEx(depthMat, depthMat, 4, morphFilter);
					//cv::normalize(handMat, handMat, 0.0, 1.0, cv::NORM_MINMAX); //Threshold and normalized to get intensities
					double min, max;
					
					cv::Point minp, centerp;
					cv::minMaxLoc(handMat, &min, &max, &minp, &centerp);
					
					cv::circle(infraredMat, cv::Point(x, y), 40, cv::Scalar(90.0, 128.0), 5, CV_AA);
					
					//cv::circle(depthMat, maxp, 2, cv::Scalar(90, 128.0, 0.0, 0.0));
					getContourBFS(centerp, borderMat, &ptsvector);
					calcAnglesAndDistances(centerp, &ptsvector, &objvector);
					plotter.graph(objvector);


					ptsvector.clear();	//memory wipe the unneeded image data
					objvector.clear();

				} //if point is in the frame.
			
		} //bodypoints
	} //access buffers
}
HRESULT MyKinect::getRightHandPoints(DepthSpacePoint dsp[], IBodyFrame*bodf,int *length){

	int dspcount = 0;
	IBody* bodies[BODYCOUNT] = {0};
	HRESULT hr = bodf->GetAndRefreshBodyData(BODY_COUNT, bodies);
	if (SUCCEEDED(hr)){
		for (int i = 0; i < BODYCOUNT; i++)
		{
			BOOLEAN bTracked = false;
			hr = bodies[i]->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joint[JointType::JointType_Count];
				hr = bodies[i]->GetJoints(JointType_Count, joint);
				if (SUCCEEDED(hr))
				{
					HandState handState = HandState::HandState_Unknown;
					hr = bodies[i]->get_HandRightState(&handState);
					if (SUCCEEDED(hr))
					{
						DepthSpacePoint depthSpacePoint = { 0 };
						hr = pCoordinateMapper->MapCameraPointToDepthSpace(joint[JointType::JointType_HandRight].Position, &depthSpacePoint);
						if (SUCCEEDED(hr)){
							dsp[dspcount++] = depthSpacePoint;
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < BODYCOUNT; i++){
		SafeRelease(bodies[i]);
	}
	*length = dspcount;
	return hr;
}

inline void rangemapper(int *start, int*end, int dy, int dx){
	if (dy == -1)
	{
		if (dx == -1)
		{
			(*start) = 6;
			(*end) = 10;
		}
		else if (dx == 0)
		{
			(*start) = 0;
			(*end) = 2;
		}
		else
		{
			(*start) = 0;
			(*end) = 4;
		}
	}
	else if (dy == 0)
	{
		if (dx == -1)
		{
			(*start) = 6;
			(*end) = 8;
		}
		else if (dx == 0)
		{
			throw EXCEPTION_DEBUG_EVENT;
		}
		else
		{
			(*start) = 2;
			(*end) = 4;
		}
	}
	else
	{
		if (dx == -1)
		{
			(*start) = 4;
			(*end) = 8;
		}
		else if (dx == 0)
		{
			(*start) = 4;
			(*end) = 6;
		}
		else
		{
			(*start) = 2;
			(*end) = 6;
		}
	}
}
inline void cellmapper(int *y, int *x, int input, int posy, int posx){
	switch (input % 8)
	{
		case(0) :
			(*y) = posy - 1;
			(*x) = posx - 1;
			break;
		case(1) :
			(*y) = posy - 1;
			(*x) = posx;
			break;
		case(2) :
			(*y) = posy - 1;
			(*x) = posx + 1;
			break;
		case(3) :
			(*y) = posy;
			(*x) = posx + 1;
			break;
		case(4) :
			(*y) = posy + 1;
			(*x) = posx + 1;
			break;
		case(5) :
			(*y) = posy + 1;
			(*x) = posx;
			break;
		case(6) :
			(*y) = posy + 1;
			(*x) = posx - 1;
			break;
		case(7) :
			(*y) = posy;
			(*x) = posx - 1;
			break;
	}
}
//Temp function for locateing initial crawl point
int getLeftMost(int init, int y, cv::Mat frame){
	int v2, v1;
	for (int x = init; x > 0; x--){
		
		v1 = frame.at<float>(y, x);
		v2 = frame.at<float>(y, x - 1);
		if (v1 != 0 && v2 == 0)
			return x;
	}
	return -1;
}
inline bool okpoint(int cx, int cy, cv::Mat img){
	return (cx >= 0) && (cx < img.cols) && (cy >= 0) && (cy < img.rows);
}
//assumes cx,cy are a boundary point
bool isBPoint(int cx, int cy, cv::Mat img){
	int px, py;
	for (int i = -1; i < 2; i++){
		for (int j = -1; j < 2; j++){
			if (i == 0 && j == 0)
				continue;
			px = cx + i;
			py = cy + j;
			if (!okpoint(px, py, img))
				continue;
			if (img.at<float>(py, px) == 0)
				return true;
		}
	}
	return false;
}

void MyKinect::getContourBFS(cv::Point center, cv::Mat bframe, vector<boost::tuple<int, int>> * ptsvector)
{
	//zoom is the minion image for checks
	//Since we are doing breadth first seach we need a queue;
	queue<cv::Point> pointq;
	start.x = getLeftMost(center.x, center.y, bframe); //find a starting point from the center of the contour hole
	start.y = center.y;
	cv::Point currentPoint;
	if (start.x == -1)
		return;
	zoom.at<unsigned char>(start.y, start.x) = 255;	//mark that we have visited the first point
	int searchx, searchy;
	pointq.push(start);								//put starting point in the queue;
	while (!pointq.empty())
	{
		//mark our tracker that we have visited this place
		currentPoint = pointq.front();
		pointq.pop();

		for (int i = 0; i <= 8; i++){
			cellmapper(&searchy, &searchx, i, currentPoint.y, currentPoint.x); //get the next cell we want to look at;
			if (!okpoint(searchx, searchy, bframe))							   //if the current point is in the frame
				continue;
			if (bframe.at<float>(searchy,searchx) == 0.0 || zoom.at<unsigned char>(searchy, searchx) != 0) //its not background or hasn't been visited already
				continue;
			if (isBPoint(searchx, searchy, bframe))	//and its a boundary point
			{
				pointq.push(cv::Point(searchx, searchy));						//push that point into the queue;
				ptsvector->push_back(boost::make_tuple(searchx, searchy));		//and add it to our list of boundary points
				zoom.at<unsigned char>(searchy, searchx) = 255;
			}
		}
		
	}
	
}
bool MyKinect::findNextBPoint(int cx, int cy, int w1,int w2, cv::Mat img, int *newx, int* newy){
	int searchx, searchy;
	double val;
	for (int i = w1; i <= w2; i++)
	{

		cellmapper(&searchy, &searchx, i, cy, cx);
		if (!okpoint(searchx, searchy, img))
			continue;
		val = img.at<float>(searchy, searchx);
		if (zoom.at<unsigned char>(searchy, searchx) != 0)
			continue;
		if (val != 1.0)
			continue;
		if (isBPoint(searchx, searchy, img)){
			zoom.at<unsigned char>(searchy, searchx) = 255;
			(*newx) = searchx;
			(*newy) = searchy;
			return true;
		}
	}
	

	return false;
}

void MyKinect::getContour(cv::Point center, cv::Mat bframe, vector<boost::tuple<int, int>> * ptsvector){
	int cx, cy;
	int curx = -1, cury = -1;
	int tempx, tempy, dy, dx,w1,w2;
	bool found;
	cx = center.x;
	cy = center.y;
	start.y = cy;
	start.x = getLeftMost(cx, cy, bframe);
	if (start.x == -1)
		return;
	ptsvector->push_back(boost::make_tuple(start.x, start.y));
	found = findNextBPoint(start.x, start.y, 0, 8, bframe,&curx,&cury);
	if (!found) ///if we couldn't find another boundary point return;
		return;

	dx = curx - start.x; //Movement vector
	dy = cury - start.y;
	do{
		ptsvector->push_back(boost::make_tuple(curx, cury));
		tempx = curx;
		tempy = cury;
		rangemapper(&w1, &w2, dy, dx);
		found = findNextBPoint(tempx, tempy, 0, 8, bframe, &curx, &cury);
		if (!found)
		{
			break;
		}
			
		dy = cury - tempy; //update movement vector;
		dx = curx - tempx;

	} while (curx != start.x || cury != start.y);

}


inline double distance2(int x1,int y1,int x2,int y2){
	return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

/*Given a central point a reference point and a current point calculate the angle
  between the two vectors originating at the center point
  */
inline double calcAngle(int x1, int y1, cv::Point start, cv::Point center){
	int x2 = start.x - center.x;		//get vector from center to starting point
	int y2 = start.y - center.y;
	x1 = x1 - center.x;					//get vector from center to current point;
	y1 = y1 - center.y;
	double dotprod = x1 * x2 + y1*y2;	//calculate dot product;
	double det = x1*y2 - x2*y1;			//calculate determinant
	double anglenpi2pi = std::atan2(det, dotprod);	//calculate angle based on these 2;
	if (anglenpi2pi < 0.0)
		return (anglenpi2pi) / (2 * PI) + 1;	//map [-pi,0] --> [pi,2pi] and normalize
	else
		return anglenpi2pi / (2 * PI);			//otherwise just normalize
}

void MyKinect::calcAnglesAndDistances(cv::Point center, vector<boost::tuple<int, int>> *ptsvector, vector<boost::tuple<double,double>> *objvector)
{
	double maxdist = 0.0;
	double dist = 0.0;
	double angle;
	int x, y;
	for (int i = 0; i < ptsvector->size(); i++)
	{
		x = ptsvector->at(i).get<XCOOR>(); //get x and y coordinates of points of interest;
		y = ptsvector->at(i).get<YCOOR>();

		dist = distance2(x, y, center.x, center.y);//calculate distance squared of current point from center
		if (dist > maxdist)
		{
			maxdist = dist;
		}
		angle = calcAngle(x, y, start, center);	   //calculate angle from starting point to current point
		objvector->push_back(boost::make_tuple(angle, dist));
	}
	for (int i = 0; i < objvector->size(); i++)
	{
		objvector->at(i).get<DIST>() /= maxdist;
	}
}