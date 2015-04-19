#include "stdafx.h"
#include "MyKinect.h"
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;

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
	cv::namedWindow("Transform");
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
		//WAITABLE_HANDLE msfWaitable;
		//m_hResult = m_reader->SubscribeMultiSourceFrameArrived(&msfWaitable);
		//if (!check("Subscription"))
			//throw - 1;
		while (1){
			//HANDLE hEvents[] = { reinterpret_cast<HANDLE>(msfWaitable) };
			//WaitForMultipleObjects(ARRAYSIZE(hEvents), hEvents, true, INFINITE);

			//IMultiSourceFrameArrivedEventArgs* pArgs;
			//m_hResult = m_reader->GetMultiSourceFrameArrivedEventData(msfWaitable, &pArgs);
			//if (SUCCEEDED(m_hResult)){
				//IMultiSourceFrameReference * msfRef;
				//m_hResult = pArgs->get_FrameReference(&msfRef);
				//if (SUCCEEDED(m_hResult)){
					IMultiSourceFrame *msf;
					//m_hResult = msfRef->AcquireFrame(&msf);
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
					cv::imshow("Transform", handMat);
					if (cv::waitKey(30) == VK_ESCAPE){
						break;
					}
				//}//reference
				//SafeRelease(msfRef);
			//}//args
			//SafeRelease(pArgs);
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
			for (int i = 0; i < numpoints; i++)
			{
				int x = static_cast<int>(handPoints[i].X);
				int y = static_cast<int>(handPoints[i].Y);
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
					cout << "xmin: " << xmin << ", xmax: " << xmax << ", ymin: " << ymin << ", ymax: " << ymax << ", height: " << dims.height << ", width: " << dims.width << endl;
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
								depthMat.at<unsigned char>(i, j) = ((cpoint >= trackpoint - 1) && (cpoint <= trackpoint + 2)) ? 255 : 0;	
						}						
					}
				
					cv::distanceTransform(depthMat, handMat, CV_DIST_L2, 3); //Use L2 norm with a 3x3 to compute distance transform
					cv::normalize(handMat, handMat, 0.0, 1.0, cv::NORM_MINMAX); //Threshold and normalized to get intensities
					double min, max;
					cv::Point minp, maxp;
					cv::minMaxLoc(handMat, &min, &max, &minp, &maxp);
					cv::morphologyEx(depthMat, depthMat, 4, morphFilter);
					cv::circle(infraredMat, cv::Point(x, y), 40, cv::Scalar(90.0, 128.0), 5, CV_AA);
					cv::circle(depthMat, maxp, 2, cv::Scalar(90, 128.0, 0.0, 0.0));
					
				}
			}
		}
	}
}
HRESULT MyKinect::getRightHandPoints(DepthSpacePoint dsp[], IBodyFrame*bodf,int *length){

	int dspcount = 0;
	IBody* bodies[BODYCOUNT] = { 0 };
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
	*length = dspcount;
	return hr;
}