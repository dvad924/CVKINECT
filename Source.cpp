#include "stdafx.h"
#include "MyKinect.h"

int main(int argc, char* argv[])
{
	cv::setUseOptimized(true);
	MyKinect * mk = new MyKinect();
	mk->processFrames();
	return 0;
}