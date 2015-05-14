#include "stdafx.h"
#include "MyKinect.h"
#include <string>
#include <cctype>
using namespace std;
int main(int argc, char* argv[])
{
	cv::setUseOptimized(true);
	MyKinect * mk;
	if (argc > 1){
		 mk = new MyKinect(true);
	}
	else{
		 mk = new MyKinect(false);
	}
		
	mk->processFrames();
	return 0;
	
	
}