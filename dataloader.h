#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost\tuple\tuple.hpp>
#include <boost\filesystem.hpp>
using namespace std;
template<class T1, class T2> using DVEC = vector<boost::tuple<T1, T2>>;
void getData(DVEC<cv::Mat, cv::Point> *, DVEC<cv::Mat, cv::Point>*, DVEC<cv::Mat, cv::Point>*);

