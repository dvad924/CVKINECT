#include "dataloader.h"
namespace fs = boost::filesystem;

void getFileNames(vector<string> *fnames, string subdir){
	fs::path rootDir("C:\\dump\\" + subdir);
	fs::directory_iterator end;


	if (fs::exists(rootDir) && fs::is_directory(rootDir)){
		for (fs::directory_iterator iterator(rootDir); iterator != end; iterator++){
			if (fs::is_regular_file(iterator->status()))
			{
				fnames->push_back(iterator->path().string());
			}
		}
	}
}

void getAllFileNames(vector<string> *fnames){
	vector<string> temp;
	getFileNames(&temp,"c");
	getFileNames(&temp, "rock");
	getFileNames(&temp, "tup");
	for (int i = 0; i < temp.size(); i++){
		string name = temp.at(i);
		int k = name.find("border");
		if (k >= 0){
			fnames->push_back(name);
		}
	}
	temp.clear();
}

cv::Point extractCenter(string fname){
	cv::Point center;
	int end = fname.rfind("_");
	int start = fname.find("_");
	string coors = fname.substr(start + 1, end);
	end = coors.find("_");
	string xstr = coors.substr(0, end);
	string ystr = coors.substr(end + 1, coors.size());
	center.x = stoi(xstr);
	center.y = stoi(ystr);
	return center;
}

void getData(DVEC<cv::Mat, cv::Point> *c, DVEC<cv::Mat, cv::Point> *rock, DVEC<cv::Mat, cv::Point> *tup){
	vector<string> fnames;
	getAllFileNames(&fnames);
	
	for (int i = 0; i < fnames.size(); i++){
		string path = fnames.at(i);
		
		
		

		if ((int)(path.find("\\c\\")) >= 0)
		{
			c->push_back(boost::make_tuple(cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE), extractCenter(path)));
		}
		else if ((int)path.find("\\rock\\") >= 0)
		{
			rock->push_back(boost::make_tuple(cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE), extractCenter(path)));
		}
		else if ((int)path.find("\\tup\\") >=0 )
		{
			tup->push_back(boost::make_tuple(cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE), extractCenter(path)));
		}
	}
}

