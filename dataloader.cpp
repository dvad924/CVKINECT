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

inline cv::Mat makesig(SIGTYPE tsig){
	cv::Mat sig(tsig.size(), 3, CV_32FC1);
	for (int i = 0; i < tsig.size(); i++){
		float weight = tsig.at(i).get<2>();
		float theta1 = tsig.at(i).get<0>();
		float theta2 = tsig.at(i).get<1>();
		sig.at<float>(i, 0) = weight;
		sig.at<float>(i, 1) = theta1;
		sig.at<float>(i, 2) = theta2;
	}
	return sig;
}

void runLooTest(vector<SIGTYPE> cs, vector<SIGTYPE> rs, vector<SIGTYPE> ts){
	vector<boost::tuple<SIGTYPE, string>> testbin;
	for (int i = 0; i < cs.size(); i++){
		testbin.push_back(boost::make_tuple(cs.at(i), "C"));
	}
	for (int i = 0; i < rs.size(); i++){
		testbin.push_back(boost::make_tuple(rs.at(i), "RockOn"));
	}
	for (int i = 0; i < ts.size(); i++){
		testbin.push_back(boost::make_tuple(ts.at(i), "ThumbsUp"));
	}

	double numcorrect = 0.0;
	double total = testbin.size();
	double numtoofar = 0.0;
	for (int i = 0; i < testbin.size(); i++){
		boost::tuple<SIGTYPE, string> test = testbin.at(i); //pick element from the list
		testbin.erase(testbin.begin() + i);
		SIGTYPE onesig = test.get<0>();
		string label = test.get<1>();
		cv::Mat curr = makesig(onesig);
		double mn = 2;
		double score;
		string mnlabel("");
		for (int j = 0; j < testbin.size(); j++){ //run test without it
			SIGTYPE sig = testbin.at(j).get<0>();
			string lab = testbin.at(j).get<1>();
			cv::Mat t = makesig(sig);
			score = cv::EMD(curr, t, CV_DIST_L2);
			if (score < mn){
				mn = score;
				mnlabel = lab;
			}
		}
		if (mnlabel == label ){
			numcorrect += 1.0;
		}
		if (mn >= 0.15){
			numtoofar += 1.0;
		}
		testbin.insert(testbin.begin() + i, test); //put it back
	}
	cout << "LOO accuracy: " << numcorrect / total << endl;
	cout << "dist accuracy: " << numtoofar / total << endl;
}