#include "stdafx.h"
#include "MyGnuPlot.h"
using namespace std;
MyGnuPlot::MyGnuPlot() : gp("gnuplot -persist")
{
	gp << "set xrange [-1:1]\nset yrange[-1,1]\n";
	gp << "plot '-' with lines title DistancePoints\n";
}


MyGnuPlot::~MyGnuPlot()
{
	datastack.clear();
}

void MyGnuPlot::livetest()
{
	vector<boost::tuple<double, double> > pts_A;
	

	for (double alpha = -1; alpha < 10; alpha += 1.0 / 24.0)
	{
		pts_A.push_back(boost::make_tuple(alpha, alpha*alpha));
		
	}

	gp << "plot '-' with lines title 'pts_A '\n ";
	gp.send1d(pts_A);
	


#ifdef _WIN32
	cout << "Press enter to exit." << endl;
	cin.get();
#endif

	pts_A.clear();
	for (double alpha = -10; alpha < 1; alpha += 1.0 / 24.0)
	{
		pts_A.push_back(boost::make_tuple(alpha, alpha*alpha));

	}

	gp << "plot '-' with lines title 'pts_A '\n ";
	gp.send1d(pts_A);

#ifdef _WIN32
	cout << "Press enter to exit." << endl;
	cin.get();
#endif
}

void MyGnuPlot::addpoint(double x, double y){
	datastack.push_back(boost::make_tuple(x, y));
}
void MyGnuPlot::clear()
{
	datastack.clear();
}
void MyGnuPlot::graph()
{
	gp.send1d(datastack);
}
