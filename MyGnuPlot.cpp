#include "stdafx.h"
#include "MyGnuPlot.h"
using namespace std;
MyGnuPlot::MyGnuPlot() : gp("gnuplot -persist")
{
	gp << "set xrange [0:1]\nset yrange[0,1]\n";
	gp << "plot '-' with lines title DistancePoints\n";
}


MyGnuPlot::~MyGnuPlot()
{
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


void MyGnuPlot::graph(vector<boost::tuple<double,double>> datastack)
{
	gp << "plot '-' with lines title 'angle vs distance '\n";
	gp.send1d(datastack);
}
