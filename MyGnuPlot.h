#pragma once
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>
class MyGnuPlot
{
	Gnuplot gp;
	std::vector<boost::tuple<double, double>> datastack;

public:
	MyGnuPlot();
	~MyGnuPlot();
	void livetest();
	void addpoint(double, double);
	void clear();
	void graph();
};

