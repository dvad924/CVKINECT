#pragma once
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>

class MyGnuPlot
{
	Gnuplot gp;
	

public:
	MyGnuPlot();
	~MyGnuPlot();
	void livetest();
	void graph(std::vector<boost::tuple<double,double>>);
};

