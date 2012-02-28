#ifndef REFERENCE_POINT_H
#define REFERENCE_POINT_H

#include <math.h>
#include <opencv2/core/core.hpp>

using namespace cv;
class reference_point {
	// Our reference pt
	const Point *_refPt;
	// Last Point we got distance from
	const Point *_lastPt;
	// Max angle we have gotten to
	double _maxTheta;
	// 
	double _deltaTheta;
	// The angle to the last point
	double _lastTheta;
	// Distance of point from the origin
	double _deltaDist;
	// Last distance we calculated
	double _lastDist;
	// Should angle be increasing or decreasing
	bool _parity;
public:
	// Initializes reference point 
	reference_point(const Point *pt, double delta, bool parity);
	// Determines the next angle, and returns true if we are moving in the right direction
	bool next_angle(const Point &pt);
	reference_point* spawn() ;
	void dist(const Point *pt);
	double last_dist();
	double last_angle();
};

#endif
