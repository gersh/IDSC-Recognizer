#include "reference_point.hpp"
#include <iostream>
using namespace cv;

reference_point::reference_point(const Point *pt, double delta, bool parity) :  _deltaDist(delta), _maxTheta(0), _parity(parity)
{
	_refPt=pt;
}
// Determines the next angle, and returns true if we are moving in the right direction
bool reference_point::next_angle(const Point &pt) {
	double x=pt.x-_refPt->x;
	double y=pt.y-_refPt->y;
	_lastTheta=atan2(y,x) + M_PI;
		
	if( (_parity && (_lastTheta >= _maxTheta || (_maxTheta > 3.0*M_PI/4.0 && _lastTheta < M_PI/4.0)))    
		|| (!_parity && (_lastTheta <= _maxTheta || (_maxTheta < M_PI/4.0 && _lastTheta > 3*M_PI/4.0)))
			)
		 {
		_maxTheta=_lastTheta;
		return true;
	}
	else 
		return false;
}
reference_point* reference_point::spawn() {
	return(new reference_point(_lastPt,_lastDist,_parity));
}
void reference_point::dist(const Point *pt) {	
	double x=pt->x-_refPt->x;
	double y=pt->y-_refPt->y;
	_lastPt=pt;
	_lastDist=sqrt(x*x+y*y);
}
double reference_point::last_dist() {
	return(_lastDist);
}
double reference_point::last_angle() {
	return(_lastTheta);
}
