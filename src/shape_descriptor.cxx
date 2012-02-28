#include "shape_descriptor.hpp"
#include <vector>
#include <list>
#include <stack>
#include <math.h>
#include <iostream>

using namespace cv;
double shape_descriptor::dist(const Point &pt1,const Point &pt2) {
	double xDist=pt1.x-pt2.x;
	double yDist=pt1.y-pt2.y;
	return(sqrt(xDist*xDist+yDist*yDist));	
}

double shape_descriptor::angle(const Point &p1,const Point &p2) {
	double vX=p2.x-p1.x;
	double vY=p2.y-p1.y;
	
	return(atan2(vY,vX));
}
#define NUM_PTS 20
void shape_descriptor::getFullShapeContext(double *sc,const std::vector<Point> &contour) 
{	
	int num=0;
	
	for(int n=0;n<NUM_PTS*64;n++) {
		sc[n]=0;
	}
	// Must have at least 2 points
	if(contour.size() <= 2) 
	{
		return;
	}
	bool orientation=getContourOrientation(contour);
	if(contour.size()<=30) {
		for(int n=0;n<contour.size();n++) {
			double scI[64];
			getShapeContext(scI,contour,n,orientation);	
		}
	}
	else {
		cv::RNG rng;
		for(int n=0;n<NUM_PTS;n++) {		
			int start=contour.size()*n/NUM_PTS;
			int end=start+contour.size()/NUM_PTS;
			if(end>contour.size()) {
				end=contour.size();
			}
			getShapeContext(sc,contour,rng.uniform(start,end),orientation);
			sc+=64;
		}
	}

}

bool shape_descriptor::intersect(const Point &p1a,const Point &p1b,const Point &p2a,const Point &p2b) {
	double v1X=p1b.x-p1a.x;
	double v1Y=p1b.y-p1b.y;
	double v2X=p2b.x-p2a.x;
	double v2Y=p2b.y-p2a.y;
	
	double cross=v1X*v2Y-v1Y*v2X;
	if(cross<1e-6) {
		return(false);
	}
	double xI = ((p1a.x*p1b.y-p1a.y*p2b.x)*v2Y-v1Y*(p2a.x*p2b.y-p2a.y*p2b.x))/cross;
	return(xI>=min(p1a.x,p2a.x)&&xI<=max(p1a.x,p2a.x)&&xI>=min(p2a.x,p2b.x)&&xI<=max(p2a.x,p2b.x));
	
}

int shape_descriptor::moveLeft(int n, int size) {
	if(--n<0) {
		n=size-1;
	}
	return(n);
}
int shape_descriptor::moveRight(int n, int size) {
	if(++n==size) {
		n=0;
	}
	return(n);
}
void shape_descriptor::getContext(double *sc) {
	for(int k=0;k<64;k++) {
		sc[k]=0;	
	}

	for(std::list<ShapeDistance*>::iterator i=dists.begin();i!=dists.end();i++) {
		int pos=(int)floor(log(1+(*i)->d)/log(1+maxDist)*8.0)*8+(int)floor((*i)->theta/(2.0*M_PI)*8.0);
		sc[pos]++;	
		delete(*i);	
	}
	dists.clear();
}

void shape_descriptor::addShapeDist(double dist, double angle) {
	while(angle<0) {
		angle+=M_PI*2;
	}
	while(angle > 2*M_PI) {
		angle-=2*M_PI;
	}
	if(dist>maxDist) {
		maxDist=dist;
	}
	ShapeDistance *sd=new ShapeDistance();
	sd->d=dist;
	sd->theta=angle;
	dists.push_back(sd);	
}
bool shape_descriptor::getContourOrientation(const std::vector<Point> &contour) {
	reference_point rp(&contour[0],0,true);
	rp.next_angle(contour[1]);
	double minAngle=0;
	double maxAngle=0;
	double delta=rp.last_angle();
	for(int n=1;n<contour.size();n++) {
		rp.next_angle(contour[n]);
		double angle=rp.last_angle()-delta;
		double maxDist;
		double minDist;
		if(angle>=maxAngle) {
			maxDist=angle-maxAngle;
		}
		else {
			maxDist=(angle+2*M_PI-maxAngle);
		}
		if(angle<=minAngle) {
			minDist=minAngle-angle;
		}
		else {
			minDist=minAngle-angle+2*M_PI;
		}
		if(minDist<maxDist) {
			if(minDist < M_PI/2.0) {
				minAngle=angle;
			}
		}
		else {
			if(maxDist < M_PI/2.0) {
				maxAngle=angle;
			}
		}
	}
	double midAngle;
	if(maxAngle>minAngle) {
		midAngle=(maxAngle+minAngle)/2.0;
	}
	else {
		midAngle=maxAngle+minAngle+M_PI;
		if(midAngle>= 2*M_PI) {
			midAngle-=2*M_PI;
		}
	}
	return(midAngle < M_PI);
	
}

void shape_descriptor::getShapeContext(double *sc,const std::vector<Point> &contour, int ptNum, bool orientation) {
	maxDist=0;
	std::stack<reference_point*> srfp_r, srfp_l;
	reference_point *rfp_r= new reference_point(&contour[ptNum],0,orientation);
	reference_point *rfp_l=new reference_point(&contour[ptNum],0,!orientation);
	int n=moveRight(ptNum,contour.size());
	int m=moveLeft(ptNum,contour.size());
	rfp_r->dist(&contour[n]);
	rfp_l->dist(&contour[m]);
	rfp_r->next_angle(contour[n]);
	rfp_l->next_angle(contour[m]);
	double theta_l=0;
	double theta_r=0;
	double delta_l=-rfp_l->last_angle();
	double delta_r=-rfp_r->last_angle();
	while(n!=m) {
		if(rfp_r->last_dist() <= rfp_l->last_dist()) {
			//MOVING RIGHT
			n=moveRight(n,contour.size());
			if(rfp_r->next_angle(contour[n])) {
				//CONVEX
				while(!srfp_r.empty() && srfp_r.top()->next_angle(contour[n])) {
					delete rfp_r;
					rfp_r = srfp_r.top();
					srfp_r.pop();
				}
				if(srfp_r.empty()) {
					theta_r=rfp_r->last_angle()+delta_r;
				}
				rfp_r->dist(&contour[n]);
				addShapeDist(rfp_r->last_dist(),theta_r);
			}
			else {
				//CONCAVE
				srfp_r.push(rfp_r);
				rfp_r=rfp_r->spawn();
				rfp_r->dist(&contour[n]);
				addShapeDist(rfp_r->last_dist(),theta_r);
			}
		}
		else {
			
			//MOVING LEFT
			m=moveLeft(m,contour.size());
			if(rfp_l->next_angle(contour[m])) {
				//CONVEX
				while(!srfp_l.empty() && srfp_l.top()->next_angle(contour[m])) {
					
					delete rfp_l;
					rfp_l = srfp_l.top();
					srfp_l.pop();
				}
				if(srfp_l.empty()) {
					theta_l=rfp_l->last_angle()+delta_l;
				}
				rfp_l->dist(&contour[n]);
				addShapeDist(rfp_l->last_dist(),theta_l);
			}
			else {
				//CONCAVE
				srfp_l.push(rfp_l);
				rfp_l=rfp_l->spawn();
				rfp_l->dist(&contour[m]);
				addShapeDist(rfp_l->last_dist(),theta_l);
			}
			
		}
	}
	//Cleanup
	while(!srfp_r.empty()) {
		delete srfp_r.top();
		srfp_r.pop();
	}
	while(!srfp_l.empty()) {
		delete srfp_l.top();
		srfp_l.pop();
	}
	delete rfp_r;
	delete rfp_l;
	
	getContext(sc);
	
}	
void shape_descriptor::normalizeShapeContext(double *sc)
{
	for(int m=0;m<NUM_PTS;m++) {
		double sum=0;
		for(int n=0;n<64;n++) {
			sum+=sc[64*m+n];
		}
		for(int n=0;n<64;n++) {
				sc[64*m+n]/=sum;
		}
	}
}
cv::Mat shape_descriptor::graphSc(double *sc, int width, int height)
{
	double max=-1;
	double min=1e6;
	for(int n=0;n<64;n++) {
		if(sc[n]<min)
			min=sc[n];
		if(sc[n]>max)
			max=sc[n];
	}
	cv::Mat mat(height,width,CV_8UC3);
	std::cout <<"CHART ";
	for(int n=0;n<8;n++	) {
		for(int m=0;m<8;m++) {
			double scale= (sc[n*8+m]-min)/(max-min);
			std::cout <<  "," << scale;
			cv::Scalar color(scale*255.0,scale*255.0,scale*255.0);
			cv::rectangle(mat,cv::Point(n*(width/8),m*(height/8)),cv::Point((n+1)*(width/8),(m+1)*(height/8)),color,-1);
		}
	}
	std::cout << std::endl;
	return(mat);
}
