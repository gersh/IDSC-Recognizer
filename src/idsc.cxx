#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include "shape_descriptor.hpp"

struct our_descriptor {
	double descriptors[64*20];
	int klass;
};
std::string getName(const char *buffer) {
	std::string str(buffer);
	int pos=str.find_last_of('/');
	str=str.substr(pos+1);
	pos=0;
	while(str[pos]<'0' || str[pos]>'9') {
		pos++;
	}
	return(str.substr(0,pos-1));
}
int main(int argc, char **argv) {
	using namespace std;
	using namespace cv;

	if(argc < 2) {
		printf("plantid [imagename] [imagename] ...\n");
		return 0;
	}
	std::map<std::string,int> klasses;
	std::list<our_descriptor*> descriptor;
	for(int n=0;n<(argc-1);n++) {
		Mat image = cv::imread(argv[n+1]);
		Mat thImage;
		Mat nC;
		Mat sb;
	
		cv::cvtColor(image,nC,CV_BGR2GRAY);
		cv::threshold(nC,thImage,128,255,THRESH_BINARY);
		vector<vector<Point> > contours;
		cv::findContours(nC,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);			
		
		int ourContour=0;
		int maxContourSize=0;
		for(int l=0;l<contours.size();l++) {
			if(contours[l].size()>maxContourSize) {
				ourContour=l;
				maxContourSize=contours[l].size();
			}
		}
		shape_descriptor sd;
		our_descriptor *od=new our_descriptor;
		std::string name=getName(argv[n+1]);
		if(klasses.find(name)==klasses.end()) {
			klasses[name]=klasses.size();
		}
		od->klass=klasses[name];
		sd.getFullShapeContext(od->descriptors,contours[ourContour]);
		sd.normalizeShapeContext(od->descriptors);
		descriptor.push_back(od);
		for(int l=0;l<20;l++) {
			std::cout << n << "," << name;
			for(int m=0;m<64;m++) {
				std:cout << "," << od->descriptors[l*64+m];
			}
			std::cout << std::endl;
		}
		delete od;
	}

	
	return 0;
}
