#ifndef HEADER_APP
#define HEADER_APP

#include <iostream>
#include <memory>
#include <vector>

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxBox2d.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void exit();

		std::shared_ptr<ofxBox2dCircle> createCircle(int x, int y, int radius);

private:
        // ofxOpenCV
        ofxCvColorImage			colorImg;
        ofxCvGrayscaleImage 	depthImage;
        ofxCvContourFinder 	contourFinder;

		int 				threshold;

        // ofxKinect
		int camWidth;
		int camHeight;
        ofxKinect kinect;

        // ofxBox2d
        ofxBox2d box2d;
        std::vector < std::shared_ptr<ofxBox2dCircle> > circles;
        std::deque<ofxBox2dEdge> edgeLine;
        ofPolyline drawing;

        int numberCirclesMax;


        // Timer
        int previousTime;
        bool inverse;
};

#endif
