
#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup(){

    ofSetLogLevel(OF_LOG_ERROR);

    kinect.setRegistration(true);
    kinect.init(true, true);
    kinect.open();

    camWidth 		= kinect.getWidth();
	camHeight 		= kinect.getHeight();

	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

    colorImg.allocate(camWidth,camHeight);
    depthImage.allocate(camWidth, camHeight);

	threshold = 200;

	// Box 2D
	box2d.init();
	box2d.setGravity(0, 10);

	numberCirclesMax = 50;



    // timer stuff
    previousTime = 0;
    inverse = false;
}

//--------------------------------------------------------------
void ofApp::update(){

    if (!kinect.isConnected()){
        kinect.open();
    }


    if (circles.size() < numberCirclesMax){
        circles.push_back(createCircle(ofRandom(0, camWidth), 0, ofRandom(4, 20)));
    }

    ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));

	ofBackground(100,100,100);

    // Updates to Box2d elements
	box2d.update();
    // remove all box2d that are out of the screen
	ofRemove(circles, ofxBox2dBaseShape::shouldRemoveOffScreen);

    // update kinect feed
    kinect.update();

	if (kinect.isFrameNew()){
        colorImg.setFromPixels(kinect.getPixels(), camWidth, camHeight);
        depthImage.setFromPixels(kinect.getDepthPixels(), camWidth, camHeight);
		depthImage.threshold(threshold);
		contourFinder.findContours(depthImage, 50, (camWidth*camHeight), 5, false, true);
	}


    edgeLine.clear();
    edgeLine.resize(contourFinder.blobs.size());
    for (unsigned int i = 0; i < contourFinder.blobs.size(); i++){
        drawing.addVertices(contourFinder.blobs[i].pts);
        edgeLine[i].addVertexes(drawing);
        drawing.clear();
        edgeLine[i].setPhysics(0.0, 0.5, 0.5);
        edgeLine[i].create(box2d.getWorld());
    }



    for (auto &edge : edgeLine){
        if (!(edge.getVertices().size() < 3)){
            circles.erase(std::remove_if(circles.begin(), circles.end(),
                       [&](shared_ptr<ofxBox2dCircle> &circle){
                       return edge.inside(circle->getPosition().x, circle->getPosition().y);
                       }), circles.end());
        }
    }



}

//--------------------------------------------------------------
void ofApp::draw(){


	ofSetHexColor(0xffffff);

    //colorImg.draw(0,0);
    depthImage.draw(0, 0);

    ofSetHexColor(0xff0000);
    for (auto &edge : edgeLine){
        edge.draw();
    }


    for(auto circle : circles) {
		ofFill();
		ofSetHexColor(0x90d4e3);
		circle->draw();
	}
}

void ofApp::exit(){
    kinect.close();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'c') {
		float r = ofRandom(4, 20);
		circles.push_back(createCircle(mouseX, mouseY, r));
	}
}

std::shared_ptr<ofxBox2dCircle> ofApp::createCircle(int x, int y, int radius){
    std::shared_ptr<ofxBox2dCircle> c(new ofxBox2dCircle());
    c->setPhysics(3.0, 0.52, 0.1);
    c->setup(box2d.getWorld(), x, y, radius);
    return c;
}
