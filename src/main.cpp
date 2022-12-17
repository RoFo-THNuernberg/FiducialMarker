#include "ofMain.h"
#include "ofApp.h"
#include <ros/ros.h>

//========================================================================
int main( int argc, char *argv[]){

	int i,j;

	ofGLFWWindowSettings settings;
	settings.resizable = false;
	ofSetupOpenGL(PICTURE_WIDTH,PICTURE_HEIGHT,OF_WINDOW);			// <-------- setup the GL context
	ros::init(argc,argv,"talker");
	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
