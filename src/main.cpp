#include "ofMain.h"
#include "ofApp.h"
#include <ros/ros.h>

//========================================================================
int main( int argc, char *argv[]){


	ofGLFWWindowSettings settings;
	settings.resizable = false;
	ofSetupOpenGL(PICTURE_WIDTH,PICTURE_HEIGHT,OF_WINDOW);
		// <-------- setup the GL context
	ros::init(argc,argv,"talker");
	ofRunApp(new ofApp());

}
