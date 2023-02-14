#include "ofMain.h"
#include "ofApp.h"
#include <ros/ros.h>


int main( int argc, char *argv[]){

	//======================================================
	//ROS
	//======================================================
	ros::init(argc,argv,"talker");
	
	//======================================================
	//GENERAL
	//======================================================
	ofGLFWWindowSettings settings;
	settings.resizable = false;
	ofCreateWindow(settings);
	ofSetWindowShape(PICTURE_WIDTH,PICTURE_HEIGHT);
	ofRunApp(new ofApp());

	

}
