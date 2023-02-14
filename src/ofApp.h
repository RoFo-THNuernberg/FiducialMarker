#pragma once

#include "ofMain.h"
#include "ofxTuio.h"
#include "ofxGui.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

#define PICTURE_WIDTH 1280 		//width of OF-Window !!same resolution as reacTIVision!!
#define PICTURE_HEIGHT 720 		//height of OF-Window !!same resolution as reacTIVision!!		
#define ROBOT_RADIUS 20			//size of drawn robots



class ofApp : public ofBaseApp{

	public:

		//======================================================
		// General 
		//======================================================
		stringstream framerate; 										//display framerate
		string s_width = ""; 											//display width value
		string s_height = ""; 											//display height value
		int xField = 0, yField = 0, widthField = 0, heightField = 0; 	//x-coordinate, y-coordinate, width and height of the Field
		int widthValReal = 0, heightValReal = 0; 						//real size of the Field entered by user
		bool setField = 0,newInitialized = 0; 							//draw Field and throw error message

		//======================================================
		// TUIO 
		//======================================================
		struct object { //relevant data
			ofVec2f pos;
			long sessionID;
			long objectID;
			long  angle;
			float xReal;
			float yReal;
		};
		vector<object> objects; 	//track multiple robots
		ofxTuioReceiver tuio; 		//receiver for TUIO messages

		//======================================================
		//ROS
		//======================================================
		ros::NodeHandle n_; 								//nodehandler
		ros::Publisher pub_; 								//publisher
		const std::string pub_sub_name = "/robot_1/pose";  	//name for publishing

		//======================================================
		//GUI
		//======================================================
		ofxPanel gui;
		ofxIntField ofxIF_heightInput; 		//integer input field for height
		ofxIntField ofxIF_widthInput;		//integer input field for width
		ofxTextField ofxTF_Error;			// to indicate wrong initialization
		ofxTextField ofxTF_Status;			// status of initialization
		ofxButton ofxB_initializeField; 	// button to initialize field

		//======================================================
		//METHODS
		//======================================================
		void setup();	//for setup, called only once
		void update();	//called every cycle
		void draw();	//called every cycle

		//======================================================
		//OPTIONAL
		//======================================================
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		void tuioAdded(ofxTuioObject& tuioObject);
		void tuioRemoved(ofxTuioObject& tuioObject);
		void tuioUpdated(ofxTuioObject& tuioObject);
		
};
