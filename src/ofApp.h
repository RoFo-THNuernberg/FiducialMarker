#pragma once

#include "ofMain.h"
#include "ofxTuio.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#define PICTURE_WIDTH 1280
#define PICTURE_HEIGHT 720
#define OBJECT_SIZE 60


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

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

		struct object {
			ofVec2f pos;
			long sessionID;
			long objectID;
			long angle;
		};
		vector<object> objects;

		ofxTuioReceiver tuio;

		ofVideoGrabber cam;

		stringstream s;
		ofImage test;

		float xReal = 0.0,yReal = 0.0;
		int xField = 0, yField = 0, widthField = 0, heightField = 0;
		int widthValReal = 0, heightValReal = 0, tempMarkerID =0;
		bool setField = 0;

		//Ros
		ros::NodeHandle n_;
		ros::Publisher pub_;
		
};
