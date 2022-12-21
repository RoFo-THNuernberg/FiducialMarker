#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	
	ofSetWindowShape(PICTURE_WIDTH,PICTURE_HEIGHT);
	ofBackground(0);
	ofSetFrameRate(60);

	
	//openFramworks TUIO-Client setup
	//======================================================
	tuio.setup(new ofxTuioUdpReceiver(3333));

	ofAddListener(tuio.AddTuioObject, this, &ofApp::tuioAdded);
	ofAddListener(tuio.UpdateTuioObject, this, &ofApp::tuioUpdated);
	ofAddListener(tuio.RemoveTuioObject, this, &ofApp::tuioRemoved);

	tuio.connect(false);
	//======================================================


	//Ros create publisher
	//======================================================
	pub_ = n_.advertise<std_msgs::String>(pub_sub_name, 1000);
	//======================================================
	
}

//--------------------------------------------------------------
void ofApp::update() {
	
	//Set Windowtitle
	//======================================================
	s << ofGetFrameRate();
	ofSetWindowTitle(s.str());
	s.str("");
	//======================================================

	//Create and publish ROS messages
	//======================================================
	std_msgs::String msg;
	std::stringstream ss;
	for(int i = 0;i<objects.size();i++)
	{
		if(objects[i].objectID == 4)
			{
			ss << objects[i].objectID << "#" <<objects[i].xReal << "," << objects[i].yReal << "," << (360-(objects[i].angle -90))%360  << endl;
			msg.data = ss.str();

			ROS_INFO("%s", msg.data.c_str());
			pub_.publish(msg);
			ros::spinOnce();
			}
	}

	//======================================================


}

//--------------------------------------------------------------
void ofApp::draw(){


	ofNoFill(); //shapes with no fill

	//Draw Field 
	//======================================================
	if(setField)
	{
		ofSetColor(255);
		ofDrawRectangle(xField,yField,widthField,-heightField);
		ofDrawLine(xField,(yField-heightField -30 ),xField, yField);
		ofDrawLine(xField,yField,xField+widthField + 30,yField);
		
	}
	//======================================================

	//Draw Robots
	//======================================================
	for (int i = 0; i < objects.size(); i++)
	{
		
		// draw Robots
		ofSetColor(0,0,255);
		ofDrawCircle(objects[i].pos.x, objects[i].pos.y , ROBOT_RADIUS);
		string id = ofToString(objects[i].objectID);
		ofDrawBitmapString(id, objects[i].pos.x,objects[i].pos.y);

		ofPushMatrix();
		ofTranslate(objects[i].pos.x, objects[i].pos.y );
		ofRotateZDeg(objects[i].angle);
		ofDrawLine(0,-(ROBOT_RADIUS-5),0,-(ROBOT_RADIUS+5));
		ofPopMatrix();
		
		//getReal Coordinates
		if(setField)
		{	
		objects[i].xReal = (((objects[i].pos.x - xField)/widthField)*widthValReal);
		objects[i].yReal = (((yField-objects[i].pos.y)/heightField)*heightValReal);
		}
	}
	//======================================================
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

	//Determine Field if F1 is pressed
	//======================================================	
		if(key == OF_KEY_F1)
		{
			setField = false;

			std:cout << "Enter Width in CM: ";
			std::cin >> widthValReal ;
			std::cout << "Enter Height in CM: ";
			std::cin >> heightValReal;

			if(objects.size() == 2)
			{
				if(objects[0].pos.x < objects[1].pos.x)
				{
					xField = objects[0].pos.x;
					widthField = objects[1].pos.x-objects[0].pos.x;
				}
				else{
					xField = objects[1].pos.x;
					widthField = objects[0].pos.x- objects[1].pos.x;
				}

				if(objects[0].pos.y < objects[1].pos.y)
				{
					yField = objects[1].pos.y;
					heightField = objects[1].pos.y - objects[0].pos.y;
				}
				else{
					yField = objects[0].pos.y;
					heightField = objects[0].pos.y - objects[1].pos.y;
				}
			}
			setField = true;
			
			//std::cout << "xField = " << xField << " yField = " << yField << " widthField = " << widthField << " heightField = " << heightField << std::endl;
		}
	//======================================================
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

	//Console Out of actual Positions of Robots 
	//======================================================
		for (int i = 0; i < objects.size(); i++)
		{
		//getReal Coordinates
			if(setField)
			{	
			std::cout << "ID " << objects[i].objectID << "// x (pixel):" << objects[i].pos.x << " x (cm):" << objects[i].xReal <<" y (pixel):" << objects[i].pos.y << " y (cm):" << objects[i].yReal<< " Angle (Deg): " <<(360-(objects[i].angle -90))%360 <<endl;
			}	
		}
	//======================================================
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::tuioAdded(ofxTuioObject& tuioObject)
{
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();
	float x = tuioObject.getScreenX(ofGetWidth());
	float y = tuioObject.getScreenY(ofGetHeight());
	long angle  = tuioObject.getAngleDegrees();

	object o;
	o.sessionID = id;
	o.objectID = objectID;
	o.pos.x = x;
	o.pos.y = y;
	o.angle = angle;
	objects.push_back(o);
	//cout << "add " << objectID << " at Session " << id << "    xPos =" << o.pos.x << "    yPos =" << o.pos.y << "    angle=" << o.angle  << endl;
}

void ofApp::tuioUpdated(ofxTuioObject& tuioObject)
{
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();
	float x = tuioObject.getScreenX(ofGetWidth());
	float y = tuioObject.getScreenY(ofGetHeight());
	long angle = tuioObject.getAngleDegrees();

	vector<object> o = objects;
	for (int i = 0; i < o.size(); i++)
	{
		if (o[i].sessionID != id) continue;
		o[i].pos.x = x;
		o[i].pos.y = y;
		o[i].angle = angle;

	}

	/*for (int i = 0; i < o.size(); i++)
	{
		cout << "update " << objectID << " at Session " << id << "    xPos =" << o[i].pos.x << "    yPos =" << o[i].pos.y << "    angle=" << o[i].angle << endl;
	
	}*/

	objects = o;
}

void ofApp::tuioRemoved(ofxTuioObject& tuioObject)
{
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();
	//cout << "remove " << objectID << "at Session " << id << endl;

	vector<object> o = objects;
	for (int i = 0; i < o.size(); i++) {
		if (o[i].sessionID != id) continue;
		o.erase(o.begin() + i);
		break;
	}
	objects = o;
}
