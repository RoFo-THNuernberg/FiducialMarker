#include "ofApp.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"


//--------------------------------------------------------------
void ofApp::setup(){
	
	//======================================================
	//GENERAL
	//======================================================
	ofBackground(0);
	ofSetFrameRate(60);


	//======================================================
	//GUI
	//======================================================
	gui.setup();
	gui.setName("--Field--");
	gui.add(ofxIF_widthInput.setup("Width in CM:",0));
	gui.add(ofxIF_heightInput.setup("Height in CM:",0));
	gui.add(ofxB_initializeField.setup("Initialize Field"));
	gui.add(ofxTF_Status.setup("Status:","not Initialized"));
	gui.add(ofxTF_Error.setup("Error:",""));
	
	//======================================================
	//TUIO-CLIENT SETUP
	//======================================================
	tuio.setup(new ofxTuioUdpReceiver(3333));
	ofAddListener(tuio.AddTuioObject, this, &ofApp::tuioAdded);
	ofAddListener(tuio.UpdateTuioObject, this, &ofApp::tuioUpdated);
	ofAddListener(tuio.RemoveTuioObject, this, &ofApp::tuioRemoved);
	tuio.connect(false);
	

	//======================================================
	//ROS
	//======================================================
	pub_ = n_.advertise<turtlesim::Pose>(pub_sub_name, 1000);
	
}

//--------------------------------------------------------------
void ofApp::update() {
	
	//======================================================
	//GENERAL
	//======================================================
	framerate << ofGetFrameRate();
	ofSetWindowTitle(framerate.str());
	framerate.str("");

	//======================================================
	//INIT FIELD
	//======================================================
	if(ofxB_initializeField)
	{
		//read GUI input in cm
		widthValReal = ofxIF_widthInput;
		heightValReal = ofxIF_heightInput;

		setField = false;

			//determine field coordinates
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

				ofxTF_Status = "Initialized";
				ofxTF_Error = "";
				newInitialized = true;
				setField = true;
			}
			else
			{
				ofxTF_Error = "To many Markers";
			}
			

	//=============================TEST====================================
	//std::cout << "xField = " << xField << " yField = " << yField << " widthField = " << widthField << " heightField = " << heightField << std::endl;
	//=====================================================================
	}
	
	//======================================================
	//CREATE AND PUBLISH ROS MESSAGE
	//======================================================
	std_msgs::String msg;
	turtlesim::Pose pose_msg;

	std::stringstream ROS_Info_msgs;

	for(int i = 0;i<objects.size();i++)
	{
		if(objects[i].objectID == 4) //example for Marker Nr. 4, arbitrarily expandable
			{
			pose_msg.x = objects[i].xReal/100; //cm to m
			pose_msg.y =  objects[i].yReal/100; //cm to m
			pose_msg.theta = ((360-(objects[i].angle-90))%360) * (3.141/180.0); //convert reacTIVision angle to intervall [0;2*PI]

			pub_.publish(pose_msg);

			//=============================TEST====================================
			ROS_Info_msgs << objects[i].objectID << "#" <<objects[i].xReal << "," << objects[i].yReal << "," << ((360-(objects[i].angle-90))%360) * (3.141/180.0) << endl;
			msg.data = ROS_Info_msgs.str();
			ROS_INFO("%s", msg.data.c_str());
			//=====================================================================
	
			ros::spinOnce();
			}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){

	//======================================================
	//GUI
	//======================================================
	gui.draw();
	
	//======================================================
	//DRAW FIELD
	//======================================================
	if(setField)
	{
		if(newInitialized) 
		{
			s_width = ofToString(long(ofxIF_widthInput));
			s_height = ofToString(long(ofxIF_heightInput));
			newInitialized = false;
		}
		
		ofSetColor(255);
		//draw bounding box
		ofDrawRectangle(xField,yField,widthField,-heightField);
		//draw coordinate system
		ofDrawLine(xField,(yField-heightField -30 ),xField, yField);
		ofDrawLine(xField,yField,xField+widthField + 30,yField);
		//insert designations
		ofDrawBitmapString("X", xField + widthField + 40 ,yField);
		ofDrawBitmapString("Y", xField , yField-heightField - 40);
		ofDrawBitmapString(s_width + " cm", xField + (widthField/2),yField + 20);
		ofDrawBitmapString(s_height + " cm", xField - 55 ,yField-(heightField/2));

		
	}
	
	//======================================================
	//DRAW ROBOTS
	//======================================================
	ofNoFill(); 
	for (int i = 0; i < objects.size(); i++)
	{	
		//draw robots
		ofSetColor(0,0,255);
		ofDrawCircle(objects[i].pos.x, objects[i].pos.y , ROBOT_RADIUS);
		string s_id = ofToString(objects[i].objectID);
		//insert designations
		ofDrawBitmapString(s_id, objects[i].pos.x,objects[i].pos.y);
		//get the right rotation
		ofPushMatrix();
		ofTranslate(objects[i].pos.x, objects[i].pos.y );
		ofRotateZDeg(objects[i].angle);
		ofDrawLine(0,-(ROBOT_RADIUS-5),0,-(ROBOT_RADIUS+5));
		ofPopMatrix();
		
		//conversion of image coordinates to real coordinates
		if(setField)
		{	
		objects[i].xReal = (((objects[i].pos.x - xField)/widthField)*widthValReal);
		objects[i].yReal = (((yField-objects[i].pos.y)/heightField)*heightValReal);
		}
	}

	
}

void ofApp::tuioAdded(ofxTuioObject& tuioObject)
{
	//read data that has been added
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();
	float x = tuioObject.getScreenX(ofGetWidth());
	float y = tuioObject.getScreenY(ofGetHeight());
	long l_angle  = tuioObject.getAngleDegrees();

	//insert new object to vector
	object o;
	o.sessionID = id;
	o.objectID = objectID;
	o.pos.x = x;
	o.pos.y = y;
	o.angle = l_angle;
	objects.push_back(o);

	//=============================TEST====================================
	//cout << "add " << objectID << " at Session " << id << "    xPos =" << o.pos.x << "    yPos =" << o.pos.y << "    angle=" << o.angle  << endl;
	//=====================================================================
}

void ofApp::tuioUpdated(ofxTuioObject& tuioObject)
{
	//read data that has been updated
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();
	float x = tuioObject.getScreenX(ofGetWidth());
	float y = tuioObject.getScreenY(ofGetHeight());
	long l_angle = tuioObject.getAngleDegrees();


	//update object vector
	vector<object> o = objects;
	for (int i = 0; i < o.size(); i++)
	{
		if (o[i].sessionID != id) continue;
		o[i].pos.x = x;
		o[i].pos.y = y;
		o[i].angle = l_angle;
	}

	objects = o;

	//=============================TEST====================================
	//for (int i = 0; i < o.size(); i++)
	//{
	//	cout << "update " << objectID << " at Session " << id << "    xPos =" << o[i].pos.x << "    yPos =" << o[i].pos.y << "    angle=" << o[i].angle << endl;
	//}
	//=====================================================================
	
}

void ofApp::tuioRemoved(ofxTuioObject& tuioObject)
{
	//read ID that has been removed
	long objectID = tuioObject.getSymbolID();
	long id = tuioObject.getSessionID();

	//=============================TEST====================================
	//cout << "remove " << objectID << "at Session " << id << endl;
	//=====================================================================

	//remove object from vector
	vector<object> o = objects;
	for (int i = 0; i < o.size(); i++) {
		if (o[i].sessionID != id) continue;
		o.erase(o.begin() + i);
		break;
	}
	objects = o;
}


//=====================================================================
//OPTIONAL METHODS
//=====================================================================
//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

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
