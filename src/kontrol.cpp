#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include "RtMidi.h"
#include <sensor_msgs/Joy.h>

enum KONTROL_TYPE{KONTROL,KONTROL2};

static std::vector<char> nanoKontrolButtonMapping {23, 33, 24, 34, 25, 35, 26, 36, 27, 37, 28, 38, 29, 39, 30, 40, 31, 41,47, 45, 48, 49, 46, 44};
static std::vector<char> nanoKontrolAxisMapping {2,3,4,5,6,8,9,12,13, 14,15,16,17,18,19,20,21,22};
static std::vector<char> nanoKontrol2ButtonMapping {32, 33, 34, 35, 36, 37, 38, 39, 48, 49, 50, 51, 52, 53, 54, 55, 64, 65, 66, 67, 68, 69, 70, 71, 43, 44, 42, 41, 45,58, 59, 60, 61, 62, 46};
static std::vector<char> nanoKontrol2AxisMapping {0,1,2,3,4,5,6,7, 16,17,18,19,20,21,22,23};

class Kontrol
{
public:
	Kontrol(int portn, ros::NodeHandle *nh);
	~Kontrol();
	void getMessage();
private:
	int slider_range; // 1 sets sliders to [-1,1] and 0 sets them to [0,1]

	RtMidiIn *midiin = 0;
	uint port;
	KONTROL_TYPE type;
	std::map<uint,float *> axis_map;
	std::map<uint,int *> button_map;
	sensor_msgs::Joy joy_msg;
	ros::Publisher pub;

	void printPortInfo();	
	int findKontrolPort();
	void setType();
	void bindMaps();
	void publishMsg(uint b1,uint b2);
};
Kontrol::Kontrol(int portn, ros::NodeHandle *nh) {
	midiin = new RtMidiIn();
	printPortInfo();
	if(portn < 0) {
		port = findKontrolPort();
	}
	else {
		ROS_INFO_STREAM("Using port " << portn);	
		port = portn;
	}
	setType();
	midiin->openPort(port);
	midiin->ignoreTypes( false, false, false );
	bindMaps();
	pub = nh->advertise<sensor_msgs::Joy>("nanokontrol",5,this);
	ros::param::param<int>("slider_range", slider_range, 0);
}
void Kontrol::printPortInfo() {
	unsigned int nPorts = midiin->getPortCount();
	ROS_INFO_STREAM( "There are " << nPorts << " MIDI devices.");
	std::string portName;
	for ( uint i=0; i < nPorts; i++ ) {
		portName = midiin->getPortName(i);
		ROS_INFO_STREAM(" Port " << i << ": " << portName);
	}
}
Kontrol::~Kontrol(){
	delete midiin;
}
int Kontrol::findKontrolPort() {
	unsigned int nPorts = midiin->getPortCount();
	std::string portName;
	for ( uint i=0; i < nPorts; i++ ) {
		portName = midiin->getPortName(i);
		if(portName.find("nanoKONTROL") != std::string::npos) {
			ROS_INFO_STREAM("Defaulting to port " << i);
			return i;
		}
	}
	ROS_ERROR_STREAM("Cannot find nanoKONTROL or nanoKONTROL2");
	return -1;
}
void Kontrol::setType(){
	std::string portName = midiin->getPortName(port);
	if(portName.find("nanoKONTROL2") != std::string::npos)
		type = KONTROL2;
	else
		type = KONTROL;
}
void Kontrol::getMessage() {
	std::vector<unsigned char> message;
	midiin->getMessage( &message );
	uint nBytes = message.size();
	if(nBytes == 3 && message[0] == 176)
		publishMsg(message[1],message[2]);
}
void Kontrol::bindMaps() {
	std::vector<char> axis_numbers, button_numbers;
	if(type == KONTROL) {
		axis_numbers = nanoKontrolAxisMapping;
		button_numbers = nanoKontrolButtonMapping;
	}
	else {
		axis_numbers = nanoKontrol2AxisMapping;
		button_numbers = nanoKontrol2ButtonMapping;
	}
	// make space
	joy_msg.buttons = std::vector<int> (button_numbers.size(),0);
	joy_msg.axes 	= std::vector<float> (axis_numbers.size(),0);

	// create maps to button data
	uint counter = 0;
	for(auto &id: axis_numbers) {
		axis_map[id] = joy_msg.axes.data() + counter;
		counter++;	
	}
	counter = 0;
	for(auto &id: button_numbers) {
		button_map[id] = joy_msg.buttons.data() + counter;
		counter++;	
	}

}
void Kontrol::publishMsg(uint b1,uint b2) {
	auto it1 = axis_map.find(b1);
	if(it1 != axis_map.end() && slider_range)
		*(it1->second) = (float(b2)  - 63.5)/ 63.5;
	if(it1 != axis_map.end() && !slider_range)
		*(it1->second) = float(b2) / 127.0;
	auto it2 = button_map.find(b1);
	if(it2 != button_map.end())
		*(it2->second) = b2 > 0;

	pub.publish(joy_msg);
}

int main(int argc, char *argv[])
{
	try{
		ros::init(argc,argv,"nanokontrol");
		ros::NodeHandle nh("~");
		Kontrol kontrol(-1, &nh);
		while(nh.ok()){
			ros::spinOnce();
			kontrol.getMessage();

		}
	}
	catch ( RtMidiError &error ) {
		error.printMessage();
		ROS_ERROR_STREAM("Shutting down");
	}

	return 0;
}
