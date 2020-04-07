#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <serial/serial.h>
#include <serial_comm/quats.h>
#include "MadgwickAHRS.hpp"
#include <thread>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
//#include <std_msgs/String.h>
//#include <serial_comm/um7imu.h>


tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
/**
 * FUNCTIONS DECLARATION
 */
std::vector<std::string> split(const std::string& s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}
void getQuaternion(int _portID);
void getAngle();
void initCallback(std_msgs::String msg);

/**
 * GLOBAL VARIABLES
 */
serial::Serial port_shoulder("/dev/port_shoulder",115200);
serial::Serial port_elbow("/dev/port_elbow",115200);
serial::Serial port_wrist("/dev/port_wrist",115200);

madgwickahrs::MadgwickAHRS madgwick_wrist(0.02, 0.04);
madgwickahrs::MadgwickAHRS madgwick_shoulder(0.02, 0.04);	

double Q1_q0, Q1_q1, Q1_q2, Q1_q3;	// Quaternion from wrist_IMU
double Q2_q0, Q2_q1, Q2_q2, Q2_q3;	// Quaternion from shoulder_IMU

float mx1, my1, mz1, gx1, gy1, gz1, ax1, ay1, az1;	// raw data values from wrist_IMU
float mx2, my2, mz2, gx2, gy2, gz2, ax2, ay2, az2;	// raw data values from shoulder_IMU

float elbow_angle_in_radians;

int closeRobotiQ = 0;

std::vector<float> last_angles;

geometry_msgs::Quaternion wrist_quat;
geometry_msgs::Quaternion shoulder_quat;
tf::Quaternion q_shoulder_init(0, 0, 0, 1);
tf::Quaternion q_wrist_init(0, 0, 0, 1);
tf::Quaternion q_shoulder_pre(0, 0, 0, 1);
tf::Quaternion q_wrist_pre(0, 0, 0, 1);

tf::Quaternion wrist_norm();

/**
 * THREAD ROUTINES
 */
void get_angle(){
	while(ros::ok()){
		getAngle();
	}
	port_elbow.close();
}

void get_Quaternion(int i){
	while(ros::ok()){
		getQuaternion(i);
	}
	if(i == 1){
		port_wrist.close();
	}else{
		port_shoulder.close();
	}
}

/*
 * MAIN
 */
int main(int argc, char **argv)
{

	ros::init(argc,argv,"serial_comm");
	ros::NodeHandle nh;

	ros::Rate loop_rate(50);

	ros::Publisher publishQuats = nh.advertise<serial_comm::quats>("/quats",1);
	ros::Subscriber sub = nh.subscribe("/initialize", 1, initCallback);

	serial_comm::quats quats;

	std::thread elbow_thread (get_angle);
	std::thread wrist_thread (get_Quaternion, 1);
	std::thread shoulder_thread (get_Quaternion, 2);

	while(ros::ok()){

		quats.elbow_angle_in_rad = elbow_angle_in_radians;

		madgwick_wrist.Update(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1);

		madgwick_shoulder.Update(gx2, gy2, gz2, ax2, ay2, az2, mx2, my2, mz2);

		// q_shoulder = new Quaternion(Hardware.IMU2_q1, Hardware.IMU2_q2, Hardware.IMU2_q3, Hardware.IMU2_q0);
        	// q_shoulder = Quaternion.Inverse(q_shoulder_init) * q_shoulder * q_shoulder_pre;

		tf::Quaternion q_shoulder(-madgwick_shoulder.getQ1(), -madgwick_shoulder.getQ2(), madgwick_shoulder.getQ3(), madgwick_shoulder.getQ0());
		q_shoulder = q_shoulder_init.inverse() * q_shoulder * q_shoulder_pre;

		tf::Quaternion q_wrist(madgwick_wrist.getQ1(), -madgwick_wrist.getQ2(), -madgwick_wrist.getQ3(), madgwick_wrist.getQ0());
		q_wrist = q_wrist_init.inverse() * q_wrist * q_wrist_pre;

		// set wrist quaternion message
		wrist_quat.w = q_wrist.w();
		wrist_quat.x = q_wrist.x();
		wrist_quat.y = q_wrist.y();
		wrist_quat.z = q_wrist.z();

		shoulder_quat.w = q_shoulder.w();
		shoulder_quat.x = q_shoulder.x();
		shoulder_quat.y = q_shoulder.y();
		shoulder_quat.z = q_shoulder.z();

		// write wrist_quat to quats
		quats.wrist_quat = wrist_quat;
		quats.shoulder_quat = shoulder_quat;

		publishQuats.publish(quats);

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}

void getQuaternion(int _portID)
{
	std::string _quatTxt = "";
	std::string _gyroTxt = "";
	std::string _acceTxt = "";
	std::string _magnTxt = "";
	std::string _quatTxtBig = "";

	if (_portID == 1)
	{
		int _countSignals1 = 0;
		while (true)
		{
			if(port_wrist.waitReadable())
			{
				std::string buffer;
				while(true){
					buffer = "";
					port_wrist.read(buffer,1);
					_quatTxtBig += buffer;
					if(buffer == "\n"){
						break;
					}
				}
				_countSignals1 += 1;
			}

			if (_countSignals1==4) {
				ROS_INFO_STREAM("wrist: " + _quatTxtBig);
				std::vector<std::string> _splitTxtBig1 = split(_quatTxtBig,'\n');
				for(int i = 0; i < _splitTxtBig1.size(); i++)
				{
					if(_splitTxtBig1[i].empty()){
						return;	
					}
					std::vector<std::string> tempStr = split(_splitTxtBig1[i],',');
					if (tempStr.size() != 7) return;
					if (tempStr[6].length() != 4) return; // check sum must be 3 char (*XX)
					if (tempStr[6][0] != '*') return; // check sum must start with '*' (*XX)
					if (tempStr[0].compare ("$PCHRS") == 0) {
						if (tempStr[1].compare ("0") == 0) {
							_gyroTxt = _splitTxtBig1[i];
						}
						else if (tempStr[1].compare ("1") == 0) {
							_acceTxt = _splitTxtBig1[i];
						}
						else if (tempStr[1].compare ("2") == 0) {
							_magnTxt = _splitTxtBig1[i];
						}
					}
					else if(tempStr[0].compare("$PCHRQ")==0){
						_quatTxt = _splitTxtBig1[i];
					}
					else return;
				}
				break;
			}
		}
	}
	else if (_portID == 2)
	{
		int _countSignals2 = 0;
		while (true)
		{
			if(port_shoulder.waitReadable())
			{
				std::string buffer;
				while(true){

					buffer = "";
					port_shoulder.read(buffer,1);

					_quatTxtBig += buffer;
					if(buffer == "\n"){
						break;
					}
				}
				_countSignals2 += 1;
			}

			if (_countSignals2==4) {
				ROS_INFO_STREAM("shoulder: "+_quatTxtBig); //for debugging
				std::vector<std::string> _splitTxtBig1 = split(_quatTxtBig,'\n');
				for(int i = 0; i < _splitTxtBig1.size(); i++)
				{
					if(_splitTxtBig1[i].empty()){
						return;	
					}
					std::vector<std::string> tempStr = split(_splitTxtBig1[i],',');
					if (tempStr.size() != 7) return;
					if (tempStr[6].length() != 4) return; // check sum must be 3 char (*XX)
					if (tempStr[6][0] != '*') return; // check sum must start with '*' (*XX)
					if (tempStr[0].compare ("$PCHRS") == 0) {
						if (tempStr[1].compare ("0") == 0) {
							_gyroTxt = _splitTxtBig1[i];
						}
						else if (tempStr[1].compare ("1") == 0) {
							_acceTxt = _splitTxtBig1[i];
						}
						else if (tempStr[1].compare ("2") == 0) {
							_magnTxt = _splitTxtBig1[i];
						}
					}
					else if(tempStr[0].compare("$PCHRQ")==0){
						_quatTxt = _splitTxtBig1[i];
					}
					else{ return;}
				}
				break;
			}
		}
	}

	if (_quatTxt.length() < 50 && _quatTxt.length() > 0) return;
	if (_gyroTxt.length() < 35 && _gyroTxt.length() > 0) return;
	if (_acceTxt.length() < 41 && _acceTxt.length() > 0) return;
	if (_magnTxt.length() < 41 && _magnTxt.length() > 0) return;

	std::string _quatChars;
	std::string _gyroChars;
	std::string _acceChars;
	std::string _magnChars;

	_quatChars = _quatTxt; // convert to chars
	_gyroChars = _gyroTxt;
	_acceChars = _acceTxt;
	_magnChars = _magnTxt;

	char _checksumCalculated;
	char _checksumCalculatedG;
	char _checksumCalculatedA;
	char _checksumCalculatedM;

	_checksumCalculated = _quatChars[1];
	_checksumCalculatedG = _gyroChars[1];
	_checksumCalculatedA = _acceChars[1];
	_checksumCalculatedM = _magnChars[1];

	// calculate check sum for quaternion line
	for (int i = 2; i < _quatChars.length(); i++)
	{
		if (_quatChars[i] == '*') break;
		_checksumCalculated ^= _quatChars[i];
	}
	// calculate check sum for gyro line
	for (int i = 2; i < _gyroChars.length(); i++)
	{
		if (_gyroChars[i] == '*') break;
		_checksumCalculatedG ^= _gyroChars[i];
	}
	// calculate check sum for accel line
	for (int i = 2; i < _acceChars.length(); i++)
	{
		if (_acceChars[i] == '*') break;
		_checksumCalculatedA ^= _acceChars[i];
	}
	// calculate check sum for magn line
	for (int i = 2; i < _magnChars.length(); i++)
	{
		if (_magnChars[i] == '*') break;
		_checksumCalculatedM ^= _magnChars[i];
	}

	// split
	std::vector<std::string> _splitTxt = split(_quatTxt,',');
	std::vector<std::string> _splitTxtG = split(_gyroTxt,',');
	std::vector<std::string> _splitTxtA = split(_acceTxt,',');
	std::vector<std::string> _splitTxtM = split(_magnTxt,',');

	if(_splitTxt.size() != 7) return;
	if(_splitTxtG.size() != 7) return;
	if(_splitTxtA.size() != 7) return;
	if(_splitTxtM.size() != 7) return;

	//QUATERNION
	if (_splitTxt[0].compare("$PCHRQ") == 0)
	{
		std::string _receivedChecksum = _splitTxt[6].substr(1,2);
		std::istringstream hex_chars_stream(_receivedChecksum);

		unsigned int c;	// c is integer equivalent of _receivedChecksum
		hex_chars_stream >> std::hex >> c;

		if (_checksumCalculated == c)
		{
			// get quaternions
			if (_portID == 1)
			{
				Q1_q0 = std::stod(_splitTxt[2]);
				Q1_q1 = std::stod(_splitTxt[3]);
				Q1_q2 = std::stod(_splitTxt[4]);
				Q1_q3 = std::stod(_splitTxt[5]);
			}
			else if(_portID == 2)
			{
				Q2_q0 = std::stod(_splitTxt[2]);
				Q2_q1 = std::stod(_splitTxt[3]);
				Q2_q2 = std::stod(_splitTxt[4]);
				Q2_q3 = std::stod(_splitTxt[5]);
			}
			_quatTxt = "";
		}
	}
	//GYRO
	if (_splitTxtG[0].compare("$PCHRS") == 0)
	{
		std::string _receivedChecksum = _splitTxtG[6].substr(1,2);
		std::istringstream hex_chars_stream(_receivedChecksum);

		unsigned int c;	// c is integer equivalent of _receivedChecksum
		hex_chars_stream >> std::hex >> c;

		if (_checksumCalculatedG == c)
		{
			float convUnitToRads = 3.1415/180;
			if (_portID == 1)
			{
				gx1 = std::stof(_splitTxtG[3])*convUnitToRads;
				gy1 = std::stof(_splitTxtG[4])*convUnitToRads;
				gz1 = std::stof(_splitTxtG[5])*convUnitToRads;
			}
			else if(_portID == 2)
			{
				gx2 = std::stof(_splitTxtG[3])*convUnitToRads;
				gy2 = std::stof(_splitTxtG[4])*convUnitToRads;
				gz2 = std::stof(_splitTxtG[5])*convUnitToRads;
			}
			_gyroTxt = "";
		}
	}
	//ACCEL
	if (_splitTxtA[0].compare("$PCHRS") == 0)
	{
		std::string _receivedChecksum = _splitTxtA[6].substr(1,2);
		std::istringstream hex_chars_stream(_receivedChecksum);

		unsigned int c;	// c is integer equivalent of _receivedChecksum
		hex_chars_stream >> std::hex >> c;

		float convGtoAcc = 9.81;
		if (_checksumCalculatedA == c)
		{
			if (_portID == 1)
			{
				ax1 = convGtoAcc*std::stof(_splitTxtA[3]);
				ay1 = convGtoAcc*std::stof(_splitTxtA[4]);
				az1 = convGtoAcc*std::stof(_splitTxtA[5]);
			}
			else if(_portID == 2)
			{
				ax2 = convGtoAcc*std::stof(_splitTxtA[3]);
				ay2 = convGtoAcc*std::stof(_splitTxtA[4]);
				az2 = convGtoAcc*std::stof(_splitTxtA[5]);
			}
			_acceTxt = "";
		}
	}
	//MAGN
	if (_splitTxtM[0].compare("$PCHRS") == 0)
	{
		std::string _receivedChecksum = _splitTxtM[6].substr(1,2);
		std::istringstream hex_chars_stream(_receivedChecksum);

		unsigned int c;	// c is integer equivalent of _receivedChecksum
		hex_chars_stream >> std::hex >> c;

		if (_checksumCalculatedM == c)
		{
			if (_portID == 1)
			{
				mx1 = std::stof(_splitTxtM[3]);
				my1 = std::stof(_splitTxtM[4]);
				mz1 = std::stof(_splitTxtM[5]);
			}
			else if(_portID == 2)
			{
				mx2 = std::stof(_splitTxtM[3]);
				my2 = std::stof(_splitTxtM[4]);
				mz2 = std::stof(_splitTxtM[5]);
			}
			_magnTxt = "";
		}
	}
}

float filterAngle(float current_angle)
{
	// filter parameters
	int N = 10; // number of average elements
	float alpha = 0.2; // smoothing weight factor
	float epsilon = 200.0; // threshold for "odd" values

	float filtered_angle = current_angle;



	last_angles.push_back(current_angle);

	if (last_angles.size() == N+1) // ten moving average
	{

		float last_angle_avg = 0.0;

		for (int i = 0; i < last_angles.size()-1; i++)
		{
			last_angle_avg += last_angles[i];
		}
		last_angle_avg = last_angle_avg / N;

		// remove odd values
		if((std::abs(current_angle) - last_angle_avg) > epsilon)
		{
			return last_angle_avg;
		}

		// apply low-pass filter
		filtered_angle = alpha * last_angles[N - 1] + (1.0 - alpha) * last_angles[N - 2];

		last_angles.erase(last_angles.begin()); // remove first element

	}

	return filtered_angle;
}

void getAngle()
{
	//region prepare for the parser
	std::string _quatTxt = "";
	std::string _quatTxtBig = "";

	if(port_elbow.waitReadable()){
		std::string buffer;
		while(true){
			buffer = "";
			port_elbow.read(buffer,1);
			_quatTxt += buffer;
			if(buffer == "\n"){
				break;
			}
		}
	}
	if (_quatTxt.length() < 54) return;

	std::size_t found = _quatTxt.find("3C-71-AD-5B");
	if(found!=std::string::npos)
	{
		//ROS_INFO_STREAM(_quatTxt); //for debugging
		_quatTxt = _quatTxt.substr(found,54);
	}
	std::vector<std::string> _splitTxtTrash = split(_quatTxt,' ');
	std::vector<std::string> _splitTxt;
	for(int i = 0; i < _splitTxtTrash.size(); i++)
	{
		if(!_splitTxtTrash[i].empty()){
			_splitTxt.push_back(_splitTxtTrash[i]);
		}
	}

	//region get elbow angle
	// check validity of the received number: must be for digit
	float temp_angle;

	temp_angle = std::stof(_splitTxt[2]);
	//ROS_INFO_STREAM("Before filter: " << temp_angle);
	temp_angle = filterAngle(temp_angle);
	//ROS_INFO_STREAM("After filter: " << temp_angle);

	float scale_on_elbow_angle = (3.1415/2) / (1670.0 - 500.0);
	elbow_angle_in_radians = (-500.0 + temp_angle) * scale_on_elbow_angle;
	//endregion

	//region get soft status for RobotiQ
	if (_splitTxt[3].length() <= 4) closeRobotiQ = 0;

	int temp_pressure_level = std::stoi(_splitTxt[3]);

	if (temp_pressure_level > 1000)
	{
		closeRobotiQ = 1;
	}
	else{
		closeRobotiQ = 0;
	}
	//endregion
}

void initCallback(std_msgs::String msg){
	if(msg.data == "init"){
		q_shoulder_init.setW(madgwick_shoulder.getQ0());
		q_shoulder_init.setX(madgwick_shoulder.getQ1());
		q_shoulder_init.setY(madgwick_shoulder.getQ2());
		q_shoulder_init.setZ(madgwick_shoulder.getQ3());

		q_wrist_init.setW(madgwick_wrist.getQ0());
		q_wrist_init.setX(madgwick_wrist.getQ1());
		q_wrist_init.setY(madgwick_wrist.getQ2());
		q_wrist_init.setZ(madgwick_wrist.getQ3());
	}
}
