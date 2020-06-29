#ifndef APRILTAG_TX2_H
#define APRILTAG_TX2_H
#include <iostream>
#include <cstring>
#include <vector>
#include <string>
#include <map>
#include <list>
#include <ctime>
#include <fcntl.h>
#include <errno.h> 
#include <cmath>
#include "opencv2/opencv.hpp"
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h" 

#include "landing_vision_defs.h"

#include <condition_variable>
#include <mutex>

using namespace std;
#define COMMENT_CHAR '#'

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
inline double standardRad(double t);
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
class AprilTag
{
public:
	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;
	const char* windowName = "apriltags_demo";

	clock_t startTime, endTime;
	//  bool m_arduino; // send tag detections to serial port?
	bool m_timing; // print timing information for each tag extraction call

	int c_width;
	int c_height;
	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;

	int m_deviceId; // camera id (in case of multiple cameras) 
	list<string> m_imgNames;
	cv::VideoCapture m_cap;
	

	int m_exposure;
	int m_gain;
	int m_brightness;

public:
	// default constructor
	AprilTag() :
		// default settings, most can be modified through command line options (see below)
		m_tagDetector(NULL),
		m_tagCodes(AprilTags::tagCodes36h11),
		m_draw(true),
		m_timing(false),
		c_width(1920),
		c_height(1080),
		m_width(640),
		m_height(360),
		m_tagSize(0.166),
		m_fx(1565),
		m_fy(1565),
		m_px(m_width / 2),
		m_py(m_height / 2),

		m_exposure(-1),
		m_gain(-1),
		m_brightness(-1),

		m_deviceId(0),

		be_processing(false)
	{}
	vector<landing_vision_defs::land_mark_pos> pos_vec;
	void parseOptions(std::string config_path = std::string("landmark.cfg"), int argc = 0, char* argv[] = nullptr);
	void setup();
	int loadImages(cv::Mat& image);
private:
	vector<AprilTags::TagDetection> detections;
	map<string, string> LandMarkCfg;
	map<int, double> m_size;
	string MarkSizeStr;
	void get_position(AprilTags::TagDetection& detection);
	void processImage(cv::Mat& image, cv::Mat& image_gray);
	void setTagCodes(string s);
	bool ReadConfig(const string & filename, map <string, string> & m);
	void PrintConfig(const map<string, string> & m);
	string FindInConfig(map<string, string> m, string key);
	void String2map(const string& s, map<int, double> &m, const string& c);

public:
  std::condition_variable april_process_cv;
  std::mutex              april_process_mt;

  bool be_processing;

  bool m_draw; // draw image and April tag detections?

  std::mutex 							img_event_mu;
  std::condition_variable img_event_cv;
};
#endif
