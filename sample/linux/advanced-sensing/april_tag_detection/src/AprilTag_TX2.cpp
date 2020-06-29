

#include"AprilTag_TX2.h"
#include <fstream>

inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t + PI, TWOPI) - PI;
	}
	else {
		t = fmod(t - PI, -TWOPI) + PI;
	}
	return t;
}

/**
* Convert rotation matrix to Euler angles
*/
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
	yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0)*c + wRo(1, 0)*s));
	roll = standardRad(atan2(wRo(0, 2)*s - wRo(1, 2)*c, -wRo(0, 1)*s + wRo(1, 1)*c));
}
bool IsSpace(char c)
{
	if (c == ' ' || c == '\t')
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool IsCommentChar(char c)
{
	if (c == COMMENT_CHAR)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Trim(string & str)
{
	if (str.empty())
	{
		return;
	}
	int i, start_pos, end_pos;
	for (i = 0; i < str.size(); i++)
	{
		if (!IsSpace(str[i]))
		{
			break;
		}
	}
	if (i == str.size())
	{
		str = "";
		return;
	}
	start_pos = i;

	for (i = str.size() - 1; i >= 0; i--)
	{
		if (!IsSpace(str[i]))
		{
			break;
		}
	}
	end_pos = i;
	str = str.substr(start_pos, end_pos - start_pos + 1);
}

bool AnalyseLine(const string & line, string & key, string & value)
{
	if (line.empty())
	{
		return false;
	}
	int start_pos = 0, end_pos = line.size() - 1, pos;
	if ((pos = line.find(COMMENT_CHAR)) != -1)
	{
		if (0 == pos)
		{
			return false;
		}
		end_pos = pos - 1; 
	}
	string new_line = line.substr(start_pos, end_pos - start_pos + 1); 
	if ((pos = new_line.find("=")) == -1) 
	{
		return false;
	}
	key = new_line.substr(0, pos);
	value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));
	Trim(key);
	if (key.empty())
	{
		return false;
	}
	Trim(value);
	return true;
}


void  AprilTag::String2map(const string& s, map<int, double> &m, const string& c)
{
	string::size_type pos1, pos2;
	vector<string> v;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
	for (vector<string>::size_type i = 0; i != v.size(); i += 2)
	{
		m[int(atoi(v[i].c_str()))] = double(atof(v[i + 1].c_str()));
		if (i == v.size() - 2)
			break;
	}
}

bool AprilTag::ReadConfig(const string & filename, map<string, string> & m)
{
	m.clear();
	ifstream infile(filename.c_str());
	if (!infile)
	{
		cout << "file open failed!" << endl;
		return false;
	}
	string line, key, value;
	while (getline(infile, line))
	{
		if (AnalyseLine(line, key, value))
		{
			m[key] = value;
		}
	}
	infile.close();
	return true;
}

void AprilTag::PrintConfig(const map<string, string> & m)
{
	map<string, string>::const_iterator mite;
	for (mite = m.begin(); mite != m.end(); mite++)
	{
		cout << mite->first << "=" << mite->second << endl;
	}
}

string AprilTag::FindInConfig(map<string, string>  m, string  key)
{
	map<string, string>::iterator it;
	it = m.find(key);
	if (it == m.end())
	{
		cout << "there is no " << key << endl;
		return NULL;
	}
	else
	{
		cout << it->second << endl;
	}
	return  it->second;
}

void AprilTag::setTagCodes(string s) {

	//if (s == "16h5") {
	if ( s.find("16h5") != std::string::npos )
	{
		m_tagCodes = AprilTags::tagCodes16h5;
	}
	else if ( s.find("25h7") != std::string::npos )
	{
		m_tagCodes = AprilTags::tagCodes25h7;
	}
	else if ( s.find("25h9") != std::string::npos )
	{
		m_tagCodes = AprilTags::tagCodes25h9;
	}
	else if ( s.find("36h9") != std::string::npos )
	{
		m_tagCodes = AprilTags::tagCodes36h9;
	}
	else if ( s.find("36h11") != std::string::npos )
	{
		m_tagCodes = AprilTags::tagCodes36h11;
	}
	else {
		cout << "Invalid tag family specified" << endl;
		exit(1);
	}
}

// parse command line options to change default behavior
void AprilTag::parseOptions(std::string config_path, int argc, char* argv[])
{
	//ReadConfig("landmark.cfg", LandMarkCfg);
	ReadConfig(config_path, LandMarkCfg);
	PrintConfig(LandMarkCfg);
	MarkSizeStr = FindInConfig(LandMarkCfg, "m_size");
	String2map(MarkSizeStr, m_size, ",");
	m_draw = int(atoi(LandMarkCfg.find("m_draw")->second.c_str()));
	m_timing = int(atoi(LandMarkCfg.find("m_timing")->second.c_str()));
	setTagCodes(LandMarkCfg.find("setTagCodes")->second);

	//intrinsic parameters
	c_width = int(atof(LandMarkCfg.find("c_width")->second.c_str()));
	c_height = int(atof(LandMarkCfg.find("c_height")->second.c_str()));
	m_width = int(atof(LandMarkCfg.find("m_width")->second.c_str()));
	m_height = int(atof(LandMarkCfg.find("m_height")->second.c_str()));
	m_py = double(atof(LandMarkCfg.find("m_py")->second.c_str()))*((double)m_height / c_height);
	m_px = double(atof(LandMarkCfg.find("m_px")->second.c_str()))*((double)m_width / c_width);
	m_fx = double(atof(LandMarkCfg.find("m_fx")->second.c_str()))*((double)m_width / c_width);
	m_fy = double(atof(LandMarkCfg.find("m_fy")->second.c_str()))*((double)m_height / c_height);//focal length

}

void AprilTag::setup()
{
	m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
	if (m_draw)
	{
		//cv::namedWindow(windowName, 1);
	}
}


void AprilTag::get_position(AprilTags::TagDetection& detection)
{
	map<int, double>::iterator p;
	p = m_size.find(detection.id);
	if (p != m_size.end())
	{
		landing_vision_defs::land_mark_pos land_mark_temp;
		//cout << "  Id: " << detection.id
			//<< " (Hamming: " << detection.hammingDistance << ")";
		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(p->second, m_fx, m_fy, m_px, m_py, translation, rotation);

		Eigen::Matrix3d F;
		F << 1, 0, 0,
			0, -1, 0,
			0, 0, 1;
		Eigen::Matrix3d fixed_rot = F*rotation;
		double yaw, pitch, roll;
		wRo_to_euler(fixed_rot, yaw, pitch, roll);
		land_mark_temp.mark_id = detection.id;
		land_mark_temp.hammingDistance = detection.hammingDistance;
		land_mark_temp.distance = translation.norm();
		land_mark_temp.x = translation(0);
		land_mark_temp.y = translation(1);
		land_mark_temp.z = translation(2);
		land_mark_temp.yaw = yaw;
		land_mark_temp.pitch = pitch;
		land_mark_temp.roll = roll;
		pos_vec.push_back(land_mark_temp);
	}
}

void AprilTag::processImage(cv::Mat& image, cv::Mat& image_gray)
{
	cv::cvtColor(image, image_gray, CV_BGR2GRAY);
	startTime = clock();
	detections = m_tagDetector->extractTags(image_gray);
	endTime = clock();
	// cout << "Every frame time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	// print out each detection
	pos_vec.clear();//If you need to keep pos_vec, write it under the if statement; 
	if (detections.size()>0)
		//cout << detections.size() << " tags detected:" << endl;
	for (int i = 0; i<detections.size(); i++) {
		get_position(detections[i]);
	}

	// show the current image including any detections
	if (m_draw) {
		for (int i = 0; i<detections.size(); i++) {
			// also highlight in the image
			detections[i].draw(image);
		}
		//cv::namedWindow(windowName, 1);
		//imshow(windowName, image); // OpenCV call
		//cv::waitKey(1);
	}
}

// Load and process a single image
int AprilTag::loadImages(cv::Mat& image)
{
  if ( be_processing == true )
  {
    return -1;
  }
  be_processing = true;
	if (image.size() != cv::Size( m_width,m_height))
	{
		cv::resize(image, image, cv::Size(m_width, m_height));
	}
	cv::Mat image_gray;
	processImage(image, image_gray);
	be_processing = false;
	{
		std::lock_guard<std::mutex> auto_lock(img_event_mu);
	}
	img_event_cv.notify_one();

	return 0;
}


// here is were everything begins
