#ifndef _TESTPROJECT_H_
#define _TESTPROJECT_H_

#include "gogglesGSP.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace cv;
class testproject : public gogglesGSP {
	MatVideoBuffer combinedVideoBuffer;
	unsigned char charKey;
	pthread_mutex_t keymutex;

	ofstream imu_log_file, forceTorque_log_file,globMet_log_file;
	int imgcount, imgnum;

public:

	testproject ();
	void GSsetup();
	void GSprocessImages(Mat& leftImage, Mat& rightImage);
	void GSparseParameterFile(string line);
	void GSprocessGuiKeyPress(unsigned char networkkey);
	void GSprocessIMU(double timestamp, msg_imu_data imu_data);
	void GSprocessForceTorque(double timestamp, msg_body_forces_torques FT_data);
	void GSprocessGlobalMetrology(double timestamp, msg_state gm_data);
	void GScleanup();

};

#endif
