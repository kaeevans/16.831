#include "testproject.h"

	testproject::testproject () {
	}

	void testproject::GSsetup(){
		videostreaming.createNew_MatVideoBuffer(combinedVideoBuffer, "Combined Image");
		videostreaming.setAsDefaultVideoMode(combinedVideoBuffer);

		string imu_log_filename = this->datastorage.getGSdatastoragePath() + "imu_data.txt";
		imu_log_file.open(imu_log_filename.c_str());

		string forceTorque_log_filename = this->datastorage.getGSdatastoragePath() + "forceTorque_data.txt";
		forceTorque_log_file.open(forceTorque_log_filename.c_str());

		string globMet_log_filename = this->datastorage.getGSdatastoragePath() + "globMet_data.txt";
		globMet_log_file.open(globMet_log_filename.c_str());

		imgcount = 0;
		imgnum = 0;
	}

	void testproject::GSprocessImages(Mat& leftImage, Mat& rightImage) {
		Mat combined_img;
		cv::equalizeHist(leftImage, leftImage);
		cv::equalizeHist(rightImage, rightImage);
		if (this->videostreaming.videoStreamingOn) {


			Size size( leftImage.cols + rightImage.cols, leftImage.rows );

			//place two images side by side
			combined_img.create( size, CV_MAKETYPE(leftImage.depth(), 3) );
	        Mat imgLeft = combined_img( Rect(0, 0, leftImage.cols, leftImage.rows) );
	        Mat imgRight = combined_img( Rect(leftImage.cols, 0, rightImage.cols, rightImage.rows) );

	        cvtColor( leftImage, imgLeft, CV_GRAY2BGR );
			cvtColor( rightImage, imgRight, CV_GRAY2BGR );

			videostreaming.update_MatVideoBuffer(combinedVideoBuffer, combined_img);
		}

		imgnum++;

		if (imgnum % 5 == 0) {
			char imgname[250];
			sprintf(imgname, "%s/LeftRight%d.bmp", datastorage.getGSdatastoragePath().c_str(), imgcount++);
			cv::imwrite(imgname, combined_img);
		}



		pthread_mutex_lock(&keymutex);
		if (charKey == 0x1B) { // ESC
			printf("Quitting...\n");
			shutdownCriterion = true;
		}
		charKey = 0x00;
		pthread_mutex_unlock(&keymutex);
	}
	void testproject::GSprocessGuiKeyPress(unsigned char networkkey) {
		pthread_mutex_lock(&keymutex);
		charKey = networkkey;
		pthread_mutex_unlock(&keymutex);
	}
	void testproject::GSparseParameterFile(string line) {

	}

	void testproject::GSprocessIMU(double timestamp, msg_imu_data imu_data) {
		imu_log_file << timestamp << "," << imu_data.testTime << ","  << imu_data.gyro[0] << "," << imu_data.gyro[1] << "," << imu_data.gyro[2] << "," << imu_data.accel[0] << "," << imu_data.accel[1] << "," << imu_data.accel[2] << std::endl;
	}

	void testproject::GSprocessForceTorque(double timestamp, msg_body_forces_torques FT_data) {
		forceTorque_log_file << timestamp << "," << FT_data.testTime << ","  << FT_data.forces[0] << "," << FT_data.forces[1] << "," << FT_data.forces[2] << "," << FT_data.torques[0] << "," << FT_data.torques[1] << "," << FT_data.torques[2] << std::endl;
	}

	void testproject::GSprocessGlobalMetrology(double timestamp, msg_state gm_data) {
		globMet_log_file << timestamp << "," << gm_data.testTime << ","
												<< gm_data.pos[0] << "," << gm_data.pos[1] << "," << gm_data.pos[2] << ","
												<< gm_data.vel[0] << "," << gm_data.vel[1] << "," << gm_data.vel[2] << ","
												<< gm_data.quat[0] << "," << gm_data.quat[1] << "," << gm_data.quat[2] << "," << gm_data.quat[3] << ","
												<< gm_data.angvel[0] << "," << gm_data.angvel[1] << "," << gm_data.angvel[2] << std::endl;
	}

	void testproject::GScleanup() {
		imu_log_file.close();
		forceTorque_log_file.close();
		globMet_log_file.close();
	}

