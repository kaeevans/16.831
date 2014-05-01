#include "spheres.h"


unsigned int Spheres::initSpheres()
{
	int retVal;
	struct termios options;

	test_running_flag = false;
	last_global_test_time = 0;

	//open the serial port
	serial_fd = open(serial_port_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial_fd == -1)
	{
		printf("Open Serial Port Failed: %s\n", serial_port_path.c_str());
		return -1;
	}
	else
	{
		fcntl(serial_fd, F_SETFL, 0);
	}

	//get termios options
	tcgetattr(serial_fd, &options);

	//set up baud rates
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	//set up number of bits
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= (CS8 | CLOCAL | CREAD);

	//set up no parity checking
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	//disable hardware/software flow control
	options.c_cflag &= ~CRTSCTS;
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	//set canonical
//	options.c_lflag = ICANON;
//	options.c_lflag &= ~ICANON;		//old
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG); //new

	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);

	//set termios options
	tcflush(serial_fd, TCIFLUSH);
	tcsetattr(serial_fd, TCSANOW, &options);

	//initialize mutex
	retVal = pthread_mutex_init(&spheres_data_mutex, NULL);
	if (retVal !=0)
	{
		printf("pthread_mutex_init failed\n");
		return retVal;
	}

	pthread_mutex_init(&imuProcessingMutex, NULL);
//	pthread_cond_init(&imuProcessingCond, NULL);

	pthread_mutex_init(&forceTorqueProcessingMutex, NULL);
//	pthread_cond_init(&forceTorqueProcessingCond, NULL);

	pthread_mutex_init(&globMetProcessingMutex, NULL);
//	pthread_cond_init(&globMetProcessingCond, NULL);

	pthread_mutex_init(&mannum_mutex, NULL);

	pthread_mutex_init(&IRcmd_mutex, NULL);
	pthread_mutex_lock(&IRcmd_mutex);

	return 0;
}

unsigned int Spheres::closeSpheres()
{
	int retVal;

	test_running_flag = false;

	msg_header msg;
	msg_terminate_test msgTerm;
	msg.startByte = START_BYTE;
	msg.from = FROM_GOGGLES;
	msg.length = sizeof(msgTerm);
	msg.ID = TERMINATE_TEST;
	msgTerm.return_code = return_code;

	printf("Terminate Test: %d\n", return_code);
	retVal = write(serial_fd, (void *)&msg, sizeof(msg));
	if (retVal < sizeof(msg))
	{
		printf("Write failed\n");
		return -1;
	}
	retVal = write(serial_fd, (void *)&msgTerm, sizeof(msgTerm));
	if (retVal < sizeof(msgTerm))
	{
		printf("Write failed\n");
		return -1;
	}

	tcflush(serial_fd, TCIFLUSH);
	usleep(1000000);

	//close serial port
	close(serial_fd);

	//destroy mutex mutex
	retVal = pthread_mutex_destroy(&spheres_data_mutex);
	if (retVal !=0)
	{
		printf("pthread_mutex_destroy spheres_data_mutex failed!\n");
		return retVal;
	}

	printf("Closed spheres\n");

	return 0;
}


unsigned int Spheres::parseString(msg_header msgHeader, unsigned char* buffer) // the first
{
	int retVal;
	//locking mutex

	if ( msgHeader.ID == WATCHDOG_REQUEST) {
		//WATCHDOG MESSAGE
		pthread_mutex_lock(&spheres_data_mutex);
		clock_gettime(CLOCK_REALTIME, &latest_sys_time);
		pthread_mutex_unlock(&spheres_data_mutex);

		msg_header msg;
		msg.startByte = START_BYTE;
		msg.ID = WATCHDOG_REPLY;
		msg.from = FROM_GOGGLES;
		msg.length = 0;

		retVal = write(serial_fd, (void *)&msg, sizeof(msg));
		if (retVal < sizeof(msg))
		{
			printf("Write failed\n");
			return -1;
		}
		printf("Received Watchdog request and replied\n");
	} else if (msgHeader.ID == SYNC_TEST) {
		if (test_running_flag != 1) {
			test_running_flag = 1;
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &start_sys_time);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);
			printf("Received SYNC_TEST\n");
		}

	} else if (msgHeader.ID == IMU_DATA) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			double timestamp = timeDiff(&start_sys_time, &latest_sys_time);

			pthread_mutex_lock(&imuProcessingMutex);
			memcpy(&imuData, buffer, sizeof(msg_imu_data));
			timestampedIMU tIMU(timestamp, imuData);
			imuData_dq.push_front(tIMU);
			pthread_mutex_unlock(&imuProcessingMutex);

		}

	} else if (msgHeader.ID == ACT_FORCES_TORQUES) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			double timestamp = timeDiff(&start_sys_time, &latest_sys_time);

			pthread_mutex_lock(&forceTorqueProcessingMutex);
			memcpy(&forceTorqueData, buffer, sizeof(msg_body_forces_torques));
			timestampedForceTorque tFT(timestamp, forceTorqueData);
			forceTorqueData_dq.push_front(tFT);
			pthread_mutex_unlock(&forceTorqueProcessingMutex);

//			printf("Received IMU_DATA\n");

//			printf("Gyro: [%f, %f, %f]\n", imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]);
//			printf("Accel: [%f, %f, %f]\n", imuData.accel[0], imuData.accel[1], imuData.accel[2]);
		}

	} else if (msgHeader.ID == GLOBAL_MET) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			double timestamp = timeDiff(&start_sys_time, &latest_sys_time);

			pthread_mutex_lock(&globMetProcessingMutex);
			memcpy(&globMetData, buffer, sizeof(msg_state));
			timestampedState tState(timestamp, globMetData);
			globMetData_dq.push_front(tState);

/*
			memcpy(&globMetData, buffer, sizeof(msg_state));
			new_globMet_data = true;
			pthread_cond_signal(&globMetProcessingCond);
*/
			pthread_mutex_unlock(&globMetProcessingMutex);
//			printf("Received GLOBAL_MET\n");

//			printf("Pos: [%f, %f, %f]\n", globMetData.pos[0], globMetData.pos[1], globMetData.pos[2]);
//			printf("Vel: [%f, %f, %f]\n", globMetData.vel[0], globMetData.vel[1], globMetData.vel[2]);
//			printf("Quat: [%f, %f, %f, %f]\n", globMetData.quat[0], globMetData.quat[1], globMetData.quat[2], globMetData.quat[3]);
//			printf("AngVel: [%f, %f, %f]\n", globMetData.angvel[0], globMetData.angvel[1], globMetData.angvel[2]);

		}

	} else if (msgHeader.ID == MAN_NUM) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			pthread_mutex_lock(&mannum_mutex);
			maneuver_number = msgHeader.ID - MAN_NUM;
			pthread_mutex_unlock(&mannum_mutex);
		}

	} else if (msgHeader.ID == IR_CMD) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			// release the lock on the IR mutex
			pthread_mutex_unlock(&IRcmd_mutex);

			// hwo do I "get" the INSPECT thread and give it the IRcmd lock?
			// this shouldn't be sending a message--it should just tell INSPECT to take the lock
			// send message to INSPECT to call Acquire() on the ORF
			// honestly this should be in here at all--INSPECT should talk directly to SPHERES
			
			
			// get the lock on the IR mutex
			pthread_mutex_lock(&IRcmd_mutex);
		}

	} else {
		//NO MESSAGE DETECTED
		std::cout << "No matched message\n";
	}

	//unlock mutex

	return 0;
}

unsigned int Inspect::parseString(msg_header msgHeader)
{
	if (msgHeader.ID == ENABLE_ORF) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			
		}

	} else if (msgHeader.ID == DISABLE_ORF) {
		if (test_running_flag == 1) {
			pthread_mutex_lock(&spheres_data_mutex);
			clock_gettime(CLOCK_REALTIME, &latest_sys_time);
			pthread_mutex_unlock(&spheres_data_mutex);

			
		}

	} else {
		//NO MESSAGE DETECTED
		std::cout << "No matched message\n";
	}

	//unlock mutex

	return 0;
}

unsigned int Spheres::waitGspInitTest()
{
	int count = 0;

	while(test_running_flag != 1)
	{
		if (count < maxWaitTimeToStartTest)
		{
			usleep(1000);
			count++;
			if (count % 200 == 0) {
				printf("Waiting for init test... Count: %d\n", count);
			}
		}
		else
		{
			printf("Timeout waiting for gspInitTest()\n");
			return 1;
		}
	}
	return 0;
}

//return 1 if it is time to terminate test
bool Spheres::checkTestTerminate()
{
	struct timespec current_time;

	if (test_running_flag == 1)
	{
		clock_gettime(CLOCK_REALTIME, &current_time);

		pthread_mutex_lock(&spheres_data_mutex);
		//10 second watchdog on spheres
		if ((current_time.tv_sec - latest_sys_time.tv_sec) > 10)
		{
			pthread_mutex_unlock(&spheres_data_mutex);
			printf("Watchdog expired: No data received from spheres for 10 seconds. Terminating test. \n");
			return true; // terminate test
		}
		pthread_mutex_unlock(&spheres_data_mutex);

		return false; // test is still running
	}

	return true; // terminate test, because test_running_flag = 0
}


int Spheres::sendCtrlBodyFrame(float forces[3], float torques[3]) {
	int retVal;
	msg_header msg;
	msg_body_forces_torques msgData;

	msg.startByte = START_BYTE;
	msg.from = FROM_GOGGLES;
	msg.length = sizeof(msgData);
	msg.ID = FORCES_TORQUES;

	msgData.forces[0] = forces[0];
	msgData.forces[1] = forces[1];
	msgData.forces[2] = forces[2];
	msgData.torques[0] = torques[0];
	msgData.torques[1] = torques[1];
	msgData.torques[2] = torques[2];

	printf("Sending Control Forces and Torques\n");
	retVal = write(serial_fd, (void *)&msg, sizeof(msg));
	if (retVal < sizeof(msg))
	{
		printf("Write failed\n");
		return -1;
	}

	retVal = write(serial_fd, (void *)&msgData, sizeof(msgData));
	if (retVal < sizeof(msgData))
	{
		printf("Write failed\n");
		return -1;
	}


}

int Spheres::sendUserMessage(unsigned char user_message_id, void * msgData, unsigned char msgLength) {
	int retVal;
	msg_header msg;

	msg.startByte = START_BYTE;
	msg.from = FROM_GOGGLES;
	msg.length = msgLength;
	msg.ID = user_message_id;

	//printf("Sending User Message: %X\n", user_message_id);
	retVal = write(serial_fd, (void *)&msg, sizeof(msg));
	if (retVal < sizeof(msg))
	{
		printf("Write failed\n");
		return -1;
	}

	if (msgData != NULL) {
		retVal = write(serial_fd, msgData, msgLength);
		if (retVal < sizeof(msgData))
		{
			printf("Write failed\n");
			return -1;
		}
	} else if (msgLength != 0) {
		printf("No data provided\n");
		return -1;
	}

}

int Spheres::changeManeuverNumber(unsigned char _mannum, bool sendToSPHERES) {
	pthread_mutex_lock(&mannum_mutex);
	maneuver_number = _mannum;
	pthread_mutex_unlock(&mannum_mutex);

	if (sendToSPHERES && useSpheres) {
		sendUserMessage(MAN_NUM + _mannum, NULL, 0);
		std::cout << "Send change maneuver number: " << _mannum << std::endl;
	}
}

unsigned int Spheres::getManeuverNumber() {
	unsigned int _mannum;

	pthread_mutex_lock(&mannum_mutex);
	_mannum = maneuver_number;
	pthread_mutex_unlock(&mannum_mutex);

	return _mannum;
}

int Spheres::toggleMetrol(unsigned char _IRcmd, bool sendToSPHERES) {
	if (sendToSPHERES && useSpheres) {
		sendUserMessage(IR_CMD + _IRcmd, NULL, 0);
		std::cout << "Toggle metrology data acquisition" << _IRcmd << std::endl;
	}
}

int Inspect::enableORF(bool sendToINSPECT) {
	if (sendToINSPECT && useInspect) {
		sendUserMessage(IR_CMD, NULL, 0);
		std::cout << "Enable ORF image acquisition" << std::endl;
	}
}

double Spheres::timeDiff(struct timespec *start, struct timespec *end) {
	double milliseconds;
	double tv_sec, tv_nsec;
	if ((end->tv_nsec-start->tv_nsec)<0) {
		tv_sec = end->tv_sec-start->tv_sec-1;
		tv_nsec = 1.0e9+end->tv_nsec-start->tv_nsec;
	} else {
		tv_sec = end->tv_sec-start->tv_sec;
		tv_nsec = end->tv_nsec-start->tv_nsec;
	}
	milliseconds = tv_sec*1.0e3 + (tv_nsec / 1.0e6);
	return milliseconds;
}

