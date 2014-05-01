#ifndef _GOGGLES_SPHERES_INTERFACE_H_
#define _GOGGLES_SPHERES_INTERFACE_H_


// C++ standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <sys/resource.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <deque>



//#define SERIAL_PORT	"/dev/ttyS0"		//ttyS0 for pico-itx board, ttyUSB0 for keyspan usb

#define VERBOSE_SPHERES	1	//0 means turn off printfs, 1 means turn on printfs

#define CTRL_MODE_INERTIAL	1
#define CTRL_MODE_BODY		2

#define STATE_LENGTH	13
#define POS_X		0
#define POS_Y		1
#define POS_Z		2
#define VEL_X		3
#define VEL_Y		4
#define VEL_Z		5
#define QUAT_1		6
#define QUAT_2		7
#define QUAT_3		8
#define QUAT_4		9
#define RATE_X		10
#define RATE_Y		11
#define RATE_Z		12

#define DEFAULT_GYRO_BIAS	2047.0f		//[counts] of ADC
#define DEFAULT_GYRO_SCALE	0.70000e-3f	//[rad/s /count]


typedef float state_vector[STATE_LENGTH];


#define START_BYTE 			0xA7
#define FROM_SPHERES		0x01
#define FROM_GOGGLES		0x02
#define WATCHDOG_REQUEST	0xB1
#define WATCHDOG_REPLY		0xB2
#define TERMINATE_TEST		0xC1
#define SYNC_TEST			0x91
#define FORCES_TORQUES		0xD1
#define IMU_DATA			0xE2
#define ACT_FORCES_TORQUES	0xE3

#define GLOBAL_MET			0xE4
#define BODY_STATE			0xE5
#define INERTIAL_STATE		0xE6
#define INERTIAL_MAT		0xE7
#define INERTIAL_POSE		0xE8
#define MAN_NUM				0xF0

//message types
typedef struct {
	unsigned char return_code;
} msg_terminate_test;

typedef struct {
	unsigned int testTime;	//in milliseconds
	float pos[3];
	float vel[3];
	float quat[4];
	float angvel[3];
} msg_state;

typedef struct {
	unsigned int testTime;	//in milliseconds
	float pos[3];
	float quat[4];
} msg_pose;

typedef struct {
	float J11;
	float J22;
	float J33;
	float J12;
	float J13;
	float J23;
} msg_inertial_matrix;

typedef struct {
	unsigned int testTime;
	float forces[3];
	float torques[3];
} msg_body_forces_torques;

typedef struct {
	unsigned int testTime;
	float gyro[3];
	float accel[3];
} msg_imu_data;

typedef struct {
	unsigned char startByte;
	unsigned char length;
	unsigned char from;
	unsigned char ID;
} msg_header;


class timestampedState {
	msg_state stateMsg;
	double timestamp;
public:
	timestampedState(double _timestamp, msg_state& state) {
		timestamp = _timestamp;
		stateMsg = state;
	}

/*	friend std::ostream& operator<<(std::ostream& out, const timestampedState& s) {
		out << s.timestamp  << "," << s.stateMsg.testTime
							<< ","  << s.stateMsg.pos[0] << "," << s.stateMsg.pos[1] << "," << s.stateMsg.pos[2] << ","
							<< s.stateMsg.vel[0] << "," << s.stateMsg.vel[1] << "," << s.stateMsg.vel[2] << ","
							<< s.stateMsg.quat[0] << "," << s.stateMsg.quat[1] << "," << s.stateMsg.quat[2] << "," << s.stateMsg.quat[3] << ","
							<< s.stateMsg.angvel[0] << "," << s.stateMsg.angvel[1] << "," << s.stateMsg.angvel[2] << std::endl;
		return out;
	}
*/

	msg_state getState() const {
		return stateMsg;
	}

	double getTimestamp() const {
		return timestamp;
	}
};

class timestampedForceTorque {
	msg_body_forces_torques force_torque;
	double timestamp;
public:
	timestampedForceTorque(double _timestamp, msg_body_forces_torques& ft) {
		timestamp = _timestamp;
		force_torque = ft;
	}

	msg_body_forces_torques getForceTorque() const {
		return force_torque;
	}

	double getTimestamp() const {
		return timestamp;
	}
};

class timestampedIMU {
	msg_imu_data imu;
	double timestamp;
public:
	timestampedIMU(double _timestamp, msg_imu_data& _imu) {
		timestamp = _timestamp;
		imu = _imu;
	}

	msg_imu_data getIMU() const {
		return imu;
	}

	double getTimestamp() const {
		return timestamp;
	}
};

using namespace std;

class Spheres {

	friend class gogglesGSP; // GSP can access and call private functions, but derived GS class cannot directly call private functions with SPHERES

	pthread_mutex_t spheres_data_mutex;
	pthread_mutex_t imuProcessingMutex;
	//pthread_cond_t imuProcessingCond;

	pthread_mutex_t forceTorqueProcessingMutex;
	//pthread_cond_t forceTorqueProcessingCond;

	pthread_mutex_t globMetProcessingMutex;
//	pthread_cond_t globMetProcessingCond;

	msg_imu_data imuData;
//	bool new_imu_data;
	msg_body_forces_torques forceTorqueData;
//	bool new_forceTorque_data;
	msg_state globMetData;
//	bool new_globMet_data;

	std::deque<timestampedState> globMetData_dq;
	std::deque<timestampedIMU> imuData_dq;
	std::deque<timestampedForceTorque> forceTorqueData_dq;

	double imu_timestamp;
	double forceTorque_timestamp;
//	double globMet_timestamp;

	bool test_running_flag;

	int serial_fd;

	pthread_mutex_t mannum_mutex;
	unsigned int maneuver_number;

	pthread_mutex_t IR_mutex;

	int test_time;
	int last_global_test_time;
	struct timespec latest_sys_time;
	struct timespec start_sys_time;
	int test_number;

public:

	bool useSpheres;
	string serial_port_path;

	int maxWaitTimeToStartTest;

	//// Definition of serial port header beginning
	unsigned char headerBeginningOfNewMessage[2];

	int spheres_ctrl_target_struct_size;
	int spheres_ctrl_forces_struct_size;
	int global_metrology_struct_size;
	int imu_data_struct_size;
	int test_info_struct_size;

	vector<int> messageSizes;

	unsigned char return_code;

	Spheres ()
	{
		useSpheres = true;

		test_running_flag = false; // 0 means test is not running, 1 means test is running

		maxWaitTimeToStartTest = 10000; // wait 10 seconds

		maneuver_number = 1;

		return_code = 253;
	}

private:

	unsigned int initSpheres();

	void spheresThread();

	unsigned int closeSpheres();

	unsigned int parseString(msg_header msgHeader, unsigned char* buffer);

	unsigned int waitGspInitTest();

	bool checkTestTerminate();

	double timeDiff(struct timespec *start, struct timespec *end);


public:

	int sendCtrlBodyFrame(float forces[3], float torques[3]);

	int sendUserMessage(unsigned char user_message_id, void * msgData, unsigned char msgLength);

	int changeManeuverNumber(unsigned char mannum, bool sendToSPHERES);

	unsigned int getManeuverNumber();

	int toggleMetrol(unsigned char mannum, bool sendToSPHERES);

};


#endif
