
/* 
 * gsp_UDP.h
 *
 * MIT Space Systems Laboratory
 * SPHERES Guest Scientist Program v1.3
 * http://ssl.mit.edu/spheres/
 * 
 * Copyright 2014 Massachusetts Institute of Technology
 *
 * Last modified 1 March 2014
 *
 * This code is the initial code used for the UDP flight testing.
 *
 */

//***********BEGINNING OF COMM DEFINITIONS***********
//# = Sphere Comm...A = Test Start/Stop...B = Watchdog...
//C = Terminate...D = FORCE/TORQUE...E = Target Info...
//F = Misc




#define TEST_END_TRIG 	1

#define WATCHDOG_LIMIT		3
#define ANY_GOGGLES			0x40
#define SPHERE_ID_REQUEST	0xA2
#define max_length			255

#define START_BYTE			0xA7
#define FROM_SPHERES		0x01
#define FROM_GOGGLES		0x02
#define WATCHDOG_REQUEST	0xB1
#define WATCHDOG_REPLY		0xB2
#define TERMINATE_TEST		0xC1
#define SYNC_TEST			0x91
#define START_TEST			0xA3

#define FORCES_TORQUES		0xD1

#define TGT_POSITION		0xE1
#define IMU_DATA			0xE2
#define ACT_FORCES_TORQUES	0xE3
#define GLOBAL_MET			0xE4
#define BODY_STATE			0xE5
#define INERTIAL_STATE		0xE6
#define INERTIAL_MAT		0xE7
#define INERTIAL_POSE		0xE8
#define INERTIAL_STATE_INSP 0xE9
#define MAN_NUM				0xF0
//***********END OF COMM DEFINITIONS***********

//***********BEGINNING OF COMM STRUCT DEFINITIONS***********
typedef struct 
{
    unsigned int trackMethod;
	unsigned int spinType;
	unsigned int driftCtrl;
} testParametersStruct;

typedef struct {
	unsigned char to;
	unsigned char from;
	unsigned char ID;
	unsigned char test_number;
} msg_start_test;

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

typedef struct {
	float xyzPos[3];
	float xyzVel[3];
} msg_body_tgt_location;

typedef struct {
	unsigned char to;
	unsigned char from;
	unsigned char ID;
	unsigned char spheresLogicID;	
	unsigned int programID;
} msg_spheres_id;

//***********END OF COMM STRUCT DEFINITIONS***********

void gspProcessUART_TestProgram(unsigned char,unsigned char*,unsigned int);
void gspProcessUART_GogglesDaemon(unsigned char,unsigned char*,unsigned int);
void parsePacket(unsigned char *dataPacket);

/*----------------------------------------------------------------------------*/
/*                         End user-modifiable section                        */
/*----------------------------------------------------------------------------*/
