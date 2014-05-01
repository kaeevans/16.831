/* 
 * gsp_UDP.c
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

/*************************************/
/* Standard includes.  Do not modify */
/*************************************/
#include <string.h>

#include "comm.h"
#include "comm_datacomm.h"
#include "comm_internal.h"
#include "commands.h"
#include "comm_process_rx_packet.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "pads_internal.h"
#include "pads_internal.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include "smt335async.h"
#include "exp_v2.h"
#include "find_state_error.h"
#include "gsutil_thr_times.h"
#include "gsutil_checkout.h"

///////////////////////////////////////////////
//  Modify as desired below this point, but  //
//  do not change the function prototypes.   //
///////////////////////////////////////////////

#include "gsp_UDP.h"

//***********BEGINNING OF COMM DEFINITIONS***********
//# = Sphere Comm...A = Test Start/Stop...B = Watchdog...
//C = Terminate...D = FORCE/TORQUE...E = Target Info...
//F = Misc
int watchdog_flag;
float forces[3];
float torques[3];
float positionmeas[3];
float velocitymeas[3];

int initTest = 0;
int GogglesAttached = 0;

// values given by the goggles and estimated through the SLAM processing
float J11,J22,J33,J12,J13,J23;
float tgt_estim_pos[3];
float tgt_estim_vel[3];
float tgt_estim_quat[4];
float tgt_estim_angvel[3];

unsigned char packet_buffer[max_length];
int packet_buffer_length = 0;


// Needed for EXPv2 communication
void gspProcessUART_GogglesDaemon(unsigned char source,unsigned char *dataPacket,unsigned int length)
{
	dbg_ushort_packet dbg = {0};
	unsigned char datamsg[32] = {0};
	msg_spheres_id msg = {0};

	if ((length == 1) && (*dataPacket == 0xA1)) {
		//message requests spheres id
		msg.to = ANY_GOGGLES;
		msg.from = commHWAddrGet();
		msg.ID= SPHERE_ID_REQUEST;
		msg.spheresLogicID = (unsigned char) sysIdentityGet();	
		msg.programID = PROG_ID;
		expv2_uart_send(SERIALPORT_DAEMON,sizeof(msg),(unsigned char*) &msg);
		
		//send debug vector
		memcpy(&dbg,&msg, sizeof(msg));
		commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
	}
	else {
    	/* Send the packet to GROUND over the RF */
		memcpy(datamsg, dataPacket, length <= 32 ? length: 32);
	    commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_GOGGLES_PAYLOAD, datamsg, 0);
	}
}

// Needed for EXPv2 communication
void gspProcessUART_TestProgram(unsigned char source,unsigned char *dataPacket,unsigned int length)
{
	int counter = 0;
	
	if (initTest != 1)
	{
		for (counter = 0;counter<length;counter++)
		{	
			if ((dataPacket[counter] == START_BYTE) && (!packet_buffer_length))
			{
				packet_buffer[0] = dataPacket[counter];
				packet_buffer_length++;
			}
			else if (packet_buffer_length)
			{
				packet_buffer[packet_buffer_length++] = dataPacket[counter];
			}

			if ((packet_buffer_length >= 2) && (packet_buffer_length == (int)packet_buffer[1] + sizeof(msg_header)))
			{
				parsePacket(packet_buffer);
				packet_buffer_length = 0;
			}
			else if (packet_buffer_length >= max_length)
			{
				packet_buffer_length = 0;
			}
		}
	}

}

void parsePacket(unsigned char *dataPacket)
{
	msg_header *msg;
	msg_body_forces_torques *msgData;
	msg_body_tgt_location *msgLoc;
	msg_terminate_test *msgTerm;
	msg_inertial_matrix *msgInMat;
	msg_state *msgEstimTGTState;
//	msg_state *msgStateINSP;
//	msg_pose *msgPose;
	dbg_ushort_packet dbg = {0};

	msg = (msg_header *)dataPacket;

		if ((msg->startByte == START_BYTE) && (msg->from == FROM_GOGGLES))
		{
			switch (msg->ID){
			case TERMINATE_TEST:
					msgTerm = (msg_terminate_test *)&dataPacket[sizeof(msg_header)];
					initTest = 0;
					ctrlTestTerminate(msgTerm->return_code);
					//send debug vector
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
				break;
			case FORCES_TORQUES: 
					msgData = (msg_body_forces_torques *)&dataPacket[sizeof(msg_header)];
					//copy forces and torques
					memcpy(forces, msgData->forces,sizeof(float)*3);
					memcpy(torques, msgData->torques,sizeof(float)*3);
					//send debug vector
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
				break;
			case TGT_POSITION:
					msgLoc = (msg_body_tgt_location *)&dataPacket[sizeof(msg_header)];
					memcpy(positionmeas, msgLoc->xyzPos,sizeof(float)*3);
					memcpy(velocitymeas, msgLoc->xyzVel,sizeof(float)*3);
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
				break;
			case INERTIAL_STATE:	 
					msgEstimTGTState = (msg_state *)&dataPacket[sizeof(msg_header)];
					memcpy(tgt_estim_pos, msgEstimTGTState->pos,sizeof(float)*3);
					memcpy(tgt_estim_vel, msgEstimTGTState->vel,sizeof(float)*3);
					memcpy(tgt_estim_quat, msgEstimTGTState->quat,sizeof(float)*4);
					memcpy(tgt_estim_angvel, msgEstimTGTState->angvel,sizeof(float)*3);
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
				break;
			case INERTIAL_MAT:
					msgInMat = (msg_inertial_matrix *)&dataPacket[sizeof(msg_header)];
					J11=msgInMat->J11;
					J22=msgInMat->J22;
					J33=msgInMat->J33;
					J12=msgInMat->J12;
					J13=msgInMat->J13;
					J23=msgInMat->J23;
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
				break;
			case INERTIAL_POSE : 
/*					msgPose = (msg_pose *)&dataPacket[sizeof(msg_header)];
					memcpy(slamPos, msgPose->pos,sizeof(float)*3);
					memcpy(slamQuat, msgPose->quat,sizeof(float)*4);
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
*/
				break;
			case INERTIAL_STATE_INSP : 
/*
					msgStateINSP = (msg_state *)&dataPacket[sizeof(msg_header)];
					memcpy(slamPos, msgStateINSP->pos,sizeof(float)*3);
					memcpy(slamVel, msgStateINSP->vel,sizeof(float)*3);
					memcpy(slamQuat, msgStateINSP->quat,sizeof(float)*4);
					memcpy(slamAngVel, msgStateINSP->angvel,sizeof(float)*3);
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
*/
				break;
			case MAN_NUM+5 :
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
					ctrlManeuverTerminate();
				break;
			case MAN_NUM+6 :
					memcpy(&dbg,&msg, sizeof(msg));
					commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg, 0);
					ctrlManeuverTerminate();
				break;
			case WATCHDOG_REPLY:
				//Reset watchdog timer
				GogglesAttached = 1;
				watchdog_flag = 0;
				break;
			}
	
	}
}
