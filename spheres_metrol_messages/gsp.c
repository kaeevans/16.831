/* 
 * gsp.c
 *
 * SPHERES Test Project example code
 *
 * Created by Bryan McCarthy, Aug 13
 * Edited by Tim Setterfield, Oct 13
 *
 *
 * MIT Space Systems Laboratory
 * http://ssl.mit.edu/spheres/
 * 
 * Copyright 2005 Massachusetts Institute of Technology
 */


/*----------------------------------------------------------------------------*/
/*                         Do not modify this section.                        */
/*----------------------------------------------------------------------------*/

#include "comm.h"
#include "commands.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "pads.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include "smt335async.h"
#include "exp_v2.h"
#include <string.h>

/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
#include "gsp_UDP.h"


//// Additional includes
#include "pads_internal.h"    // needed for padsInertialBufferCapacity()
#include <string.h>        // needed for memset()
#include "find_state_error.h" // needed for findStateError()
#include "ctrl_mix.h"         // needed for ctrlMixWLoc()
#include "ctrl_position.h"    // needed for ctrlPositionPDgains()
#include "ctrl_attitude.h"       // needed for ctrlAttitudeNLPDwie()

//// Use this function to set satellite identity; it is the first primary interface function called
void gspIdentitySet()
{
   //// Set the logical identifier (SPHERE#) for this vehicle, which is defined in gsp.h in the spheren folder for each SPHERE
   sysIdentitySet(SPHERE_ID);
}


//// Use this function to initialize communications and other subsystems; must contain certain initialization functions for multi-sphere operations to work correctly
void gspInitProgram()
{
   //// Set the unique program identifier (must match the .ini file program number when using the Flight GUI)
   sysProgramIDSet(1);

   //// Set up communications TDMA frames (STL means SPHERE to Laptop)
   commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), NUM_SPHERES);

   //// Enable communication channel
   commTdmaEnable(COMM_CHANNEL_STL); 
   
   //// Allocate storage space for IMU samples
   padsInertialAllocateBuffers(50);

   //// Inform system of highest beacon number in use (defined in gsp.h)
   //// There are typically 5 beacons set to beacon numbers 1, 3, 5, 7, 9
   padsInitializeFPGA(NUM_BEACONS);

   //// Set location of the experiments (these are defined in SpheresCore/spheres_constants.h)
   sysLocationSet(LOCATION_LAB);
   
 
      /* custom program initialization goes below this point */
   *SMT335CP4 = 0x1104; // Talk to the Expansion Board at 250 kbps
    
    /* Register one or both UART channels */
    expv2_uart_cbk_register(SERIALPORT_DAEMON,&gspProcessUART_GogglesDaemon);
                /* OR */
    expv2_uart_cbk_register(SERIALPORT_TESTPROG,&gspProcessUART_TestProgram);
}


//// Use this function to perform test-specific configuration; called prior to starting each test
void gspInitTest(unsigned int test_number)
{
   //// Create a 'state_vector' called 'initState', which is where you want to place the SPHERE in the global frame to start the test
   //// This sample state vector sets z=0.8m (down from center of volume), which is the height of the SPHERE on an air carriage
   //// Setting quat1=1.0 means the SPHERE's tank is down and its +X face is aligned with the +X axis of the global frame
   state_vector initState = {0.0f,0.0f,0.8f,  0.0f,0.0f,0.0f,  1.0f,0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f};
   
   //// Set the control period to 1000 ms (the function gspControl will run this often)
   //// The standard control cycle for SPHERES is 1000 ms
   ctrlPeriodSet(1000);

   //// Turn on global estimator
   padsEstimatorInitWaitAndSet(initState, padsInertialBufferCapacity(), 200, 105, PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9);
}


//// Use this function to specify a task trigger mask
void gspInitTask()
{


}


//// Use this function if you want to do something with accelerometer or gyro data, such as state estimation; called periodically 
void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro, unsigned int num_samples)
{
}


//// Use this function to record global data; called at the end of each beacon’s transmission period
void gspPadsGlobal(unsigned int beacon, beacon_measurement_matrix measurements)
{
}


//// Use this function for event-driven estimation, control, and communications tasks; called whenever a masked event occurs
void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data)
{
   	// Sending EXPV2 test message
	#if SPHERE_ID == SPHERE1	
	state_vector gspTaskState;
	unsigned int testTime;
	unsigned char global_met_packet_data[66];		
	int msg_length = 0;
	msg_header *mHead;
	msg_state *mGMData;	
	
	mHead = (msg_header *) &global_met_packet_data;
	mGMData = (msg_state *) &global_met_packet_data[sizeof(msg_header)];
		
	// When global state information is received from the last beacon (beacon 9), send this data to the goggles
	if (gsp_task_trigger == PADS_ESTIMATOR_DONE_TRIG && extra_data == 9)
	{		
		//Send 1 packet with ID: GLOBAL_MET and the required data
		padsStateGet(gspTaskState);					// get global state and put in gspTaskState
		testTime = ctrlTestTimeGet();
		
		mHead->startByte = START_BYTE;
		mHead->ID = GLOBAL_MET;
		mHead->from = FROM_SPHERES;
		mHead->length = sizeof(msg_state);
		
		memcpy(&mGMData->testTime, &testTime, sizeof(unsigned int));
		memcpy(mGMData->pos, gspTaskState, sizeof(float)*3);
		memcpy(mGMData->vel, &gspTaskState[3], sizeof(float)*3);
		memcpy(mGMData->quat, &gspTaskState[6], sizeof(float)*4);
		memcpy(mGMData->angvel, &gspTaskState[10], sizeof(float)*3);
		
		msg_length = (sizeof(msg_header))+(sizeof(msg_state));
		
		expv2_uart_send(SERIALPORT_TESTPROG,msg_length,(unsigned char*) &global_met_packet_data);
	}
	#endif

}


//// Use this function to apply control laws and set thruster on-times; called periodically
void gspControl(unsigned int test_number, unsigned int test_time, unsigned int maneuver_number, unsigned int maneuver_time)
{
   //// Thruster variable declarations: Duty cycle typically 20% on ISS, 40% on ground; Minimum useful thruster pulse is 10 ms
   float duty_cycle = 40.0f;
   int min_pulse = 10;

   //// Control variable declarations
   float ctrlControl[6];             // 6 element vector: 3 forces and 3 torques
   prop_time firing_times;           // Create a 'prop_vector' called 'firing_times', in which you will put thruster firing times
   state_vector ctrlState;           // Create a 'state_vector' called 'ctrlState', which will contain the current estimated state of the SPHERE
   state_vector ctrlStateTarget;     // Create 'ctrlStateTarget', which is the SPHERE's target state
   state_vector ctrlStateError;      // Create 'ctrlStateError', which is the error between the target state and the current estimated state

   //// Control gain declarations; these are standard SPHERES gains found in SpheresCore/spheres_physical_XXX.c, but can easily be modified for different performance
   extern const float KPpositionPD;  // Proportional gain for the SPHERES position PD controller
   extern const float KDpositionPD;  // Derivative gain for the SPHERES position PD controller
   extern const float KPattitudePD;  // Proportional gain for the SPHERES attitude PD controller
   extern const float KDattitudePD;  // Derivative gain for the SPHERES attitude PD controller

   //// Debugging vector declarations; debug vector can be 16 elements of type 'short' or 8 elements of type 'float'; sending debug vector is optional
   //short DebugVector_short[16];
   //float DebugVector_float[8];

   //// Clear memory
   memset(ctrlControl,0,sizeof(float)*6);
   memset(&firing_times,0, sizeof(prop_time));
   memset(ctrlState, 0, sizeof(state_vector));
   memset(ctrlStateTarget, 0, sizeof(state_vector));
   memset(ctrlStateError, 0, sizeof(state_vector));
   //memset(DebugVec,0,sizeof(short)*16);
   //memset(DebugFloat,0,sizeof(float)*8);

   //// Run this function to get the current estimate of the SPHERE's state, which is then stored in 'ctrlState'
   padsStateGet(ctrlState);

   //// Use the following structure to set up tests and maneuvers
   switch(test_number)
   {
      case 1:  //// This is the beginning of Test 1  
         switch(maneuver_number)
         {
            case 1:  //// This is the beginning of Maneuver 1
               
               //// Typically, Maneuver 1 is used for estimator convergence only (wait ~10 s during this maneuver)

               //// Maneuver ending conditions generally go here and are typically time-outs
               if (maneuver_time > 10000) //// If maneuver is being executed for greater than 10 s
               {
                  //// Terminate the maneuver; ctrlManeuverTerminate increments the maneuver number
                  ctrlManeuverTerminate();
               }
               break; //// end of Maneuver 1

            case 2:  //// This is the beginning of Maneuver 2
               
               //// Command a specific target state using ctrlStateTarget; for example, command x-position to 1.0 m and y-position to -0.5 m and QUAT_1 to 1
               //// QUAT_1 = 1 means tank-down (since the SPHERES body frame and the global frame are rotated 180 deg about the x-axis from each other) and gives the quaternion the necessary unit magnitude
               ctrlStateTarget[POS_X] = 0.5f;
               ctrlStateTarget[POS_Y] = -0.5f;
               ctrlStateTarget[QUAT_1] = 1.0f;

               if (maneuver_time > 30000)
               {
                  ctrlManeuverTerminate();
               }
               break; //// end of Maneuver 2

            case 3:
               
               //// Command firing times directly as follows (firing thruster number 1 for 1000 ms)
               firing_times.on_time[1] = 0;
               firing_times.off_time[1] = 1000;
               
               //// Termination conditions do not have to be time-based
               if (ctrlState[POS_X] < 1.5f)
               {
                  ctrlManeuverTerminate();
               }
               break;

            default: //// This code is executed if 'maneuver_number' does not match any of the cases above
               
               //// Test termination conditions generally go here; you can return any single value per satellite per test
               //// Typical return values are 'normal', which is defined as '1' in SpheresCore/control.h, or 'error', which is defined as 7 in SpheresCore/control.h, among others
               ctrlTestTerminate(TEST_RESULT_NORMAL);
               //ctrlTestTerminate(TEST_RESULT_ERROR);
               //ctrlTestTerminate(insert any number here)
               break;  //// end of default maneuver
         }
         break; //end of Test 1
            
      case 2: //// This is the beginning of Test 2
         switch(maneuver_number)
         {
            case 1:
               {
               }
               break;
            case 2:
               {
               }
               break;
            default:
               {
               }
               break;   
         }
         break; 
      default: //// This is the beginning of the default test
         break;
   }

   if (maneuver_number >1) //// Do not execute the following code during Manuever 1 (estimator convergence)
   {
      if (maneuver_number == 2) //// Only execute the following code during Maneuver 2 (when commanding a target state)
      {
         //// Calculate the ctrlStateError between the ctrlState and the ctrlStateTarget
         findStateError(ctrlStateError,ctrlState,ctrlStateTarget);

         //// The control law goes here; you can use a standard SPHERES controller or make your own
         //// Standard SPHERES controller (found in SpheresCore/Science/controllers)
         ctrlPositionPDgains(KPpositionPD,KDpositionPD,KPpositionPD,KDpositionPD,KPpositionPD,KDpositionPD,ctrlStateError,ctrlControl);
         ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,ctrlStateError,ctrlControl);
         //// Sample PD control law for x-axis position and z-axis attitude
         ctrlControl[FORCE_X] = KPpositionPD * ctrlStateError[POS_X] + KDpositionPD * ctrlStateError[VEL_X];
         ctrlControl[TORQUE_Z] = KPattitudePD * ctrlStateError[QUAT_3] + KDattitudePD * ctrlStateError[RATE_Z];

         //// This mixer converts forces and torques into thruster firing times for a single SPHERE
         ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, duty_cycle, FORCE_FRAME_INERTIAL);
      }

      if (maneuver_number == 3) //// Only execute the following code during Maneuver 3 (when commanding forces or torques directly)
      {
         //// This mixer converts forces and torques into thruster firing times for a single SPHERE
         ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, duty_cycle, FORCE_FRAME_INERTIAL);
      }

      //// Turn off global metrology while commanding thrusters to fire by setting the PADS period to forever
      padsGlobalPeriodSetAndWait(SYS_FOREVER,SYS_FOREVER);

      //// Set thruster firing times
      propSetThrusterTimes(&firing_times);
       
      //// Restart metrology to a 200 ms period, allowing 200ms or 400ms for thrusting (depending on duty cycle)
      padsGlobalPeriodSetAndWait(200,200);
      padsGlobalPeriodSetAndWait(200,400);
   }
   


}


//// Use this function if you want to do something right at the end of a test
#ifdef TEST_END_TRIG
void gspEndTest(unsigned int test_number, unsigned char ctrl_result)
{
}
#endif


//// The SPHERE runs this function every time it receives a packet from another SPHERE
void gspProcessRXData(default_rfm_packet packet)
{
}

