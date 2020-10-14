
#include <Windows.h>
#include <timeapi.h>
#include <stdio.h>
#include <conio.h>
#include <stdarg.h>

#pragma warning( disable : 4244 4101 )

//#include "cocoos/cocoos.h"

#include "typedefs.h"
#include "robot_controller.h"

#include "extApi.h" 

static int clientID;

//object handles
int left_motor, right_motor;				//the 3pi as 2 motors
int l_s_1, l_s_2, l_s_3, l_s_4, l_s_5;		//the 3pi has 5 down-facing line sensors
int robot;

int pingTime;

//the following are used by the move_object() function
const float IGNORE_X = 9999999.0f;
const float IGNORE_Y = 9999999.0f;
const float IGNORE_Z = 9999999.0f;
const float IGNORE_THETA = 9999999.0f;
const float INVALID_XYZ = 9999999.0f;

typedef struct
{
	float robot_position[3];
	float robot_orientation[3];
} t_simulation_state;

t_simulation_state sim_state;


static double pix2=6.283185307179586476925286766559;
static double pi=3.1415926535897932384626433832795;

float* auxValues = NULL;
int* auxValuesCount = NULL;
u08 state;

static float lp1, lp2, lpd, rp1, rp2, rpd;
static double lticks = 0, rticks = 0;

#if 1
//for better performance, at the expense of full synchronization, retrieve data from the simulation in streaming mode; 
//+10 means simulator will send updates ever 10ms
#define STREAMING_MODE (simx_opmode_streaming+10)
#else
//slows things down way too much, but might be necessary in situations where synchronization issues are suspected to be 
//the cause of strange behavior.
#define STREAMING_MODE simx_opmode_oneshot_wait
#endif

typedef struct
{
	//pololu library model
	u16 vbatt;
	s16 m1;
	s16 m2;
	double enc_ab;
	double enc_cd;
	u32 elapsed_milliseconds;

	//world state
	float x, y, theta;
	s16 actual_enc_ab_ticks_per_interval;
	s16 actual_enc_cd_ticks_per_interval;

	u08 vrep_connected;

} t_robot_model;


t_robot_model m;



void move_object(const char *object_name, float x, float y, float theta)
{
	int new_prop, org_prop;
	int result;
	float object_position[3];
	float object_orientation[3];
	int handle;

	result = simxGetObjectHandle(clientID, object_name, &handle, simx_opmode_oneshot_wait);

	simxGetObjectOrientation(clientID, handle,-1, object_orientation,simx_opmode_oneshot_wait);
	if(theta!=IGNORE_THETA) object_orientation[2] = theta;
	simxSetObjectOrientation(clientID, handle, -1, object_orientation, simx_opmode_oneshot_wait);

	simxGetObjectPosition(clientID, handle, -1, object_position, simx_opmode_oneshot_wait);
	if(x!=IGNORE_X) object_position[0] = x;
	if(y!=IGNORE_Y) object_position[1] = y;
	simxSetObjectPosition(clientID, handle, -1, object_position, simx_opmode_oneshot_wait);
}



void vrep_sim_step(void)
{
	int result;
	static int t_sim,t_sim_last=0;
	static int use_actual_position=1;
	static int inter_step_delay=0;
	static int single_step = 0;
	int pingTime;
	int c=0;

	if (single_step)
	{
		while (!_kbhit())
		{
			Sleep(1);
		}
		c = _getch();
	}

	if (_kbhit()) c = _getch();

	if(c)
	{
		if (c == 0x1b)
		{
			simxStopSimulation(clientID, simx_opmode_oneshot_wait);
			simxFinish(clientID);
			exit(0);
		}

		if (c == 'm')
		{
			result = simxSetModelProperty(clientID, robot, 0, simx_opmode_oneshot_wait);
			printf("Robot is NOT movale now!\n");
		}

		if (c == 'M')
		{
			result = simxSetModelProperty(clientID, robot, sim_modelproperty_not_dynamic | sim_modelproperty_not_respondable, simx_opmode_oneshot_wait);
			printf("Robot is movable now!\n");
		}

		if (c == 'x')
		{
			//s.inputs.x =	sim_state.robot_position[0]*1000;
			//s.inputs.y =	sim_state.robot_position[1]*1000;
			//.inputs.theta= sim_state.robot_orientation[2];
		}

		if (c == 'X')
		{
			use_actual_position = !use_actual_position;
		}

		if (c == ' ')
		{
			inter_step_delay = !inter_step_delay;
		}

		if (c == 's')
		{
			single_step = !single_step;
		}
	}

	if (use_actual_position)
	{
		//s.inputs.x =	sim_state.robot_position[0]*1000;
		//s.inputs.y =	sim_state.robot_position[1]*1000;
		//s.inputs.theta= sim_state.robot_orientation[2];
	}

	if (inter_step_delay) Sleep(300);


	//advance the simulation by one step
	result = simxSynchronousTrigger(clientID);
	//if(result != simx_return_ok) printf("simxSynchronousTrigger() failed!  t=%7d\n",t_sim);

#if 0 
	//supposedly this enforces synchronization, i.e. it ensures that simulation step which we initiated with the previous
	//call has actually finished
	//if it has not finished, we might end up retrieving simulted sensor values that are old (i.e. from the previouis step)
	//however, it seems to actually make things worse
	simxGetPingTime(clientID, &pingTime);
#endif

	//now time has advnaced by some number of milliseconds, i.e. as per configured simulation step time;
	//let's find out what time it is; 
	t_sim = simxGetLastCmdTime(clientID);

	//now make sure the robot controller code knows what time it is
	//simply synchronize the absulate time stamp that the robot controller works with, with simulator time
	s.elapsed_milliseconds = t_sim;

	//let's get the robot's actual position, i.e. exact values
	//this can be used in situations where we want to bypass the (simulated) wheel encoder odometry and use "ground truth" instead
	simxGetObjectPosition(clientID,robot,-1,sim_state.robot_position,STREAMING_MODE);
	simxGetObjectOrientation(clientID,robot,-1,sim_state.robot_orientation,STREAMING_MODE);
}




void vrep_sim_outputs(void)
{
	//here we map the target motor speed values as produced by the robot controller, into the corresponding values for the simulaterd motor
	simxSetJointTargetVelocity(clientID,left_motor,  (((float) s.out.lm)/3.85f), STREAMING_MODE);			
	simxSetJointTargetVelocity(clientID,right_motor, (-((float) s.out.rm)/3.85f), STREAMING_MODE);		
}





void vrep_sim_encoders(double encoder_ticks_per_wheel_rev)
{
	double ticks_per_rad;

	//example:  34.014:1 gear with 48cpr encoder =>  1632.672 ticks per revolution   =>   259.84781924773094164045499171293  ticks per rad
	ticks_per_rad = encoder_ticks_per_wheel_rev / pix2;

	//get the current motor axle position - angle in rad;  range of values is rad(-180deg) to rad(+180deg), i.e. not 0...360
	simxGetJointPosition(clientID, left_motor, &lp1, STREAMING_MODE);
	simxGetJointPosition(clientID, right_motor, &rp1, STREAMING_MODE);

#if 0
	//If the motors are running and not blocked/stalled,  we should be seeing a different joint position at every simulation step.
	//Sometimes, we seem to be getting the same value that we got during the last interation.  This means something is not updated.
	//Basically this means that we are not 100% in sync with the simulator.  That can happen when we are streaming data in
	//non-blocking fashion from the simulator to the controller code.
	//However, this should not really be happing often, so we can simply ignore this.
	//The following is a placeholder for some diagnostic code to check if this is happening at all or often.
	if (lp1 == lp2)
	{
		printf("got old values...\n");
	}
#endif

	//check if we have crossed over
	if ((lp1 < -2.0f) && (lp2 > 2.0f))
	{
		lpd = (lp1 - lp2) + pix2;
	}
	else if ((lp1 > 2.0f) && (lp2 < -2.0f))
	{
		lpd = (lp1 - lp2) - pix2;
	}
	else lpd = lp1 - lp2;

	if ((rp1 < -2.0f) && (rp2 > 2.0f))
	{
		rpd = (rp1 - rp2) + pix2;
	}
	else if ((rp1 > 2.0f) && (rp2 < -2.0f))
	{
		rpd = (rp1 - rp2) - pix2;
	}
	else rpd = rp1 - rp2;

	//convert delta axle angle into delta encocer ticks
	lticks = (lpd) * ticks_per_rad;
	rticks = (rpd) * ticks_per_rad;

	//if the change is very small, let's consider it as 0 change
	if (abs(lticks) < 0.05) lticks = 0.0;
	if (abs(rticks) < 0.05) rticks = 0.0;

	//accumulate the incremental changes to the simulated encoder ticks
	m.enc_ab += lticks;
	m.enc_cd += rticks;

	//printf("%7d:  %10.6f,%10.6f\n", t_sim, lticks, rticks);

	lp2 = lp1;
	rp2 = rp1;

	simxGetObjectPosition(clientID, robot, -1, sim_state.robot_position, STREAMING_MODE);
	simxGetObjectOrientation(clientID, robot, -1, sim_state.robot_orientation, STREAMING_MODE);

#if 0
	if (0)
	{
		static double enc_ab = 0, enc_cd = 0;
		enc_ab += lticks;
		enc_cd += rticks;
		//printf("l=%d, r=%d\n",enc_ab,enc_cd);

		printf("actual x,y,theta = %7.4f, %7.4f, %7.4frad/%7.4fdeg   l,r=%7.3f,%7.3f   ab,cd=%7.3f,%7.3f   calc x,y,theta = %7.4f, %7.4f, %7.4f\n",
			sim_state.robot_position[0], sim_state.robot_position[1], sim_state.robot_orientation[2], sim_state.robot_orientation[2] * (180.0f / 3.1415926535897932384626433832795f),
			lticks, rticks, enc_ab, enc_cd,
			s.inputs.x, s.inputs.y, s.inputs.theta * (180.0f / 3.1415926535897932384626433832795f)
		);
	}
#endif
	//------------------------------------------------------------------------------------------------------------------------------------

}


/// <summary>
/// This function reads a single simulated line sensor.
/// Parameter "handle" refers to the vision sensor object.
/// Returns a float value between 0.0 and 1.0, which is derived from the average intensity of the vision sensor image.
/// This function will work with any vision sensor, regardless of resolution or type.
/// </summary>
float vrep_sim_line_sensor(handle)
{
	float* auxValues=NULL;
	int *auxValuesCount=NULL;
	int result;
	u08 state;
	float value=0;
	float noise;
	int i;

	result = simxReadVisionSensor(clientID, handle, &state, &auxValues, &auxValuesCount, STREAMING_MODE); //returns 15 float values;  see documentation
	if (result == 0)
	{
		value = auxValues[10]; //the value at index 10 represents the average intensity of the image, i.e. average of all pixels;  range:  0.00 ... 1.00

		//ok let's add some noise
		noise = (rand() % 100);		// 0   ... 100
		noise = noise / 1000.0;		// 0.0 ... 0.1
		noise = noise - 0.05;		// -0.05 ... +0.05
		noise = noise * 1;			// amplify the noise
		//value = value + noise;
		if (value > 1.0) value = 1.0;
		if (value < 0.0) value = 0.0;

		value = 1.0 - value;  //to mach how the 3pi works
		/*
		for (i = 0; i < 15; i++)
		{
			printf("%d:%f  ", i, auxValues[i]);
		}
		printf("\n");
		*/
	}
	if (auxValues)simxReleaseBuffer((simxUChar*)auxValues);
	if (auxValuesCount)simxReleaseBuffer((simxUChar*)auxValuesCount);

	//printf("value=%f\n", value);
	return value;
}



/// <summary>
/// Retrieves all the inputs from the simulator, i.e. via various simxGet....() API calls, and
/// stores the information where the robot controller expects to find it
/// </summary>
void vrep_sim_inputs(void)
{
	static int t_sim,t_sim_last=0;
	static u32 t_real_now, t_real_last=0; 
	static u32 t_m,t_m_last=0;
	float value, avg, sum;

	t_sim = simxGetLastCmdTime(clientID);
	t_real_now = timeGetTime();
	t_m=m.elapsed_milliseconds;

	vrep_sim_encoders(259.84781924773094164045499171293);

	avg = 0;
	sum = 1; //avoid divide by zero

	value = (1000.0 * vrep_sim_line_sensor(l_s_1));
	avg = avg + 0*value;
	sum = sum + value;
	s.in.l_s_1 = (u16)value;

	value = (1000.0 * vrep_sim_line_sensor(l_s_2));
	avg = avg + 1000*value;
	sum = sum + value;
	s.in.l_s_2 = (u16)value;

	value = (1000.0 * vrep_sim_line_sensor(l_s_3));
	avg = avg + 2000*value;
	sum = sum + value;
	s.in.l_s_3 = (u16)value;

	value = (1000.0 * vrep_sim_line_sensor(l_s_4));
	avg = avg + 3000*value;
	sum = sum + value;
	s.in.l_s_4 = (u16)value;

	value = (1000.0 * vrep_sim_line_sensor(l_s_5));
	avg = avg + 4000*value;
	sum = sum + value;
	s.in.l_s_5 = (u16)value;

	s.in.line = (u16)(avg / sum);

	//printf("1,2,3,4,5,line = %d,%d,%d,%d,%d,  %d\n", s.in.l_s_1, s.in.l_s_2, s.in.l_s_3, s.in.l_s_4, s.in.l_s_5, s.in.line);
}




/// <summary>
/// Obtains handles for each sensor and actuator object that we need and performs some basic initialization of some of the objects.
/// This function needs to be customized for a specific robot.
/// </summary>
void vrep_sim_get_handles(void)
{
	//now obtain handles for all the simulator objects that we need to interact with
	simxGetObjectHandle(clientID, "left_motor", &left_motor, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "right_motor", &right_motor, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "l_s_1", &l_s_1, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "l_s_2", &l_s_2, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "l_s_3", &l_s_3, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "l_s_4", &l_s_4, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "l_s_5", &l_s_5, simx_opmode_oneshot_wait);

	simxGetObjectHandle(clientID, "robot", &robot, simx_opmode_oneshot_wait);


	//initialize a few items to sane values;  
	//for example we don't want the motors to start spinning in case they were configured with a default speed 
	simxSetJointTargetVelocity(clientID, left_motor, 0, simx_opmode_oneshot_wait);
	simxSetJointTargetVelocity(clientID, right_motor, 0, simx_opmode_oneshot_wait);
	simxGetObjectPosition(clientID, robot, -1, sim_state.robot_position, simx_opmode_oneshot_wait);
	simxGetJointPosition(clientID, left_motor, &lp1, STREAMING_MODE);
	simxGetJointPosition(clientID, right_motor, &rp1, STREAMING_MODE);
}





void vrep_sim_init(void)
{
	int result;

	//establish a connection with the siulator
	clientID=simxStart((simxChar*)"127.0.0.1",19997,1,1,2000,1);
	printf("clientID=%d\n",clientID);


	//make sure the simulation is not currently running
	result = simxStopSimulation(clientID,simx_opmode_oneshot_wait);
	if(result != simx_return_ok) printf("simxStopSimulation() failed!\n");

	//start the simulation and put it into synchronous mode
	result = simxSynchronous(clientID, 1);
	if (result != simx_return_ok) printf("simxSynchronous() failed!\n");
	result = simxStartSimulation(clientID,simx_opmode_oneshot_wait);
	if(result != simx_return_ok) printf("simxStartSimulation() failed!\n");

	//obtain handles to sensor and actuator objects
	vrep_sim_get_handles();

	//now move the simulation forward by one step, so that things are fully initialized
	printf("sim time prior to first step = %d  (this should be 0)\n", simxGetLastCmdTime(clientID));
	simxSynchronousTrigger(clientID);
	if(result != simx_return_ok) printf("simxSynchronousTrigger() failed!\n");
	simxGetPingTime(clientID, &pingTime); //this forces synchronization
	//printf("pingTime=%d\n", pingTime); //the first time this ping API is called it always takes a long time so reporting it here is not useful
	printf("sim time after first step    = %d (this should be equal to the configured step time)\n", simxGetLastCmdTime(clientID));
	
	//at this point the simulation is fully initialized and running and has completed so far exactly one time stamp
}





void display_loops_per_second(void)
{
	static u32 t1, t2;
	static u32 loops = 0;

	if (loops == 0) t1 = timeGetTime();

	loops++;
	t2 = timeGetTime();
	if (t2 - t1 >= 2000)
	{
		printf("%lu loops / second\n", loops / 2);
		loops = 0;
		t1 = t2;
	}
}



int main(int argc, char** argv)
{
	timeBeginPeriod(1); //improves the resolution of certain Windows timer APIs - not sure if still needed on Windows 10 to be honest.....
	srand(0);


	memset(&m, 0, sizeof(m));
	vrep_sim_init();


	while (1)
	{
		vrep_sim_step();
		vrep_sim_inputs();

		robot_controller();

		vrep_sim_outputs();

		display_loops_per_second();
	}
}
