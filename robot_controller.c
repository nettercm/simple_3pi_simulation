
#include "robot_controller.h"



#ifdef __cplusplus
extern "C"
{
#endif



volatile t_controller_state s;

const s16 LINE_CENTER_VALUE = 2000; //the pololu read_line() API returns a value of 2000 if the robot is centered on the line
const u16 LINE_NOISE_FLOOR = 200;
const s16 NOMINAL_SPEED = 50;

void robot_controller(void)
{
	s16 position;
	s16 steering;
	static s16 last_steering = 0;
	s16 speed_adjustment;

	//how far off the line center are we?   position==0 means we are on center more or less
	position = (s16)s.in.line - LINE_CENTER_VALUE;

	steering = position / 20;

	//if we completely lost the line, take a default action
	if ((s.in.l_s_1 < LINE_NOISE_FLOOR) && (s.in.l_s_2 < LINE_NOISE_FLOOR) && (s.in.l_s_3 < LINE_NOISE_FLOOR) && (s.in.l_s_4 < LINE_NOISE_FLOOR) && (s.in.l_s_5 < LINE_NOISE_FLOOR))
	{
		steering = 0;					//we lost the line; go straight
		//steering = last_steering;		//keept the previous steering angle
	}

	//if we end up having to steer either left or right, then lets also slow down a little bit. The harder we are steering, the more we should slow down.
	speed_adjustment = abs(steering) / 2;

	s.out.lm = NOMINAL_SPEED + steering - speed_adjustment;
	s.out.rm = NOMINAL_SPEED - steering - speed_adjustment;

	last_steering = steering;

#ifdef WIN32

	printf("%6lu:  l=%4d,%4d,%4d,%4d,%4d,  L=%4d,  s=%5d,  sa=%5d  m=%4d,%4d\n",
		s.elapsed_milliseconds,
		s.in.l_s_1, s.in.l_s_2, s.in.l_s_3, s.in.l_s_4, s.in.l_s_5, 
		s.in.line,
		steering, speed_adjustment, 
		s.out.lm, s.out.rm);

#endif
}




#ifdef __cplusplus
} // extern "C"
#endif
