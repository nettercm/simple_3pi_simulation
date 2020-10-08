#ifndef _robot_controller_h_
#define _robot_controller_h_

#include "typedefs.h"


typedef struct
{
	u16 l_s_1;
	u16 l_s_2;
	u16 l_s_3;
	u16 l_s_4;
	u16 l_s_5;
	u16 line;
} t_inputs;


typedef struct
{
	s16 lm;
	s16 rm;
} t_outputs;


typedef struct
{
	t_inputs in;
	t_outputs out;
} t_controller_state;


extern t_controller_state s;

extern void robot_controller(void);

#endif

