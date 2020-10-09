#ifndef _robot_controller_h_
#define _robot_controller_h_

#ifdef __cplusplus
extern "C"
{
#endif


#include "typedefs.h"
#include <stdio.h>
#include <math.h>

typedef struct t_inputs_
{
	u16 l_s_1,  l_s_2,  l_s_3,  l_s_4,  l_s_5;		//the 5 individual line sensors on the 3pi
	u16 line;										//the Polou QTR read_line() API returns a composite value 
} t_inputs;



typedef struct t_outputs_
{
	s16 lm, rm;		//left and rigth motor speed - in units as per Pololu set_motors() library function;
} t_outputs;



typedef struct t_controller_state_
{
	t_inputs in;
	t_outputs out;
	u32 elapsed_milliseconds;
} t_controller_state;


extern volatile t_controller_state s;

extern void robot_controller(void);





#ifdef __cplusplus
} // extern "C"
#endif



#endif

