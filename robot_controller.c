
#include "robot_controller.h"

t_controller_state s;


void robot_controller(void)
{
	s16 line;

	line = (s16)s.in.line - 2000;
	line = line / 20;

	if ((s.in.l_s_1 < 200) && (s.in.l_s_2 < 200) && (s.in.l_s_3 < 200) && (s.in.l_s_4 < 200) && (s.in.l_s_5 < 200))
	{
		//we lost the line; go straight
		line = 0;
	}
	s.out.lm = 100 + line;
	s.out.rm = 100 - line;
}