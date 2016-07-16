/*
 * PID.c
 *
 *
*/

#include "PID.h"
/*
 * 	PID controller
 */
double pidControl(PIDstruct_t * pid, double error, double position)
{
	double res, propT, derivT, integT;

	propT = pid->gainProp * error; 		// calculate the proportional term
// calculate the integral state with appropriate limiting
	pid->integStat += error;
	if (pid->integStat > pid->maxInteg) pid->integStat = pid->maxInteg;
	else if (pid->integStat < pid->minInteg) pid->integStat = pid->minInteg;
// calculate the integral term
	integT = pid->gainInteg * pid->integStat;
// calculate the derivative term
	derivT = pid->gainDeriv * (position - pid->derivStat);
	pid->derivStat = position;
	 res = propT + integT - derivT;
	return res;
}


