/*
 * PID.h
 *
 *  Created on: 1/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCES_PID_H_
#define SOURCES_PID_H_





typedef struct
{
	double derivStat; 			// Last status input
	double integStat; 			// Integr status
	double maxInteg; 			// Max. integr status
	double minInteg;			// Min. integr status
	double gainInteg; 			// integ gain
	double gainProp; 			// prop gain
	double gainDeriv; 			// deriv gain
} PIDstruct_t;


extern double pidControl(PIDstruct_t * pid, double error, double position);

#endif /* SOURCES_PID_H_ */
