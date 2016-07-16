/*
 * ftoa.h
 *
 *  Created on: 27/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_FTOA_H_
#define SOURCE_FTOA_H_

#include <string.h>

#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

char * ftoa(double f, char * buf, int precision);
void itoa(int n, char s[]);


#endif /* SOURCE_FTOA_H_ */
