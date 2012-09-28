#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "PID.h"

static double now()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}

//#define saturate_cmp(x,xmax) (((x)<-(xmax))?-(xmax):(((x)>(xmax))?(xmax):(x)))
//#define saturate(x,xmax) (((xmax)>0)?(saturate_cmp(x,xmax)):(x))

double saturate(double x, double xmax) {
	if (xmax > 0) {
		if (x > xmax) x = xmax;
		if (x < -xmax) x = -xmax;
	}
	return x;
}

double PID::operator()(double ystar, double y)
{
	e = ystar - y;
	if (angle) {
		e = remainder(e,2*M_PI);
	}
	return operator()(e);
}

double PID::operator()(double e)
{
	double output, de;

	saturate(e,Emax);
	if (autodt) {
		double t = now();
		if (last_t<0) {
			dt = 0;
		} else {
			dt = t - last_t;
		}
		last_t = t;
	}
	de = (e-last_e)/dt;
	last_e = e;
	I += e * dt;
	I = saturate(I,Imax);
	output = Kp*e + Ki*I + Kd*de;
	output = saturate(output,Omax);
	return output;
}

