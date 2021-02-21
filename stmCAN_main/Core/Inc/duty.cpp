/*
 * Duty.cpp
 *
 *  Created on: Feb 21, 2021
 *      Author: utida
 */
#include <duty.h>
#include <math.h>

Duty::Duty(double g_,double r_,double t_0_,double n_0_){
	g=g_;
	r=r_;
	t_0=t_0_;
	n_0=n_0_;
}
int Duty::calc(double v,double p){
	double p_a=(g*v/r)*(t_0-((t_0/n_0)*((60*g*v)/(2*M_PI*r))));
	int d=(p_a/p)*(p_a/p);
	return d;
}
