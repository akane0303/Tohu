/*
 * Duty.h
 *
 *  Created on: Feb 21, 2021
 *      Author: utida
 */

#ifndef INC_DUTY_H_
#define INC_DUTY_H_

class Duty {
private:
	double g,r,v,n,t_0,n_0;
public:
	Duty(double g_,double r_,double t_0_,double n_0_);
	int calc(double v,double p);
};

#endif /* INC_DUTY_H_ */
