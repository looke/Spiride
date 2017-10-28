/*
 * LowPassFilter1P.h
 *
 *  Created on: 2016Äê12ÔÂ3ÈÕ
 *      Author: looke
 */

#ifndef LOWPASSFILTER_FIRSTORDER_H_
#define LOWPASSFILTER_FIRSTORDER_H_


class LowPassFilterFirstOrder
{
public:
	LowPassFilterFirstOrder();
	LowPassFilterFirstOrder(double initA);
	double apply(double in);
	void setFilter(double newA);
	void resetFilter();
	void resetFilter(double newA);
private:
	double a;
	double Y_0;

};


#endif /* LOWPASSFILTER1ORDER_H_ */
