/*
 * LowPassFilter1P.h
 *
 *  Created on: 2016Äê12ÔÂ3ÈÕ
 *      Author: looke
 */

#ifndef LOWPASSFILTER1ORDER_H_
#define LOWPASSFILTER1ORDER_H_


class LowPassFilter1Order
{
public:
	LowPassFilter1Order();
	LowPassFilter1Order(float initA);
	float apply(float in);
	void setFilter(float newA);

private:
	float a;
	float Y_0;

};


#endif /* LOWPASSFILTER1ORDER_H_ */
