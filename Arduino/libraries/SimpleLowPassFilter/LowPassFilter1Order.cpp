/*
 * LowPassFilter1Order.cpp
 *
 *  Created on: 2016Äê12ÔÂ3ÈÕ
 *      Author: looke
 */

#include "LowPassFilter1Order.h"

#include "math.h"

#define defaultA 0.15f

LowPassFilter1Order::LowPassFilter1Order()
{
	this->a = defaultA;
	Y_0 = 0;
};

LowPassFilter1Order::LowPassFilter1Order(float initA)
{
	if(initA > 0 && initA < 1)
	{
		this->a = initA;
	}
	else
	{
		this->a = defaultA;
	}
	Y_0 = 0;
};

void LowPassFilter1Order::setFilter(float newA)
{
	if(newA > 0 && newA < 1)
	{
		this->a = newA;
	}
};

float LowPassFilter1Order::apply(float in)
{
	float temp = (1-a)*this->Y_0;
	if(!isfinite(temp))
	{
		temp = (1-a)*in;
	}
	float out = a * in + temp;
	Y_0 = out;
	return out;
};

