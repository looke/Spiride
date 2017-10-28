/*
 * LowPassFilterFirstOrder.cpp
 *
 *  Created on: 2016Äê12ÔÂ3ÈÕ
 *      Author: looke
 */

#include "LowPassFilterFirstOrder.h"

#include "math.h"

#define defaultA 0.15f

LowPassFilterFirstOrder::LowPassFilterFirstOrder()
{
	this->a = defaultA;
	Y_0 = 0;
};

LowPassFilterFirstOrder::LowPassFilterFirstOrder(double initA)
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

void LowPassFilterFirstOrder::setFilter(double newA)
{
	if(newA > 0 && newA < 1)
	{
		this->a = newA;
	}
};

void LowPassFilterFirstOrder::resetFilter(double newA)
{
	if(newA > 0 && newA < 1)
	{
		this->a = newA;
	}
	Y_0 = 0;
};

void LowPassFilterFirstOrder::resetFilter()
{
	resetFilter(defaultA);
};


double LowPassFilterFirstOrder::apply(double in)
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

