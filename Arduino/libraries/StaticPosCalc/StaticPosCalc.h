/*
 * StaticPosCalc.h
 *
 *  Created on: 2016年11月20日
 *      Author: looke
 */

#ifndef STATICPOSCALC_H_
#define STATICPOSCALC_H_
#include "MyMatrix.h"

class StaticPosCalc
{
public:
	StaticPosCalc();
	StaticPosCalc(float accValue[3], float magValue[3]);
	StaticPosCalc(float accValue[3], float magValue[3], float qInit[4]);

	float q_deltaValue[4];
	float q_value[4];

	float magValue[3];
	float accValue[3];


	float fx[4];

	float eulerAngel_pitch;
	float eulerAngel_roll;
	float eulerAngel_yaw;

	float precision;

	//地磁偏角分量，根据此值计算四元数Q，避免使用误差较大的加速度Ax值
	//初始姿态计算中不需要使用，因为初始姿态计算时没有加速度干扰
	float Mny;
	float Mnz;

	void printJacobiMatrix();
	void printEulerAngel();
	void printCurrentQaut();
	void printCurrentDeltaQ();
	void printMagInclination();

	void setSensorValue(float accValue[3], float magValue[3]);
	void setInitQ(float qInit[4]);
	void setPrecision(float prec);

	float* getQValue();
	float getPitch();
	float getRoll();
	float getYaw();

	void generateJacobiMatrix();
	void updateDeltaQFromJacobiRoot();
	void calcRootsJacobiMatrix();

	void calcFX();

	//ACC Equations
	float calcACC_eq0();
	float calcACC_eq1();
	float calcACC_eq2();
	//Mag Equations
	float calcMag_eq0();
	float calcMag_eq1();
	float calcMag_eq2();

	void calcStaticPos();
	void QautToEuler();
	void calcMagInclinationByQaut(); //根据牛顿迭代计算出的四元数，推算地磁偏角Mny和Mnz两个方向的分量值

	void test();
	void test_Euler();
protected:

	void initDefaultValue();
	void initDefaultValue(float accValue[3], float magValue[3]);




	void calcQ();

	void calcEulerAngelByQ();
	bool isPrcisionReached();
protected:
	MyMatrix jacobiMatrix;
};

#endif /* STATICPOSCALC_H_ */
