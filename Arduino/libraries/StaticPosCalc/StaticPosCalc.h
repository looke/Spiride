/*
 * StaticPosCalc.h
 *
 *  Created on: 2016��11��20��
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

	//�ش�ƫ�Ƿ��������ݴ�ֵ������Ԫ��Q������ʹ�����ϴ�ļ��ٶ�Axֵ
	//��ʼ��̬�����в���Ҫʹ�ã���Ϊ��ʼ��̬����ʱû�м��ٶȸ���
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
	void calcMagInclinationByQaut(); //����ţ�ٵ������������Ԫ��������ش�ƫ��Mny��Mnz��������ķ���ֵ

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
