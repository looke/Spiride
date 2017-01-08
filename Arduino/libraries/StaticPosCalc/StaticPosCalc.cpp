/*
 * StaticPosCalc.cpp
 *
 *  Created on: 2016年11月20日
 *      Author: looke
 */

#include "StaticPosCalc.h"
//#include <iostream>
//#include <cmath>
#include "math.h"
#define PI 3.14159265
using namespace std;


StaticPosCalc::StaticPosCalc()
{
	this->initDefaultValue();
};


StaticPosCalc::StaticPosCalc(float accValue[3], float magValue[3])
{
	this->initDefaultValue();
	this->setSensorValue(accValue, magValue);
};


StaticPosCalc::StaticPosCalc(float accValue[3], float magValue[3], float qInit[4])
{
	this->initDefaultValue();
	this->setSensorValue(accValue, magValue);
	this->setInitQ(qInit);
};
/*
void StaticPosCalc::printJacobiMatrix()
{
	cout << "Jacobi Matrix:" << endl;
	this->jacobiMatrix.printMatrix();
	cout << "---------------------------------------" << endl;
};
*/

void StaticPosCalc::calcFX()
{
	//使用3个加速计等式，加上1个磁力计等式
	this->fx[0] = this->calcACC_eq0();
	this->fx[0] = 0-(this->fx[0]);

	this->fx[1] = this->calcACC_eq1();
	this->fx[1] = 0-(this->fx[1]);

	this->fx[2] = this->calcACC_eq2();
	this->fx[2] = 0-(this->fx[2]);

	this->fx[3] = this->calcMag_eq0();
	this->fx[3] = 0-(this->fx[3]);
};

float StaticPosCalc::calcACC_eq0()
{
	float value;
	// 2Q1Q3 - 2Q0Q2 + Ax
	value = 2*this->q_value[1]*this->q_value[3]
		  - 2*this->q_value[0]*this->q_value[2]
		  + this->accValue[0];
	return value;
};

float StaticPosCalc::calcACC_eq1()
{
	float value;
	// 2Q2Q3 + 2Q0Q1 + Ay
	value = 2*this->q_value[2]*this->q_value[3]
		  + 2*this->q_value[0]*this->q_value[1]
		  + this->accValue[1];
	return value;

};

float StaticPosCalc::calcACC_eq2()
{
	float value;
	// Q0^2 - Q1^2 - Q2^2 + Q3^2 + Az
	value = this->q_value[0]*this->q_value[0]
		  - this->q_value[1]*this->q_value[1]
		  - this->q_value[2]*this->q_value[2]
		  + this->q_value[3]*this->q_value[3]
		  + this->accValue[2];
	return value;
};

float StaticPosCalc::calcMag_eq0()
{
	// MAGx * (Q0^2 + Q1^2 - Q2^2 - Q3^2)
	float part1 = this->q_value[0]*this->q_value[0]
			    + this->q_value[1]*this->q_value[1]
				- this->q_value[2]*this->q_value[2]
				- this->q_value[3]*this->q_value[3];
	part1 = this->magValue[0] * part1;

		// MAGy * (2Q1Q2 - 2Q0Q3)
	float part2 = 2*this->q_value[1]*this->q_value[2]
				- 2*this->q_value[0]*this->q_value[3];
	part2 = this->magValue[1] * part2;

	// MAGz * (2Q1Q3 + 2Q0Q2)
	float part3 = 2*this->q_value[1]*this->q_value[3]
				+ 2*this->q_value[0]*this->q_value[2];
	part3 = this->magValue[2] * part3;

	return part1 + part2 + part3;
};

float StaticPosCalc::calcMag_eq1()
{
	// MAGx * (2Q1Q2 + 2Q0Q3)
	float part1 = 2*this->q_value[1]*this->q_value[2]
			    + 2*this->q_value[0]*this->q_value[3];
	part1 = this->magValue[0] * part1;

		// MAGy * (Q0^2 - Q1^2 + Q2^2 - Q3^2)
	float part2 = this->q_value[0]*this->q_value[0]
			    - this->q_value[1]*this->q_value[1]
			    + this->q_value[2]*this->q_value[2]
			    - this->q_value[3]*this->q_value[3];
	part2 = this->magValue[1] * part2;

		// MAGz * (2Q2Q3 - 2Q0Q1)
	float part3 = 2*this->q_value[2]*this->q_value[3]
			    - 2*this->q_value[0]*this->q_value[1];
	part3 = this->magValue[2] * part3;

	return part1 + part2 + part3;
};

float StaticPosCalc::calcMag_eq2()
{
	// MAGx * (2Q1Q3 - 2Q0Q2)
	float part1 = 2*this->q_value[1]*this->q_value[3]
			    - 2*this->q_value[0]*this->q_value[2];
	part1 = this->magValue[0] * part1;

	// MAGy * (2Q2Q3 + 2Q0Q1)
	float part2 = 2*this->q_value[2]*this->q_value[3]
			    + 2*this->q_value[0]*this->q_value[1];
	part2 = this->magValue[1] * part2;

	// MAGz * (Q0^2 - Q1^2 - Q2^2 + Q3^2)
	float part3 = this->q_value[0]*this->q_value[0]
			    - this->q_value[1]*this->q_value[1]
			    - this->q_value[2]*this->q_value[2]
			    + this->q_value[3]*this->q_value[3];
	part3 = this->magValue[2] * part3;

	return part1 + part2 + part3;
};

void StaticPosCalc::generateJacobiMatrix()
{
	float temp;

	//calc dF[0][0]
	temp = -2*this->q_value[2];
	this->jacobiMatrix.setMatrixElement(0,0,temp);
	//calc dF[0][1]
	temp = 2*this->q_value[3];
	this->jacobiMatrix.setMatrixElement(0,1,temp);
	//calc dF[0][2]
	temp = -2*this->q_value[0];
	this->jacobiMatrix.setMatrixElement(0,2,temp);
	//calc dF[0][3]
	temp = 2*this->q_value[1];
	this->jacobiMatrix.setMatrixElement(0,3,temp);
	//set dF[0][4]
	this->jacobiMatrix.setMatrixElement(0,4,this->fx[0]);


	//calc dF[1][0]
	temp = 2*this->q_value[1];
	this->jacobiMatrix.setMatrixElement(1,0,temp);
	//calc dF[1][1]
	temp = 2*this->q_value[0];
	this->jacobiMatrix.setMatrixElement(1,1,temp);
	//calc dF[1][2]
	temp = 2*this->q_value[3];
	this->jacobiMatrix.setMatrixElement(1,2,temp);
	//calc dF[1][3]
	temp = 2*this->q_value[2];
	this->jacobiMatrix.setMatrixElement(1,3,temp);
	//set dF[1][4]
	this->jacobiMatrix.setMatrixElement(1,4,this->fx[1]);


	//calc dF[2][0]
	temp = 2*this->q_value[0];
	this->jacobiMatrix.setMatrixElement(2,0,temp);
	//calc dF[2][1]
	temp = -2*this->q_value[1];
	this->jacobiMatrix.setMatrixElement(2,1,temp);
	//calc dF[2][2]
	temp = -2*this->q_value[2];
	this->jacobiMatrix.setMatrixElement(2,2,temp);
	//calc dF[2][3]
	temp = 2*this->q_value[3];
	this->jacobiMatrix.setMatrixElement(2,3,temp);
	//set dF[2][4]
	this->jacobiMatrix.setMatrixElement(2,4,this->fx[2]);


	//calc dF[3][0]
	temp = 2*this->magValue[0] * this->q_value[0]
         - 2*this->magValue[1] * this->q_value[3]
         + 2*this->magValue[2] * this->q_value[2];
	this->jacobiMatrix.setMatrixElement(3,0,temp);

	//calc dF[3][1]
	temp = 2*this->magValue[0] * this->q_value[1]
	     + 2*this->magValue[1] * this->q_value[2]
	     + 2*this->magValue[2] * this->q_value[3];
	this->jacobiMatrix.setMatrixElement(3,1,temp);

	//calc dF[3][2]
	temp = (-2)*this->magValue[0] * this->q_value[2]
		 + 2*this->magValue[1] * this->q_value[1]
		 + 2*this->magValue[2] * this->q_value[0];
	this->jacobiMatrix.setMatrixElement(3,2,temp);

	//calc dF[3][3]
	temp = (-2)*this->magValue[0] * this->q_value[3]
		 - 2*this->magValue[1] * this->q_value[0]
		 + 2*this->magValue[2] * this->q_value[1];
	this->jacobiMatrix.setMatrixElement(3,3,temp);

	//set dF[3][4]
	this->jacobiMatrix.setMatrixElement(3,4,this->fx[3]);

};


void StaticPosCalc::calcRootsJacobiMatrix()
{
	this->jacobiMatrix.gaussElim_ColmnPrin();
	this->jacobiMatrix.calcRoots();
};
//private
void StaticPosCalc::initDefaultValue()
{
	//默认4x5矩阵
	//this->jacobiMatrix = MyMatrix(4,5);

	this->jacobiMatrix = MyMatrix();
	this->q_deltaValue[0] = 999;
	this->q_deltaValue[1] = 999;
	this->q_deltaValue[2] = 999;
	this->q_deltaValue[3] = 999;

	//默认q初值为1，0，0，0
	this->q_value[0]  = 1;
	this->q_value[1]  = 0;
	this->q_value[2]  = 0;
	this->q_value[3]  = 0;

	//默认初始化状态东北天地理坐标
	this->magValue[0] = 0;
	this->magValue[1] = 0;
	this->magValue[2] = 0;

	this->accValue[0] = 0;
	this->accValue[1] = 0;
	this->accValue[2] = -1;

	this->eulerAngel_pitch = 0;
	this->eulerAngel_roll = 0;
	this->eulerAngel_yaw = 0;


	this->fx[0] =0;
	this->fx[1] =0;
	this->fx[2] =-1;
	this->fx[3] =0;

	//默认精度0.001
	this->precision = 0.001;
};

void StaticPosCalc::initDefaultValue(float accValue[3], float magValue[3])
{
	this->initDefaultValue();
	this->magValue[0] = magValue[0];
	this->magValue[1] = magValue[1];
	this->magValue[2] = magValue[2];

	this->accValue[0] = accValue[0];
	this->accValue[1] = accValue[1];
	this->accValue[2] = accValue[2];
};

void StaticPosCalc::setSensorValue(float accValue[3], float magValue[3])
{
	this->magValue[0] = magValue[0];
	this->magValue[1] = magValue[1];
	this->magValue[2] = magValue[2];

	this->accValue[0] = accValue[0];
	this->accValue[1] = accValue[1];
	this->accValue[2] = accValue[2];
};

void StaticPosCalc::setInitQ(float qInit[4])
{
	this->q_value[0] = qInit[0];
	this->q_value[1] = qInit[1];
	this->q_value[2] = qInit[2];
	this->q_value[3] = qInit[3];
};

void StaticPosCalc::setPrecision(float prec)
{
	this->precision = prec;
};

void StaticPosCalc::test()
{
	//this->calcFX();
	//this->generateJacobiMatrix();
	//this->printJacobiMatrix();
	//this->calcRootsJacobiMatrix();
	//this->updateDeltaQFromJacobiRoot();

	this->calcStaticPos();
	//this->printCurrentQaut();
	this->test_Euler();
	this->calcMagInclinationByQaut();
	//this->printMagInclination();
};

void StaticPosCalc::QautToEuler()
{
	double check = this->q_value[0]*this->q_value[2] + this->q_value[1]*this->q_value[3];
	if(check > 0.499) //Pitch=-PI/2
	{
		this->eulerAngel_pitch = -PI/2;
		this->eulerAngel_roll = 0;
		this->eulerAngel_yaw = (-2)*atan2(q_value[1], q_value[0]);

	}

	if(check < -0.499) //Pitch=PI/2
	{
		this->eulerAngel_pitch = PI/2;
		this->eulerAngel_roll = 0;
		this->eulerAngel_yaw = 2*atan2(q_value[1], q_value[0]);
	}

	double part1 = 2 * (this->q_value[2] * this->q_value[3] - this->q_value[0]*this->q_value[1]);
	//double part2 = 2 * (this->q_value[0]*this->q_value[0] - this->q_value[1]*this->q_value[1] - this->q_value[2]*this->q_value[2] + this->q_value[3]*this->q_value[3]);
	double part2 = this->q_value[0]*this->q_value[0] - this->q_value[1]*this->q_value[1] - this->q_value[2]*this->q_value[2] + this->q_value[3]*this->q_value[3];
	this->eulerAngel_roll = atan2(part1, part2);
	//this->eulerAngel_roll = this->eulerAngel_roll * 180 / PI;

	//asin(-2*(q0q2+q1q3))
	part1 = this->q_value[0]*this->q_value[2];
	part2 = this->q_value[1]*this->q_value[3];
	this->eulerAngel_pitch = asin((-2) * (part1 + part2));
	//this->eulerAngel_pitch = this->eulerAngel_pitch * 180 / PI;
	//
	part1 = 2 * (this->q_value[1] * this->q_value[2] - this->q_value[0]*this->q_value[3]);
	//part2 = 2 * (this->q_value[0]*this->q_value[0] + this->q_value[1]*this->q_value[1] - this->q_value[2]*this->q_value[2] - this->q_value[3]*this->q_value[3]);
	part2 = this->q_value[0]*this->q_value[0] + this->q_value[1]*this->q_value[1] - this->q_value[2]*this->q_value[2] - this->q_value[3]*this->q_value[3];
	this->eulerAngel_yaw = atan2(part1, part2);
	//this->eulerAngel_yaw = this->eulerAngel_yaw * 180 / PI;
};
/*
void StaticPosCalc::printEulerAngel()
{
	cout << "Roll: " << this->eulerAngel_roll* 180 / PI << endl;
	cout << "Pitch: " << this->eulerAngel_pitch* 180 / PI << endl;
	cout << "Yaw: " << this->eulerAngel_yaw* 180 / PI << endl;
};
*/
void StaticPosCalc::test_Euler()
{
	this->QautToEuler();
	//this->printEulerAngel();
};

void StaticPosCalc::calcStaticPos()
{
	//初始化Q值，从初始值开始，迭代计算姿态
	while (!this->isPrcisionReached())
	{
		//this->printCurrentQaut();
		//this->printCurrentDeltaQ();

		this->calcFX();
		this->generateJacobiMatrix();
		this->calcRootsJacobiMatrix();

		this->updateDeltaQFromJacobiRoot();
		this->q_value[0] += this->q_deltaValue[0];
		this->q_value[1] += this->q_deltaValue[1];
		this->q_value[2] += this->q_deltaValue[2];
		this->q_value[3] += this->q_deltaValue[3];
	}

};

bool StaticPosCalc::isPrcisionReached()
{
	if(		fabs(this->q_deltaValue[0]) < this->precision &&
			fabs(this->q_deltaValue[1]) < this->precision &&
			fabs(this->q_deltaValue[2]) < this->precision &&
			fabs(this->q_deltaValue[3]) < this->precision)
		return true;
	else
		return false;
};

void StaticPosCalc::updateDeltaQFromJacobiRoot()
{
	float* roots = this->jacobiMatrix.getRoots();

	this->q_deltaValue[0] = *roots;
	this->q_deltaValue[1] = *(roots+1);
	this->q_deltaValue[2] = *(roots+2);
	this->q_deltaValue[3] = *(roots+3);
};
/*
void StaticPosCalc::printCurrentQaut()
{
	cout<<"Current Qaut is:" << endl;
	cout<< "Q0:" << this->q_value[0] << " Q1:" << this->q_value[1] << " Q2:" << this->q_value[2]<< " Q3:" << this->q_value[3] << endl;
};

void StaticPosCalc::printCurrentDeltaQ()
{
	cout << "Current Delta Q is:" << endl;
	cout<< "dq0:" << this->q_deltaValue[0] << "dq1:" << this->q_deltaValue[1] << " dq2:" << this->q_deltaValue[2]<< " dq3:" << this->q_deltaValue[3] << endl;
};
*/

void StaticPosCalc::calcMagInclinationByQaut()
{
	this->Mny = this->calcMag_eq1();
	this->Mnz = this->calcMag_eq2();
};

/*
void StaticPosCalc::printMagInclination()
{

	double angel = atan((0-this->Mnz)/this->Mny);
	angel = angel*180/PI;
	cout << "Mny:" << this->Mny << endl;
	cout << "Mnz:" << this->Mnz << endl;
	cout << "Mag Inclination:" << angel << endl;

};
*/
