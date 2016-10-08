/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 Contact information
 -------------------
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include "SpirideKalman.h"

Kalman::Kalman() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f;  //Original
    //Q_angle = 0.0001f;
    //Q_bias = 0.003f; //Original
		Q_bias = 0.7f;
    R_measure = 0.03f; //original value
		//R_measure = 10.7f;
    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias
    
    basicBias = 0.0f; //reset basicBias
    basicAngle = 0.0f; //reset basicAngle
    currentState = HUGE_ACCELERATE_STATE;
    oldState = HUGE_ACCELERATE_STATE;
    staticCount = 0; //��¼�������ھ�ֹ̬�Ĵ����������ж��Ƿ����Staticģʽ
    oldAccAngle = 0.0f;
    
    float K[2]; // Kalman gain - This is a 2x1 vector
    
    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
    
    ///���3�ε�Gyro Rateֵ
    recent_rate = 0.0;
    first_rotate_rateplus = 0.0;
    ///��ֵƫ�ƵĴ����������ж��Ƿ����Rotateģʽ
    bigDriftTime = 0;
    
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
//newAngle: ACC sensor angle
//newRate:  Gyro Sensor rate
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    
    //Multi-State Kalman
    currentState = HUGE_ACCELERATE_STATE;
    //Calc the diff between newAngle(From ACC sensor) and the old pitch
    float accDiff = newAngle - angle;
    
    //calc the diff between newAngle and the old accAngle
    float staticDiff = newAngle - oldAccAngle;
    
    //------------���д����ж��Ƿ����Static״̬ ----------------------------
      //----���ȼ�鵱ǰaccȡֵ��֮ǰһ��accȡֵ�ж����죬�����ֵ�ľ���ֵС�� 0.015 �򽫾�̬���������������������
    if(staticDiff>-0.015 && staticDiff<0.015)
    {
    	staticCount++;
    }
    else
    {
    	staticCount = 0;
    }
    
    //----���accȡֵ��֮ǰһ���Ż�ֵ�Ĳ��죬�����ֵ�ľ���ֵС�� 0.07 ����Gyro��ȡֵ��[-1.0, 0.7]���䣬���Ҿ�̬������ȡֵ����5�������static״̬
    if(accDiff>-0.07 && accDiff<0.07 && newRate>-1.1 && newRate<1.0)
    {
    	if(staticCount >=5)
    	{
    		currentState = STATIC_STATE;
    	}
    }
    
      //----��龲̬�������Ƿ���ڵ���10��������ڵ���10������֮ǰ��״̬����static״̬�������static״̬
      //----�˴�����������Ϊ�˷�ֹԤ���������accDiff���󣬵���ϵͳ������accDiff�޷�����static״̬��
      //----�ʶ�����һ�ֳ���:����10��staticDiff����С��ʱ��ֱ�ӽ���static״̬��
    if(oldState!=STATIC_STATE && staticCount>=8)
    {
    	currentState = STATIC_STATE;
    }
    if(oldState==STATIC_STATE && staticCount>0)
    {
    	currentState = STATIC_STATE;
    }
        
    //------------�ж��Ƿ����Static״̬�������----------------------------
    
    
    //--------------���д����ж��Ƿ����rotate״̬ -----------------------
    //ԭ���ж�Gyroȡֵ�Ƿ���������ѭ����������̬Ư������[-1.1, 1.0]
    //���Gyroȡֵ����3�ζ�������̬Ư�����䣬�����Rotate״̬������ǰ3�νǶȱ仯����ϲ������ս����
    //recent_rate��¼���3�ε�Rateȡֵ֮�ͣ��Ա��������ʹ�á�
    //bigDriftTime ��¼Gyroȡֵ����Ư������Ĵ�����ÿ��Gyroȡֵ����Ư������ʱ��bigDriftTime��Ҫ����
      //---������Ҫ�ж�Gyro Rateȡֵ�Ƿ���[-1.1, 1.0]���䣬������ڸ����䣬��
    if(newRate<=-1.1 || newRate>=1.0) 
    {
    	bigDriftTime++;
    	recent_rate += newRate;
    }
    else
    {
    	bigDriftTime=0;
    	recent_rate=0.0;
    }
    
    if(bigDriftTime > 0 && oldState==ROTATE_STATE) //����ǰһ��ѭ���Ѿ���Rotate״̬����bingDriftTime>0�ͼ�������Rotate״̬
    {
    	currentState = ROTATE_STATE;
    	bigDriftTime = 0;
    	recent_rate=0.0;
    }
    
    if(bigDriftTime >=3 && oldState!=ROTATE_STATE) //����ǰһ��ѭ������Rotate״̬����bingDriftTime>=3ʱ����Rotate״̬
    {
    	currentState = ROTATE_STATE;
    	bigDriftTime = 0;
    	first_rotate_rateplus = recent_rate;
    	recent_rate=0.0;
    }
    
    
    //--------------�ж��Ƿ����rotate״̬�������-----------------------
    
    
    /*
    float y = -1.0;
    //First: check boundary condition
    if(newAngle >= 27 || newAngle<=-27)
    {
    	//Normal street slope is smaller than 8%, Baldwin Street has 35% slope.
    	//If ACC sensor get the angel greater than 30 degree (60%), it means the result must be influenced greatly by other acceleration.
    	//In this case, ACC sensor does not take into the basic Kalman filter.
    	currentState = HUGE_ACCELERATE_STATE;
    	y = 0; //set y=0 means basic kalman filter will completely use Gyro sensor to calc the pitch
    }
    else
    {
    	//Calc the diff between newAngel(From ACC sensor) and the old pitch
    	float diff = newAngle - angle;
    	
    	if(diff<=-10 || diff >= 10)
    	{
    		//Dt 0.01ms during this time, normal bicycle or motorcycle hardly can gain a slope changing which greater than 20 degree.
    		//The distance between two wheels is 109cm. tan(20) = 0.36. It means during 0.01ms the hight of wheel must change 40cm which is quite impossible in normal road.
    		//The distance between two wheels is 109cm. tan(10) = 0.17. It means during 0.01ms the hight of wheel must change 19cm which is quite impossible in normal road.
    		currentState = HUGE_ACCELERATE_STATE;
    		y = 0; //set y=0 means basic kalman filter will completely use Gyro sensor to calc the pitch
    	}
    }
    */
    float y = 0.0f;
    float rate_afterBias = 0.0f;
    float angle_temp = 0.0f;
    
    if(currentState == STATIC_STATE) //in STATIC_STATE case : update the basicAngle using newACCAngle
    {
    	if(oldState != STATIC_STATE) // if state change to static, basicAngle must be update;
    	{
    		basicAngle = newAngle;
    	}
    	if(accDiff>-0.015 && accDiff<0.015)
    	{
    		basicAngle = newAngle;
    	}
    	rate_afterBias = newRate - bias;
    	
    	angle_temp = angle + dt * rate_afterBias;
    	y = basicAngle - angle_temp;
    }
    else if(currentState == HUGE_ACCELERATE_STATE) //in HUGE_ACCELERATE_STATE case : basicAngle is not change
    {
    	rate_afterBias = newRate - bias;
    	if(rate_afterBias <-1.4 || rate_afterBias >1.2)
    	{
    		basicAngle = basicAngle + dt * rate_afterBias;
    	}
    	angle_temp = angle + dt * rate_afterBias;
    	y = basicAngle - angle_temp;
    }
    else if(currentState == ROTATE_STATE) //in ROTATE_STATE case : update basicAngle with new calc pitch
    {
    	if(oldState!=ROTATE_STATE) //�����Ǵ�����״̬����Rotate״̬����Ҫ�ڼ���Ƕ�ֵʱ���벹��
    	{
    		angle = angle + dt*(first_rotate_rateplus - newRate); //����ǰ����û�м����Rate
    	}
    	//rate_afterBias = newRate - basicBias;
    	rate_afterBias = newRate; //ROTATE_STATE: no bias for rate
    	basicAngle = angle + dt * rate_afterBias;
    	rate_afterBias = newRate - bias;
    	angle_temp = angle + dt * rate_afterBias;
    	y = basicAngle - angle_temp;
    }
    
    /* Step 1 */
    //rate = newRate - bias;
    //angle += dt * rate;
    
    

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    //float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    //float y = newAngle - angle; // Angle difference
    
    //if(y!=0) //This means the boundary condition has passed.
    //{
    //	y = newAngle - angle_temp;
    //}
    
    //different y has different ratio for new bias and new angle
    /*
    if(y<=0.5 && y >= -0.5)  //y in [-0.5, 0.5] use the default Kalman algorithm to calc bias and angle
    {
    	//angle += K[0] * y;
    	angle = angle_temp + K[0] * y; 
      bias += K[1] * y;
      rate = rate_afterBias;
    }
    else // y in (-xxx, -0.5) and (0.5, +xxx) use the Multi-State Kalman algorithm
    {
    	rate = newRate - basicBias;
    	angle = angle + dt * rate;

    }
    */
    
    /*
    if(y>=10 || y<=-10) //
    {
    	 angle += K[0] * y/80;
       bias += K[1] * y/80;
    }
    else if((y>=5 && y<10) || (y<=-5 && y>-10))
    {
    	 angle += K[0] * y/50;
    	 bias += K[1] * y/50;
    }
    else if((y>=2 && y<5) || (y<=-2 && y>-5))
    {
    	 angle += K[0] * y/20;
    	 bias += K[1] * y/20;
    }
    else if((y>=1 && y<2) || (y<=-1 && y>-2))
    {
    	 angle += K[0] * y/10;
    	 bias += K[1] * y/10;
    }
    else if((y>=0 && y<1) || (y<=0 && y>-1))
    {
    	 angle += K[0] * y/7;
    	 bias += K[1] * y/7;
    }
    */
    
    /* Step 6 */
    if(currentState == STATIC_STATE) //in STATIC_STATE case : update the pitch based on newACCAngle
    {
    	angle = newAngle + K[0] * y;
    	bias = bias + K[1] * y;
    }
    angle = angle_temp + K[0] * y;
    bias = bias + K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    oldState = currentState;
    oldAccAngle = newAngle;
    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate
float Kalman::getBias() { return this->bias; };
float Kalman::getK0() { return this->K[0]; };
float Kalman::getK1() { return this->K[1]; };	
	
/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

void Kalman::setBasicBias(float bBias) { this->basicBias = bBias; };
void Kalman::setBasicAngle(float bAngle) { this->basicAngle = bAngle; this->oldAccAngle = bAngle;};
int Kalman::getCurrentState() { return this->currentState; };
int Kalman::getOldState() { return this->oldState; };
float Kalman::getBasicAngle() { return this->basicAngle; };