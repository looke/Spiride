#ifndef _Kalman_h_
#define _Kalman_h_

#define STATIC_STATE 0
#define HUGE_ACCELERATE_STATE 1
//#define SMALL_ACCELERATE_STATE 100002
#define ROTATE_STATE 2

class Kalman {
public:
    Kalman();
    
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate_1, float newRate_2, float dt);

    void setAngle(float angle); // Used to set angle, this should be set as the starting angle
    float getRate(); // Return the unbiased rate
    float getBias(); // Return the bias
    
    float getK0(); // Return the K[0]
    float getK1(); // Return the K[1]
    /* These are used to tune the Kalman filter */
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

    void setBasicBias(float bBias);
    void setBasicAngle(float bAngle);
    float getBasicAngle();
    int getCurrentState();
    int getOldState();
private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float K[2]; // Kalman gain - This is a 2x1 vector
    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    
    float basicBias; //the gyro bias calculated by the calibration process.
    float basicAngle;//the standard angle for bias carlibration.
    int currentState;
    int oldState;
    int staticCount;
    float oldAccAngle;
    
    ///最近3次的Gyro Rate值
    float recent_rate;
    float first_rotate_rateplus;
    ///高值偏移的次数，用来判断是否进入Rotate模式
    int bigDriftTime;
};

#endif