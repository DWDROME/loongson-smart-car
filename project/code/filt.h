
#ifndef FILT_H_
#define FILT_H_


// extern float angle, angle_dot;
// void Kalman_Filter(float Accel,float Gyro);
// void Yijielvbo(float angle_m, float gyro_m);
// void Erjielvbo(float angle_m, float gyro_m);


typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float pitch;
    float roll;
    float yaw;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;



extern double Lamp_fuyang;
typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
    float AXdata;
    float AYdata;
    float AZdata;
} gyro_param_t;


//extern float icm_ay;
//extern float icm_sy;
extern icm_param_t icm_data;
extern gyro_param_t GyroOffset;
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
float fast_sqrt(float x);
void gyroOffset_init(void);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getValues();
void ICM_getEulerianAngles(void);
void LPF_1(float hz,float time,float in,float *out);
void huandao_yaw_correct(void);
//void getEulerianAngles(void);
// void imu660_init(void);
extern float param_Kp ;
extern float param_Ki ;
#endif 
