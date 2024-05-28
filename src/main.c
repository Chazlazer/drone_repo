/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */
#include <stdio.h>
#include <robotcontrol.h>
#include <math.h>
#include <sys/time.h> // For gettimeofday function
#include <ncurses.h>
#include <pthread.h>

#define KEY_Q 113
#define KEY_W 119
#define KEY_E 101
#define KEY_A 97
#define KEY_S 115
#define KEY_D 100
#define KEY_Z 122
#define KEY_X 120
#define KEY_T 116

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

float constrain(float val, float max, float min);
void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
void read_sensors();
unsigned long micros();
void getDesState();
void armedStatus();
int wakeup_esc();
void controlMixer();
void controlANGLE();
void loopBlink();
void *  threadFunction(void * arg);
void loopRate(int freq);

// Global Variables
static rc_mpu_data_t mpu_data;

// ESC
int freq = 50;
int min_us = RC_ESC_DEFAULT_MIN_US;
int max_us = RC_ESC_DEFAULT_MAX_US;
float wakeup_val = -0.1;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//IMU:

// double x = 0, y = 0, z = 0;
float x = 1.0, y = 1.0;
float time_since_update =0.0;
int left_right = 0;
int fwd_bkwd = 0;
float vx = 0, vy = 0, vz = 0;
float ax = 0, ay = 0, az = 0;

// Pitch Roll Yaw
float q = 0, p = 0, r = 0;
float phi_dot =0, theta_dot = 0, psi_dot = 0;
float phi_gyro = 0, theta_gyro = 0, psi_gyro = 0;

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

float phi_accel = 0, theta_accel = 0, psi_accel = 0;
float phi = 0, theta = 0, psi = 0;

// Magnetometer Readings
float mx = 0, my = 0, mz = 0;

float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.17;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

float prev_time = 0;
float current_time = 0;
float dt = 0;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

//Flight status
bool armedFly = false;

int value = 0;
int clear_counter = 0;
int speed_control = 1000;
int speed_roll = 1500;
int speed_pitch = 1500;
int roll_angle = 1500;
int pitch_angle = 1500;
int exit = 0;

float damping_dt = 0;



int main() {
        // make sure another instance isn't running
        if(rc_kill_existing_process(2.0)<-2) return -1;
        // start signal handler so we can exit cleanly
        if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
        }

        rc_make_pid_file();
        // Keep looping until state changes to EXITING

        // Configure MPU
        rc_mpu_config_t mpu_config = rc_mpu_default_config();
        // mpu_config.dmp_sample_rate = 200;
        mpu_config.orient = ORIENTATION_Y_UP;
        mpu_config.enable_magnetometer = 1;

        
        // if gyro isn't calibrated, run the calibration routine
        if(!rc_mpu_is_gyro_calibrated()){
                printf("Gyro not calibrated, automatically starting calibration routine\n");
                printf("Let your MiP sit still on a firm surface\n");
                rc_mpu_calibrate_gyro_routine(mpu_config);
        }

        // // start mpu
        if(rc_mpu_initialize(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                return -1;
        }

        
        // Activate Motors
        // Initalize PRU
        if(rc_servo_init()) return -1;
        if(rc_servo_set_esc_range(min_us,max_us)) return -1;
        rc_servo_power_rail_en(0);

        // Wake up ESC
        wakeup_esc();

        printf("Controls\n");
        printf("Throttle: z-up,   x-down\n");
        printf("Pitch   : w-up,   s-down\n");
        printf("Yaw     : a-left, d-right\n");
        printf("Roll    : q-left, e-right\n");
        printf("t to quit\n");

        //Set radio channels to default (safe) values before entering main loop
        channel_1_pwm = channel_1_fs;
        channel_2_pwm = channel_2_fs;
        channel_3_pwm = channel_3_fs;
        channel_4_pwm = channel_4_fs;
        channel_5_pwm = channel_5_fs;
        channel_6_pwm = channel_6_fs;


        current_time = micros();
        rc_set_state(RUNNING);

        // Create pthread

        pthread_t keyboard_thread;
        pthread_create(&keyboard_thread, NULL, threadFunction, NULL);

        // Sleep 2 seconds
        rc_usleep(200000);
        while(rc_get_state()!=EXITING){
                // Keep track of how much time has elasped since last loop

                prev_time = current_time;
                current_time = micros();
                dt = (current_time - prev_time) / 1000000.0;;

                loopBlink();

                // Measure Sensors
                read_sensors();

                Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
                // printw("\nPitch: %5.2f Roll: %5.2f | dt %2.5f",pitch_IMU, roll_IMU,dt);

                getDesState();

                controlANGLE();

                controlMixer();
                // printf("\nThrottle: %d", speed_control);
                // printw("\nM1:%f | M2: %f \nM3%f | M4%f", m1_command_scaled, m2_command_scaled,m3_command_scaled,m4_command_scaled);
                // Send Motor Command
                rc_servo_send_esc_pulse_normalized(1,m1_command_scaled);
                rc_servo_send_esc_pulse_normalized(2,m2_command_scaled);
                rc_servo_send_esc_pulse_normalized(3,m3_command_scaled);
                rc_servo_send_esc_pulse_normalized(4,m4_command_scaled);
                // fflush(stdout); // Ensure the output is displayed immediately
                // always sleep at some point
                loopRate(2000);
                //rc_usleep(500);
        }
        pthread_join(&threadFunction, NULL);
        //Exit Curses Terminal
        endwin();
        // turn off LEDs and close file descriptors
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        return 0;
}

void loopBlink(){
    if(rc_get_state()==RUNNING){
        rc_led_set(RC_LED_GREEN, 1);
        rc_led_set(RC_LED_RED, 0);
        }
    else{
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 1);
    }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  
  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = 1.0/sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  pitch_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  roll_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

float constrain(float val, float min, float max){
        if( val > max){
                return max;
        }
        else if(val < min){
                return min;
        }
        else{
                return val;
        }
}

void read_sensors(){
        // read sensor data
        if(rc_mpu_read_accel(&mpu_data)<0){
                printf("read accel data failed\n");
        }
        if(rc_mpu_read_gyro(&mpu_data)<0){
                printf("read gyro data failed\n");
        }
        if(rc_mpu_read_mag(&mpu_data)<0){
                printf("read mag data failed\n");
        }

        // Measure Acceleration
        AccX = (1.0 - B_accel) * AccX_prev + B_accel * mpu_data.accel[0]; // ax
        AccY = (1.0 - B_accel) * AccY_prev + B_accel * mpu_data.accel[1]; // ay
        AccZ = (1.0 - B_accel) * AccZ_prev + B_accel * mpu_data.accel[2]; // az
        AccX_prev = AccX;
        AccY_prev = AccY;
        AccZ_prev = AccZ;

        // Measure Gyro
        GyroX = (1.0 - B_gyro) * GyroX_prev + B_gyro * mpu_data.gyro[0]; // Pitch rate
        GyroY = (1.0 - B_gyro) * GyroY_prev + B_gyro * mpu_data.gyro[1]; // Roll rate
        GyroZ = (1.0 - B_gyro) * GyroZ_prev + B_gyro * mpu_data.gyro[2]; // Yaw rate
        GyroX_prev = GyroX;
        GyroY_prev = GyroY;
        GyroZ_prev = GyroZ;


        // Measure Magnetometer
        MagX = (1.0 - B_mag) * MagX_prev + B_mag * mpu_data.mag[0];
        MagY = (1.0 - B_mag) * MagY_prev + B_mag * mpu_data.mag[1];
        MagZ = (1.0 - B_mag) * MagZ_prev + B_mag * mpu_data.mag[2];
        MagX_prev = MagX;
        MagY_prev = MagY;
        MagZ_prev = MagZ;
}

unsigned long micros() {
    // Returns the time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

void getDesState() {
    //DESCRIPTION: Normalizes desired control values to appropriate values
    /*
    * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
    * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
    * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
    * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
    * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
    */
    channel_1_pwm = speed_control;
    channel_2_pwm = roll_angle;
    channel_3_pwm = pitch_angle;
    channel_4_pwm = 1500;

    thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
    roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
    yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
    roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
    pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
    yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
    pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}


void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

int wakeup_esc(){
    printf("waking ESC up from idle for 3 seconds\n");
    for(int i =0;i<=freq*3.0;i++){
        if(rc_servo_send_esc_pulse_normalized(1,wakeup_val)==-1) return -1;
        if(rc_servo_send_esc_pulse_normalized(2,wakeup_val)==-1) return -1;
        if(rc_servo_send_esc_pulse_normalized(3,wakeup_val)==-1) return -1;
        if(rc_servo_send_esc_pulse_normalized(4,wakeup_val)==-1) return -1;
        if(rc_servo_send_esc_pulse_normalized(8,wakeup_val)==-1) return -1;
        rc_usleep(1000000/freq);
        }
    printf("done with wakeup period\n");
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
   
  //Quad mixing - EXAMPLE
  m1_command_scaled = constrain(thro_des - pitch_PID + roll_PID + yaw_PID, 0.0, 1.0); //Front Left
  m2_command_scaled = constrain(thro_des - pitch_PID - roll_PID - yaw_PID, 0.0, 1.0); //Front Right
  m3_command_scaled = constrain(thro_des + pitch_PID - roll_PID + yaw_PID, 0.0, 1.0); //Back Right
  m4_command_scaled = constrain(thro_des + pitch_PID + roll_PID - yaw_PID, 0.0, 1.0); //Back Left
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
 
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void keyboard_control(int key) {
        switch (getch()) {
                case KEY_Q:
                        refresh();
                        break;
                case KEY_W:
                        speed_pitch = 2000;
                        fwd_bkwd = 1;
                        y = 1.0;
                        refresh();
                        break;
                case KEY_E:
                        refresh();
                        break;
                case KEY_A:
                        speed_roll = 2000;
                        left_right = 2;
                        x = 1.0;
                        refresh();
                        break;
                case KEY_S:
                        speed_pitch = 2000;
                        fwd_bkwd = 2;
                        y = 1.0;
                        refresh();
                        break;
                case KEY_D:
                        speed_roll = 2000;
                        left_right = 1;
                        x = 1.0;
                        refresh();
                        break;
                case KEY_Z:
                        speed_control = speed_control + 50;
                        if (speed_control > 2000) {
                        speed_control = 2000;
                        }
                        refresh();
                        break;
                case KEY_X:
                        speed_control = speed_control - 50;
                        if (speed_control < 1000) {
                        speed_control = 1000;
                        }
                        refresh();
                        break;
                case KEY_T:
                        exit = 1;
                        refresh();
                        break;
                }
                clear_counter = clear_counter + 1;
                if (clear_counter == 1) {
                        clear_counter = 0;
                        clear();
                }
                value = key;
                refresh();
    
        if(time_since_update > 0.01){
        speed_roll = (float)speed_roll * x;
        time_since_update = 0.0;
        x = x - 0.1;
        if(speed_roll < 0){
                x = 1.0;
        }
        if(left_right == 1){
                roll_angle = (int)speed_roll;
        }
        if(left_right == 2){
                roll_angle = (int)-speed_roll;
        }
        speed_pitch = (float)speed_pitch * y;
        y = y - 0.1;
        if(speed_pitch < 0){
                y = 1.0;
        }
        if(fwd_bkwd == 1){
                pitch_angle = (int)speed_pitch;
        }
        if(fwd_bkwd == 2){
                pitch_angle = (int)-speed_pitch;
        }
    }
}

void *  threadFunction(void * arg){
    initscr();
    cbreak();
    noecho();
    timeout(20);

    char key = getch();
    value = key;
    while(1){
        keyboard_control(value);
        printw("\nPitch: %5.2f Roll: %5.2f Yaw: %5.2f | dt %2.5f",pitch_IMU, roll_IMU, yaw_IMU ,dt);
        printw("\nPitchDes: %5.2f RollDes: %5.2f YawDes: %5.2f Throttle: %2.5f | dt %2.5f",pitch_des, roll_des, yaw_des, thro_des, dt);
        printw("\nPitchPID: %5.2f RollPID: %5.2f YawPID: %5.2f ",pitch_PID, roll_PID, yaw_PID);
        printw("\nThrottle: %d", speed_control);
        printw("\nM1: %f | M2: %f \nM3: %f | M4 %f", m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled);

    }
    return NULL;
}
