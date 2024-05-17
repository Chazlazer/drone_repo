/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */
#include <stdio.h>
#include <robotcontrol.h>
#include <math.h>

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static void __attitude_controller(void);

#define Nx 2
#define Ny 1
#define Nu 1
#define dt 0.05

// Global Variables
static rc_mpu_data_t mpu_data;

// XYZ
double x = 0, y = 0, z = 0;
double vx = 0, vy = 0, vz = 0;
double ax = 0, ay = 0, az = 0;

// Pitch Roll Yaw
double q = 0, p = 0, r = 0;
double phi_dot =0, theta_dot = 0, psi_dot = 0;
double phi_gyro = 0, theta_gyro = 0, psi_gyro = 0;

double phi_accel = 0, theta_accel = 0, psi_accel = 0;
double phi = 0, theta = 0, psi = 0;

int main()
{
        // make sure another instance isn't running
        if(rc_kill_existing_process(2.0)<-2) return -1;
        // start signal handler so we can exit cleanly
        if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
        }

        rc_make_pid_file();
        // Keep looping until state changes to EXITING

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

        // Setup Kalman Filter

        // Declare Variables
        rc_kalman_t kf = RC_KALMAN_INITIALIZER;
        rc_matrix_t A  = RC_MATRIX_INITIALIZER;
        rc_matrix_t B  = RC_MATRIX_INITIALIZER;
        rc_matrix_t H  = RC_MATRIX_INITIALIZER;
        rc_matrix_t Q  = RC_MATRIX_INITIALIZER;
        rc_matrix_t R  = RC_MATRIX_INITIALIZER;
        rc_matrix_t Pi = RC_MATRIX_INITIALIZER;
        rc_vector_t u  = RC_VECTOR_INITIALIZER;
        // rc_vector_t y  = RC_VECTOR_INITIALIZER;
        
        rc_matrix_t J  = RC_MATRIX_INITIALIZER;

        // Allocate memory for the system
        rc_matrix_zeros(&A, Nx, Nx);
        rc_matrix_zeros(&B, Nx, Nu);
        rc_matrix_zeros(&H, Ny, Nx);
        rc_matrix_zeros(&Q, Nx, Nx);
        rc_matrix_zeros(&R, Ny, Ny);
        rc_matrix_zeros(&Pi, Nx, Nx);
        rc_vector_zeros(&u, Nu);
        
        rc_matrix_zeros(&J, 3,3);


        // Define system
        A.d[0][0] = 1;
        A.d[0][1] = dt;
        A.d[1][0] = 0;
        A.d[1][1] = 1;
        B.d[0][0] = 0.5*dt*dt;
        B.d[0][1] = dt;
        H.d[0][0] = 1;
        H.d[0][1] = 0;

        // covariange matricies

        Q.d[0][0] = 0.0;
        Q.d[1][1] = 0.0;

        R.d[0][0] = 0.00009;
        // Inital P

        Pi.d[0][0] = 0.0001;
        Pi.d[1][1] = 0.0001;

        if(rc_kalman_alloc_ekf(&kf,Q,R,Pi) == -1) return -1;

        // if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
        //         fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
        //         rc_led_blink(RC_LED_RED, 5, 5);
        //         return -1;
        // }

        // rc_mpu_set_dmp_callback(&__attitude_controller);

        rc_set_state(RUNNING);
        while(rc_get_state()!=EXITING){
                // do things based on the state

                // read sensor data
                if(rc_mpu_read_accel(&mpu_data)<0){
                        printf("read accel data failed\n");
                }
                if(rc_mpu_read_gyro(&mpu_data)<0){
                        printf("read gyro data failed\n");
                }

                // Measure Sensors

                // Measure Acceleration
                ax = mpu_data.accel[0]; // ax
                ay = mpu_data.accel[1]; // ay
                az = mpu_data.accel[2]; // az

                // Solve for XYZ

                vx = vx + ax*dt;
                vy = vy + ay*dt;
                vz = vz + az*dt;

                x = x + vx*dt + 1/2 *ax*dt*dt;
                y = y + vy*dt + 1/2 *ay*dt*dt;
                z = z + vz*dt + 1/2 *az*dt*dt;

                q = mpu_data.gyro[0]; // Pitch rate
                p = mpu_data.gyro[1]; // Roll rate
                r = mpu_data.gyro[2]; // Yaw rate
        
                phi_dot = p + q *sin(phi) * tan(theta) + r* cos(phi)*tan(theta);
                theta_dot = q *cos(phi) - r*sin(phi);
                psi_dot = q * (sin(phi)/cos(theta)) + r * (cos(phi)/cos(theta));
                
                phi_gyro = phi_gyro + phi_dot * dt;
                theta_gyro = theta_gyro + theta_dot * dt;
                psi_gyro = psi_gyro + psi_dot * dt;

                J.d[0][0] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
                J.d[0][1] = q*(sec(theta)*sec(theta) +1)*sin(phi) + r*(sec(theta)*sec(theta) + 1)*cos(phi);
                J.d[0][2] = 0;
                J.d[1][0] = -q*sin(phi) - r*cos(phi);
                J.d[1][1] = 0;
                J.d[1][2] = 0;
                J.d[2][0] = q*cos(phi)/cos(theta) - r*sin(phi)/cos(theta);
                J.d[2][1] = q*sin(phi)*sin(theta)/(cos(theta)*cos(theta)) + r*sin(theta)*cos(phi)/(cos(theta)*cos(theta));
                J.d[2][2] = 0;


                // ax = mpu_data.accel[0]*MS2_TO_G;
                // u.d[0] = ax
                // v_pos = v_pos * ax*dt;
                // x_pos = x_pos + v_pos * dt + 0.5*ax*dt*dt;
                // printf("%6.4f , %6.4f, |  %6.4f , %6.4f | %6.4f \r\n", kf.x_est.d[0],x_pos, v_pos,kf.x_est.d[1], ax);                
                printf("%6.2f , %6.2f, %6.2f  |  %6.2f , %6.2f \r", phi, theta, psi, phi_dot , theta_dot);                

                if(rc_get_state()==RUNNING){
                        rc_led_set(RC_LED_GREEN, 1);
                        rc_led_set(RC_LED_RED, 0);
                }
                else{
                        rc_led_set(RC_LED_GREEN, 0);
                        rc_led_set(RC_LED_RED, 1);
                }
                rc_kalman_update_ekf(&kf,u,y);
                // always sleep at some point
                rc_usleep(50000);
        }
        // turn off LEDs and close file descriptors
        rc_kalman_free(&kf);
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        return 0;
}

double low_pass(double xn, double yn, double alpha){
        return alpha * yn + + (1 - alpha)*xn;
}

// static void __attitude_controller(void){
//     printf("%f",mpu_data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG);
//     printf(" | %f",mpu_data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG);
//     printf(" | %f\r\n",mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
// }


