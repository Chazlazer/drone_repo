/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */
#include <stdio.h>
#include <robotcontrol.h>

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static void __attitude_controller(void);

// Global Variables
static rc_mpu_data_t mpu_data;


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
        mpu_config.dmp_sample_rate = 200;
        mpu_config.orient = ORIENTATION_Y_UP;
        mpu_config.enable_magnetometer = 1;
        // if gyro isn't calibrated, run the calibration routine
        if(!rc_mpu_is_gyro_calibrated()){
                printf("Gyro not calibrated, automatically starting calibration routine\n");
                printf("Let your MiP sit still on a firm surface\n");
                rc_mpu_calibrate_gyro_routine(mpu_config);
        }

        // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }

        rc_mpu_set_dmp_callback(&__attitude_controller);

        rc_set_state(RUNNING);
        while(rc_get_state()!=EXITING){
                // do things based on the state
                if(rc_get_state()==RUNNING){
                        rc_led_set(RC_LED_GREEN, 1);
                        rc_led_set(RC_LED_RED, 0);
                }
                else{
                        rc_led_set(RC_LED_GREEN, 0);
                        rc_led_set(RC_LED_RED, 1);
                }
                // always sleep at some point
                rc_usleep(100000);
        }
        // turn off LEDs and close file descriptors
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        return 0;
}

static void __attitude_controller(void){
    printf("%f",mpu_data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG);
    printf(" | %f",mpu_data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG);
    printf(" | %f\r\n",mpu_data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
}
