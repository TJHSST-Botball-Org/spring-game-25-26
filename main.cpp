#include <kipr/wombat.h>
#include <iostream>
#include <fstream>
#include <nano.h>
#include "shared.h"

const int FRONT_LEFT_PIN = 0;
const int FRONT_RIGHT_PIN = 3;
const int BACK_LEFT_PIN = 1;
const int BACK_RIGHT_PIN = 2;

const int PVC_ARM = 1;
const int PVC_WRIST = 0;

const int PVC_ARM_DROP_VALUE = 1231;
const int armDropTemp = 1499;
const int wristDropTemp = 975;

const int PVC_ARM_RAISE_VALUE = 1231;
const int PVC_ARM_LOWER_VALUE = 2047; // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE
const int PVC_ARM_STOW_VALUE = 413;  // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE
const int PVC_WRIST_DROP_VALUE = 670;
const int PVC_WRIST_PICKUP_VALUE = 2047;

const int TOPHAT_FRONT_LEFT = 5;
const int TOPHAT_FRONT_RIGHT = 4;

const int TOPHAT_FRONT_LEFT_THRESHOLD = 3900;
const int TOPHAT_FRONT_RIGHT_THRESHOLD = 3900;

std::atomic<double> orientation(0);
std::atomic<double> pix(0);
std::atomic<double> piy(0);
std::atomic<double> piz(0);
std::atomic<bool> nr(true);
std::atomic<bool> nr2(true);
std::atomic<bool> nr3(true);


void turn_right_90(){
    double o = orientation;
    while((orientation-o)<90){
        std::cout << std::to_string(orientation) + " - turning right" <<std::endl;   
    }
}

void go_straight(double speed, double duration_sec){
    //double xbias = calibrate_gyroscope();
    //msleep(1000);

    double start_time = seconds();
    double last_time = start_time;
    //double orientation = 0;
    double oo = 0;
    double Kp = 1.09; //0.005    0.5
    double gx = orientation;

    while(seconds() - start_time < duration_sec){
        std::cout<< orientation << std::endl;
        double current_time = seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        //double gx = orientation;
        oo = orientation - gx;

        double error = orientation; // 0 :(
        double correction = Kp * error;

        int left = speed - correction;
        int right = speed + correction;

        motor(0, left);
        motor(1, left);
        motor(3, right);
        motor(2, right);

        msleep(10);


    }

    ao();
}

void go_sideways(double speed, double duration_sec) {
    double start_time = seconds();
    double last_time = start_time;
    double Kp = 1.09;
    double gx = orientation;  // Lock in starting orientation
    
    while (seconds() - start_time < duration_sec) {
        std::cout << orientation << std::endl;
        double current_time = seconds();
        double dt = current_time - last_time;
        last_time = current_time;
        
        double error = orientation - gx;  // How far we've drifted
        double correction = Kp * error;
        
        // Flipped correction signs
        motor(0, speed - correction);   // front-left
        motor(1, -speed - correction);  // back-left
        motor(2, speed + correction);   // back-right
        motor(3, -speed + correction);  // front-right
        
        msleep(10);
    }
    ao();
}

double max(double a, double b){
    if (a>b){
        return a;
    }
    return b;
}
void turn(int amount){
    if(amount < 0){
        double init_o = orientation;
        double speed;
        while(orientation > init_o + amount){ //+10
            speed = orientation - (init_o + amount);
            speed = max(speed, 5);
            std::cout<< orientation << std::endl;
            motor(0, -speed);
            motor(1, -speed);
            motor(3, speed);
            motor(2, speed);   
            msleep(10);
        }
        ao();
    }
    else{
        double init_o = orientation;
        double speed;
        while(orientation < init_o + amount){ //-10
            speed = -1 * (orientation - (init_o + amount));
            speed = max(speed, 5);
            std::cout<< orientation << std::endl;
            motor(0, speed);
            motor(1, speed);
            motor(3, -speed);
            motor(2, -speed);   
            msleep(10);
        }
        ao();
    }
    ao();
}

void stop()
{
    freeze(FRONT_LEFT_PIN);
    freeze(FRONT_RIGHT_PIN);
    freeze(BACK_LEFT_PIN);
    freeze(BACK_RIGHT_PIN);
    msleep(100);
}

void slowly_set_servo_position(int pin, int position, int wait_delay_ms = 10)
{
    int initial_pos = get_servo_position(pin);

    while (initial_pos > position ? get_servo_position(pin) > position : get_servo_position(pin) < position)
    {
        set_servo_position(pin, get_servo_position(pin) + (initial_pos > position ? -10 : 10));
        msleep(wait_delay_ms);
    }
}

void turn_wrist_drop()
{
    enable_servo(PVC_WRIST);
    slowly_set_servo_position(PVC_WRIST, PVC_WRIST_DROP_VALUE);
    disable_servo(PVC_WRIST);
}

void turn_wrist_pickup()
{
    enable_servo(PVC_WRIST);
    slowly_set_servo_position(PVC_WRIST, PVC_WRIST_PICKUP_VALUE);
    disable_servo(PVC_WRIST);
}

void set_arm_up()
{
    enable_servo(PVC_ARM);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_RAISE_VALUE);
    disable_servo(PVC_ARM);
}

void set_arm_down()
{
    enable_servo(PVC_ARM);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_LOWER_VALUE);
    disable_servo(PVC_ARM);
}

int main()
{
    set_arm_up();
    turn_wrist_drop();
    //set_arm_down();
    return 0;
    Nano::BaseRobot gyro_bot;       // Starts Nano

    std::cout << "Robot created!" << std::endl;
    gyro_bot.calibrate_gyro();
	msleep(1000);
    gyro_bot.get_gyro_x();
    go_sideways(50, 15);
}
