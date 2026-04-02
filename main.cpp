#include <kipr/kipr.h>
#include <nano.h>
#include <nano2.h>
#include <nano3.h>
#include <iostream>
#include "shared.h"

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

double max(double a, double b){
    if (a>b){
        return a;
    }
    return b;
}

/*
void turn(int amount){
    double init_o = orientation.load();
    double target = init_o + amount;
    double last_o = init_o;

    double Kp = 1.0;
    double Kd = 1;  // start here, tune up if still overshooting

    while(true){
        double current = orientation.load();
        double error = target - current;

        // Exit condition
        if (amount > 0 && error <= 1.0) break;
        if (amount < 0 && error >= -1.0) break;

        // Derivative = how fast orientation is changing
        double derivative = (current - last_o) / 0.01;

        // PD control
        double speed = error * Kp - derivative * Kd;

        // Clamp
        if (speed > 50) speed = 50;
        if (speed < -50) speed = -50;

        std::cout << "err: " << error << " deriv: " << derivative << " spd: " << speed << std::endl;

        motor(0, speed);
        motor(1, speed);
        motor(2, -speed);
        motor(3, -speed);

        last_o = current;
        msleep(10);
    }
    ao();
}
*/


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

void straight(int inches) {
    std::cout << "STRAIGHT METHOD STARTED" << std::endl;
    if (inches < 0) {
        // Backward
        double init_pos = pix;  // Your atomic/global position variable
        double speed;
        while (pix > init_pos + inches) {  // inches is negative, so we're decreasing
            speed = (init_pos + inches) - pix;
            speed = max(speed, 50);
            std::cout << pix << std::endl;
            // All motors same direction for straight (backward)
            motor(0, -speed);
            motor(1, -speed);
            motor(2, -speed);
            motor(3, -speed);
            msleep(10);
        }
        ao();
    }
    else {
        // Forward
        double init_pos = pix;
        double speed;
        while (pix < init_pos + inches) {
            speed = -1 * ((init_pos + inches) - pix);
            speed = max(speed, 50);
            std::cout << pix << std::endl;
            // All motors same direction for straight (forward)
            motor(0, -speed);
            motor(1, -speed);
            motor(2, -speed);
            motor(3, -speed);
            msleep(10);
        }
        ao();
    }
    ao();
}


int main() {






    /*const int armDropTemp = 1499;
//const int wristDropTemp = 975;
    enable_servo(0);
    set_servo_position(0, 975);
    enable_servo(1);
    set_servo_position(1, 1499);
    msleep(10000);
    return(0);
    */


    //SO COPY NANO INTO A NEW FILE AND CLASS AND MAKE ONE RUN GYRO, ONE RUN ACCEL TO AVOID THE WHILE TRUE ISSUE

    Nano::BaseRobot gyro_bot;       // Starts Nano
    Nano2::BaseRobot accel_x_bot;   // Starts Nano2
    Nano3::BaseRobot accel_y_bot;   // Starts Nano3

    std::cout << "Robot created!" << std::endl;

    //gyro_bot.calibrate_gyro();
    accel_x_bot.calibrate_accel_x();

    // go_straight(50,10);

    msleep(1000);
    //msleep(10000);
    /*  for(int i = 0; i < 200; i++){
    std::cout << "pix: " << pix << std::endl;
    msleep(10);
}*/

    //gyro_bot.get_gyro_x();

    accel_x_bot.get_accel_x();
    //  std::cout << "Driving straight...\n";
    //while(true){
    //	std::cout<< orientation << std::endl;  
    //    msleep(50);
    //}
    //turn(180);
    //msleep(10);
    straight(5);


    //robot.calibrate_gyro();
    // msleep(1000);
    // robot.get_gyro_x();
    // turn(90);


    //go_straight(50, 15);  // use seconds
    //motor(0, 50);
    //motor(1, 50);
    //motor(3, 50);
    //motor(2, 50);
    // msleep(5000); //500
    return 0;
    //turn_right_90();
    //while(true){
    //	std::cout << orientation << std::endl; 
    //msleep(10);
    //}
    // Move motor 0
    //robot.set_motor_power(0, 100);
    //robot.set_motor_power(1, 100);
    //msleep(1000);
    //robot.wait_for_milliseconds(1000);
    //robot.set_motor_power(0, 0);
    //msleep(10000);

    std::cout << "Done!" << std::endl;

    return 0;
}
