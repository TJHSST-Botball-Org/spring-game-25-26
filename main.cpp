#include <kipr/kipr.h>
#include <nano.h>
#include <iostream>
#include "shared.h"

std::atomic<double> orientation(0);

void turn_right_90(){
	double o = orientation;
    while((orientation-o)<90){
     	std::cout << orientation + " - turning right" <<std::endl;   
    }
}

void go_straight(double speed, double duration_sec){
    double xbias = calibrate_gyroscope();
    msleep(1000);

    double start_time = seconds();
    double last_time = start_time;
    double orientation = 0;
    double Kp = 0.005;

    while(seconds() - start_time < duration_sec){
        double current_time = seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        double gx = gyro_x() - xbias;
        orientation += gx * dt;

        double error = -orientation;
        double correction = Kp * error;

        int left = speed - correction;
        int right = speed + correction;

        motor(0, left);
        motor(3, left);
        motor(1, right);
        motor(2, right);

        msleep(10);
    }

    ao();
}

int main() {
    Nano::start_nano();
    Nano::BaseRobot robot;
    
    std::cout << "Robot created!" << std::endl;
    
    robot.calibrate_gyro();
    robot.get_gyro_x();
    turn_right_90();
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
