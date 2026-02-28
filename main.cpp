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
