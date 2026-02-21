#include <kipr/kipr.h>
#include <nano.h>
#include <iostream>

int main() {
    Nano::BaseRobot robot;
    
    std::cout << "Robot created!" << std::endl;
    
    // Move motor 0
    robot.set_motor_power(0, 100);
    //robot.set_motor_power(1, 100);
    msleep(1000);
    //robot.wait_for_milliseconds(1000);
    robot.set_motor_power(0, 0);
    msleep(10000);
    
    std::cout << "Done!" << std::endl;
    
    return 0;
}
