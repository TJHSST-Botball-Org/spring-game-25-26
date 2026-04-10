#include <kipr/kipr.h>
#include <nano.h>
#include <iostream>
#include "shared.h"

std::atomic<double> orientation(0);

const int SCOOP_PIN = 2;
const int CLAW_PIN = 0;
const int ARM_PIN = 1;

const int UPPER_SCOOP = 1030;
const int MIDDLE_SCOOP = 360;//400
const int LOWER_SCOOP = 95;//121
const int STORAGE_SCOOP = 1015;
const int STORAGE_ARM = 1000;
const int RAISE_ARM = 1325;
const int LOWER_ARM = 1911;
const int OPEN_CLAW = 450;
const int CLOSE_CLAW = 950;
const int SCOOP_LOWEST = 85;
const int RAMP_SCOOP = 525;
const int LOWER_ARM_PALET = 1690;

void go_straight_normal(double speed, double duration_sec, int direction) { //non-gyro
    double start_time = seconds();

    while (seconds() - start_time < duration_sec) {
        int right = 0;
        int left = 0;

        if(direction>0)
        {
        	left  = (int)(speed);
        	right = (int)(-1)*(speed);
        }
        else
        {
            left  = (int)(-1)*(speed);
        	right = (int)(speed);
        }

        left  = std::max(-100, std::min(100, left));
        right = std::max(-100, std::min(100, right));

        std::cout << "ori=" << orientation.load()
                  << " err=" << error
                  << " L=" << left
                  << " R=" << right << std::endl;

        motor(2, right);
        motor(3, left);

        msleep(10);
    }
    ao();
}


void go_straight(double speed, double duration_sec, int direction) {
    double start_time = seconds();
    double Kp = 4.2;
    double baseline = orientation.load();

    while (seconds() - start_time < duration_sec) {
        double error = orientation.load() - baseline;
        double correction = Kp * error;

        int right = 0;
        int left = 0;

        if(direction>0)
        {
        	left  = (int)(speed - correction);
        	right = (int)(-1)*(speed + correction);
        }
        else
        {
            left  = (int)(-1)*(speed + correction);
        	right = (int)(speed - correction);
        }

        left  = std::max(-100, std::min(100, left));
        right = std::max(-100, std::min(100, right));

        std::cout << "ori=" << orientation.load()
                  << " err=" << error
                  << " L=" << left
                  << " R=" << right << std::endl;

        motor(2, right);
        motor(3, left);

        msleep(10);
    }
    ao();
}

void slowly_set_servo_position(int pin, int position) {
	int currentPos = get_servo_position(pin);
    if (currentPos > position) {
    	for (int i = currentPos; i>=position; i=i-15) {
    		set_servo_position(pin, i);
        	msleep(10);
    	}
    }

    if (currentPos < position) {
    	for (int i = currentPos; i<=position; i=i+15) {
    		set_servo_position(pin, i);
        	msleep(10);
    	}
    }
}

void turn(int amount) {
    double init_o = orientation.load();
    double target = init_o + amount;

    if (amount > 0) {
        // turn right: left forward, right backward
        while (orientation.load() < target) {
            double error = target - orientation.load();
            double speed = std::max(5.0, std::min((double)error, 50.0));
            motor(2, (int) speed);
            motor(3, (int) speed);
            msleep(10);
        }
    } else {
        // turn left: left backward, right forward
        while (orientation.load() > target) {
            double error = orientation.load() - target;
            double speed = std::max(5.0, std::min((double)error, 50.0));
            motor(2, (int) -speed);
            motor(3, (int) -speed);
            msleep(10);
        }
    }
    ao();
}

void raise_scoop() {
	set_servo_position(SCOOP_PIN, UPPER_SCOOP);
    msleep(1000);
}
void store_scoop() {
	set_servo_position(SCOOP_PIN, STORAGE_SCOOP);
    msleep(1000);
}

void ramp_scoop() {
	set_servo_position(SCOOP_PIN, RAMP_SCOOP);
    msleep(1000);
}

void raise_arm() {
	slowly_set_servo_position(ARM_PIN, RAISE_ARM);
    msleep(1000);
}

void store_arm() {
	slowly_set_servo_position(ARM_PIN, STORAGE_ARM);
    msleep(1000);
}

void lower_arm() {
	slowly_set_servo_position(ARM_PIN, LOWER_ARM);
    msleep(1000);
}

void lower_arm_palet() {
	slowly_set_servo_position(ARM_PIN, LOWER_ARM_PALET);
    msleep(1000);
}

void lower_scoop() {
	set_servo_position(SCOOP_PIN, LOWER_SCOOP);
    msleep(1000);
}

void middle_scoop() {
	slowly_set_servo_position(SCOOP_PIN, MIDDLE_SCOOP);
    msleep(1000);
}

void open_claw() {
	slowly_set_servo_position(CLAW_PIN, OPEN_CLAW);
    msleep(1000);
}

void close_claw() {
	slowly_set_servo_position(CLAW_PIN, CLOSE_CLAW);
    msleep(1000);
}

void scoop_lowest() {
    set_servo_position(SCOOP_PIN, SCOOP_LOWEST);
    msleep(500);
}

int main() {
    Nano::start_nano();
    Nano::BaseRobot robot;
    std::cout << "Robot created!" << std::endl;

    enable_servos();

    robot.calibrate_gyro();
    msleep(1000);
    robot.get_gyro_x();

    std::cout << "Driving straight...\n";


    //set up
    store_scoop();
    store_arm();
    close_claw();

	//pick up cubes
    open_claw();
    turn(-90);
    go_straight(40, 4.5, 1);
    turn(90);
    go_straight(40, 2.2, -1);
    go_straight(35, 3.7, 1);
    lower_arm();
    close_claw();
    raise_arm();

    //place cubes on other pallet
    go_straight(50, 1.5, 1);
    lower_arm_palet();
    open_claw();

	//leave start box
    go_straight(50, 1.65, -1);
    store_arm();
    close_claw();
    turn(150);
    go_straight(55, 5.3, -1);
    turn(30);

	// push cones
    go_straight(75, 4, -1);
    lower_scoop();
    go_straight(75, 10, -1);
    scoop_lowest();
    go_straight(75, 5.3, -1);

	//pick up cones
    go_straight(15, 1.2, 1);
    middle_scoop();

	//go back to start box
    go_straight(50, 20, 1);
    turn(-90);
    go_straight(50, 7.2, 1);
    turn(-90);

	//go up ramp
    ramp_scoop();
    go_straight(60, 10, 1);
    middle_scoop();




    std::cout << "Done!" << std::endl;
    return 0;
}
