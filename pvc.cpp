#include "nano.h"
#include "shared.h"
#include <iostream>
#include <fstream>

// Definitions for shared gyro/accel state declared in shared.h
std::atomic<double> orientation(0);
std::atomic<double> pix(0);
std::atomic<double> piy(0);
std::atomic<double> piz(0);
std::atomic<bool> nr(true);
std::atomic<bool> nr2(true);
std::atomic<bool> nr3(true);

const int FRONT_LEFT_PIN = 0;
const int FRONT_RIGHT_PIN = 3;
const int BACK_LEFT_PIN = 1;
const int BACK_RIGHT_PIN = 2;

const float GLOBAL_MULTIPLIER = 1.5;
const float FRONT_LEFT_MULTIPLIER = 0.95;
const float FRONT_RIGHT_MULTIPLIER = 1;
const float BACK_LEFT_MULTIPLIER = 0.95;
const float BACK_RIGHT_MULTIPLIER = 0.912;

const int PVC_ARM = 1;
const int PVC_WRIST = 2;

const int PVC_ARM_DROP_VALUE = 1472;
const int armDropTemp = 1499;
const int wristDropTemp = 975;

const int PVC_ARM_LOWER_VALUE = 413; // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE
const int PVC_ARM_STOW_VALUE = 413;  // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE
const int PVC_ARM_RAISE_VALUE = 1231; //temp
const int PVC_WRIST_DROP_VALUE = 1030;
const int PVC_WRIST_PICKUP_VALUE = 305;

const int ENTREE_TOUCH_SENSOR = 0;

const int TOPHAT_FRONT_LEFT = 5;
const int TOPHAT_FRONT_RIGHT = 4;
const int TOPHAT_BACK_LEFT = 2;
const int TOPHAT_BACK_RIGHT = 3;

const int TOPHAT_FRONT_LEFT_THRESHOLD = 3900;
const int TOPHAT_FRONT_RIGHT_THRESHOLD = 3900;
const int TOPHAT_BACK_LEFT_THRESHOLD = 3900;
const int TOPHAT_BACK_RIGHT_THRESHOLD = 3900;

const int LEFT_NINETY_DEGREE_WAIT_MS = 1600;  // last 1750
const int RIGHT_NINETY_DEGREE_WAIT_MS = 1590; // last 1750

const int MOVE_FORWARD_FOR_DISTANCE_CONSTANT = 173;        // Adjust based on testing
const int MOVE_LEFT_FOR_DISTANCE_CONSTANT = 183.41463394;  // Adjust based on testing
const int MOVE_RIGHT_FOR_DISTANCE_CONSTANT = 183.41463394; // Adjust based on testing

Nano::BaseRobot* robot = nullptr;

void stop()
{
    robot->freeze(FRONT_LEFT_PIN);
    robot->freeze(FRONT_RIGHT_PIN);
    robot->freeze(BACK_LEFT_PIN);
    robot->freeze(BACK_RIGHT_PIN);
    robot->wait_for_milliseconds(100);
}

void move_forward()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, 750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, 750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_backward()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, -750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, -750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, -750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, -750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_left()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, -750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, 750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, -750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_right()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, -750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, -750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, 750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_diagonal_forward_left()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 0);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, 750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, 0);
}

void move_diagonal_forward_right()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 0);
    robot->move_at_velocity(BACK_LEFT_PIN, 0);
    robot->move_at_velocity(BACK_RIGHT_PIN, 750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_diagonal_backward_left()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, -750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 0);
    robot->move_at_velocity(BACK_LEFT_PIN, 0);
    robot->move_at_velocity(BACK_RIGHT_PIN, -750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void move_diagonal_backward_right()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 0);
    robot->move_at_velocity(FRONT_RIGHT_PIN, -750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, -750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, 0);
}

void turn_clockwise_continuous()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, 750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, -750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, 750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, -750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

void turn_counter_clockwise_continuous()
{
    robot->move_at_velocity(FRONT_LEFT_PIN, -750 * FRONT_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(FRONT_RIGHT_PIN, 750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_LEFT_PIN, -750 * BACK_LEFT_MULTIPLIER * GLOBAL_MULTIPLIER);
    robot->move_at_velocity(BACK_RIGHT_PIN, 750 * BACK_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER);
}

double max(double a, double b)
{
    if (a > b) return a;
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

void turn_left_90_deg()
{
    turn(-90);
}

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

void move_forward_for_distance(float distance)
{
    double duration_sec = (MOVE_FORWARD_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    go_straight(100, duration_sec);
}

void move_backward_for_distance(float distance)
{
    double duration_sec = (MOVE_FORWARD_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    go_straight(-100, duration_sec);
}

void move_left_for_distance(float distance)
{
    double duration_sec = (MOVE_LEFT_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    go_sideways(-100, duration_sec);
}

void move_right_for_distance(float distance)
{
    double duration_sec = (MOVE_RIGHT_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    go_sideways(100, duration_sec);
}

void slowly_set_servo_position(int pin, int position, int wait_delay_ms = 10)
{
    // get_servo_position is intentionally excluded from Nano; safe to call raw as a read-only query
    int current_pos = get_servo_position(pin);

    while (current_pos > position ? current_pos > position : current_pos < position)
    {
        current_pos += (current_pos > position ? -10 : 10);
        robot->set_servo_position(pin, current_pos);
        robot->wait_for_milliseconds(wait_delay_ms);
    }
}

void turn_wrist_drop()
{
    robot->set_servo_enabled(PVC_WRIST, true);
    slowly_set_servo_position(PVC_WRIST, PVC_WRIST_DROP_VALUE);
    robot->set_servo_enabled(PVC_WRIST, false);
}

void turn_wrist_pickup()
{
    robot->set_servo_enabled(PVC_WRIST, true);
    slowly_set_servo_position(PVC_WRIST, PVC_WRIST_PICKUP_VALUE);
    robot->set_servo_enabled(PVC_WRIST, false);
}

void set_arm_up()
{
    robot->set_servo_enabled(PVC_ARM, true);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_RAISE_VALUE);
    robot->set_servo_enabled(PVC_ARM, false);
}

void set_arm_down()
{
    robot->set_servo_enabled(PVC_ARM, true);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_LOWER_VALUE);
    robot->set_servo_enabled(PVC_ARM, false);
}

bool is_fl_tophat_black()
{
    return robot->get_analog(TOPHAT_FRONT_LEFT) > TOPHAT_FRONT_LEFT_THRESHOLD;
}

bool is_fr_tophat_black()
{
    return robot->get_analog(TOPHAT_FRONT_RIGHT) > TOPHAT_FRONT_RIGHT_THRESHOLD;
}

bool is_bl_tophat_black()
{
    return robot->get_analog(TOPHAT_BACK_LEFT) > TOPHAT_BACK_LEFT_THRESHOLD;
}

bool is_br_tophat_black()
{
    return robot->get_analog(TOPHAT_BACK_RIGHT) > TOPHAT_BACK_RIGHT_THRESHOLD;
}

void line_up_with_black_line_front()
{
    move_forward();

    while (!(is_fl_tophat_black() and is_fr_tophat_black()))
    {
        if (is_fl_tophat_black() and !is_fr_tophat_black())
        {
            turn_counter_clockwise_continuous();
        }
        else if (!is_fl_tophat_black() and is_fr_tophat_black())
        {
            turn_clockwise_continuous();
        }
        else if (!is_fl_tophat_black() and !is_fr_tophat_black())
        {
            move_forward();
        }
    }

    stop();
}

void line_up_with_black_line_behind()
{
    move_backward();

    while (!(is_bl_tophat_black() and is_br_tophat_black()))
    {
        if (is_bl_tophat_black() and !is_br_tophat_black())
        {
            turn_clockwise_continuous();
        }
        else if (!is_bl_tophat_black() and is_br_tophat_black())
        {
            turn_counter_clockwise_continuous();
        }
        else if (!is_bl_tophat_black() and !is_br_tophat_black())
        {
            move_backward();
        }
    }

    stop();
}

int main()
{
    Nano::BaseRobot r;
    robot = &r;
    robot->calibrate_gyro();
    robot->wait_for_milliseconds(1000); // let gyro settle

    //start positions
    set_arm_up();
    turn_wrist_pickup();

    //getting pvcs
    turn(90);
    turn(90);
    go_straight(-1.0, 1.0);     //temp val
    set_arm_down();
    turn_wrist_pickup();

    //lining z for dropping pvcs
    go_straight(1.0, 1.0);     //temp val
    turn(90);
    turn(90);
    line_up_with_black_line_front();
    go_straight(-1.0, 1.0);     //temp val

    //dropping pvcs

    set_arm_up();
    turn_wrist_drop();

    //reset
    turn_wrist_drop();
    set_arm_down();
}

/*

open_entree_claw();
    move_forward_for_distance(6.4);
    close_entree_claw();
    move_backward_for_distance(3.9);
    move_right_for_distance(5.8);
    turn_right_90_deg();
    turn_right_90_deg();
    move_forward_for_distance(1);
*/
