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

// Kp for heading correction while driving straight. Tune on robot.
const double STRAIGHT_Kp = 5.0;
// Kp and minimum speed for gyro-based turns. Tune on robot.
const double TURN_Kp = 15.0;
const double TURN_MIN_SPEED = 150.0;

void turn_left_90_deg()
{
    double init_o = orientation;
    double target = init_o - 90.0;

    while (orientation > target)
    {
        double remaining = orientation - target;
        double speed = remaining * TURN_Kp;
        if (speed < TURN_MIN_SPEED) speed = TURN_MIN_SPEED;

        robot->move_at_velocity(FRONT_LEFT_PIN,  (int)(-speed * FRONT_LEFT_MULTIPLIER  * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(FRONT_RIGHT_PIN, (int)( speed * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(BACK_LEFT_PIN,   (int)(-speed * BACK_LEFT_MULTIPLIER   * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(BACK_RIGHT_PIN,  (int)( speed * BACK_RIGHT_MULTIPLIER  * GLOBAL_MULTIPLIER));
        robot->wait_for_milliseconds(10);
    }
    stop();
}

void turn_right_90_deg()
{
    double init_o = orientation;
    double target = init_o + 90.0;

    while (orientation < target)
    {
        double remaining = target - orientation;
        double speed = remaining * TURN_Kp;
        if (speed < TURN_MIN_SPEED) speed = TURN_MIN_SPEED;

        robot->move_at_velocity(FRONT_LEFT_PIN,  (int)( speed * FRONT_LEFT_MULTIPLIER  * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(FRONT_RIGHT_PIN, (int)(-speed * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(BACK_LEFT_PIN,   (int)( speed * BACK_LEFT_MULTIPLIER   * GLOBAL_MULTIPLIER));
        robot->move_at_velocity(BACK_RIGHT_PIN,  (int)(-speed * BACK_RIGHT_MULTIPLIER  * GLOBAL_MULTIPLIER));
        robot->wait_for_milliseconds(10);
    }
    stop();
}

void move_forward_for_distance(float distance)
{
    double start_time = seconds();
    double duration_sec = (MOVE_FORWARD_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    double heading = orientation; // lock in starting heading

    while (seconds() - start_time < duration_sec)
    {
        double error = orientation - heading;
        double correction = STRAIGHT_Kp * error;

        robot->move_at_velocity(FRONT_LEFT_PIN,  (int)(750 * FRONT_LEFT_MULTIPLIER  * GLOBAL_MULTIPLIER - correction));
        robot->move_at_velocity(FRONT_RIGHT_PIN, (int)(750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER + correction));
        robot->move_at_velocity(BACK_LEFT_PIN,   (int)(750 * BACK_LEFT_MULTIPLIER   * GLOBAL_MULTIPLIER - correction));
        robot->move_at_velocity(BACK_RIGHT_PIN,  (int)(750 * BACK_RIGHT_MULTIPLIER  * GLOBAL_MULTIPLIER + correction));
        robot->wait_for_milliseconds(10);
    }
    stop();
}

void move_backward_for_distance(float distance)
{
    double start_time = seconds();
    double duration_sec = (MOVE_FORWARD_FOR_DISTANCE_CONSTANT * distance) / 1000.0;
    double heading = orientation;

    while (seconds() - start_time < duration_sec)
    {
        double error = orientation - heading;
        double correction = STRAIGHT_Kp * error;

        robot->move_at_velocity(FRONT_LEFT_PIN,  (int)(-750 * FRONT_LEFT_MULTIPLIER  * GLOBAL_MULTIPLIER - correction));
        robot->move_at_velocity(FRONT_RIGHT_PIN, (int)(-750 * FRONT_RIGHT_MULTIPLIER * GLOBAL_MULTIPLIER + correction));
        robot->move_at_velocity(BACK_LEFT_PIN,   (int)(-750 * BACK_LEFT_MULTIPLIER   * GLOBAL_MULTIPLIER - correction));
        robot->move_at_velocity(BACK_RIGHT_PIN,  (int)(-750 * BACK_RIGHT_MULTIPLIER  * GLOBAL_MULTIPLIER + correction));
        robot->wait_for_milliseconds(10);
    }
    stop();
}

void move_left_for_distance(float distance)
{
    move_left();
    robot->wait_for_milliseconds(MOVE_LEFT_FOR_DISTANCE_CONSTANT * distance);
    stop();
}

void move_right_for_distance(float distance)
{
    move_right();
    robot->wait_for_milliseconds(MOVE_RIGHT_FOR_DISTANCE_CONSTANT * distance);
    stop();
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
    turn_right_90_deg();
    turn_right_90_deg();
    move_backward_for_distance(1.0);     //temp val
    set_arm_down();
    turn_wrist_pickup();

    //lining up for dropping pvcs
    move_forward_for_distance(1.0); //temp val
    turn_right_90_deg();
    turn_right_90_deg();
    line_up_with_black_line_front();
    move_backward_for_distance(1.0); //temp val

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
