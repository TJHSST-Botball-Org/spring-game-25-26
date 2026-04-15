#include <kipr/wombat.h>
#include <iostream>
#include <fstream>
#include <nano.h>
#include "shared.h"

const int FRONT_LEFT_PIN = 3;
const int FRONT_RIGHT_PIN = 0;
const int BACK_LEFT_PIN = 1;
const int BACK_RIGHT_PIN = 2;
//MAKE IT SO IT ISNT HARDCODED LATER ON
const int PVC_ARM = 1;
const int PVC_WRIST = 2;

const int PVC_ARM_DROP_VALUE = 1231; //1231
const int armDropTemp = 1499;
const int wristDropTemp = 975;

const int PVC_ARM_RAISE_VALUE = 1231;
const int PVC_ARM_LOWER_VALUE = 176; // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE 200    120
const int PVC_ARM_FULL_LOWER_VALUE = 67; //FOR WHEN WE REACH THE DISPENSOR. PUTTING IT AT THIS ALL THE TIME MESSES WITH MOVING.
const int PVC_ARM_STOW_VALUE = 413;  // TENTATIVE, WE DONT HAVE WHEELS YET SO THIS COULD CHANGE
const int PVC_ARM_START_VALUE = 1510;
const int PVC_WRIST_START_VALUE = 2407;
const int PVC_WRIST_DROP_VALUE = 1424;
const int PVC_WRIST_PICKUP_VALUE = 395; //292    350      400

const int TOPHAT_FRONT_LEFT = 0;
const int TOPHAT_FRONT_RIGHT = 1;

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
    double gx = orientation.load();

    while(seconds() - start_time < duration_sec){
        std::cout<< orientation.load() << std::endl;
        double current_time = seconds();
        //double dt = current_time - last_time;
        last_time = current_time;

        //double gx = orientation;
        oo = orientation.load() - gx;

        double error = oo; // 0 :(
        double correction = Kp * error;

        int left = speed - correction;
        int right = speed + correction;

        motor(3, left);
        motor(1, left);
        motor(0, right);
        motor(2, right);

        msleep(10);


    }

    ao();
}

void go_straight_until_tophat(double speed) {
    double last_time = seconds();
    double Kp = 1.09;
    double gx = orientation;
    
    while (analog(0) < TOPHAT_FRONT_LEFT_THRESHOLD && analog(1) < TOPHAT_FRONT_RIGHT_THRESHOLD) {
        std::cout << "L: " << analog(0) << " R: " << analog(1) << " orientation: " << orientation << std::endl;
        double current_time = seconds();
        double dt = current_time - last_time;
        last_time = current_time;
        
        double error = orientation - gx;
        double correction = Kp * error;
        
        int left = speed - correction;
        int right = speed + correction;
        
        motor(3, left);
        motor(1, left);
        motor(0, right);
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
        motor(3, speed - correction);   // front-left
        motor(1, -speed - correction);  // back-left
        motor(2, speed + correction);   // back-right
        motor(0, -speed + correction);  // front-right
        
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
            motor(0, speed);
            motor(1, -speed);
            motor(3, -speed);
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
            motor(3, speed);
            motor(1, speed);
            motor(0, -speed);
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

void set_arm_fully_down() //for the slammed position
{
    enable_servo(PVC_ARM);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_FULL_LOWER_VALUE);
    disable_servo(PVC_ARM);
}

void start_position(){
    enable_servo(PVC_ARM);
    slowly_set_servo_position(PVC_ARM, PVC_ARM_START_VALUE);
    disable_servo(PVC_ARM);
    
    enable_servo(PVC_WRIST);
    slowly_set_servo_position(PVC_WRIST, PVC_WRIST_START_VALUE);
    disable_servo(PVC_WRIST);
    return;
}

int main()
{
    //start_position();
    wait_for_light(5);
    shut_down_in(119);
    //msleep(100);
    
    set_arm_up();
    turn_wrist_drop();
    //set_arm_down();
    //return 0;
    Nano::BaseRobot gyro_bot;       // Starts Nano

    std::cout << "Robot created!" << std::endl;
    gyro_bot.calibrate_gyro();
	msleep(1000);
    gyro_bot.get_gyro_x();

/*
    set_arm_up();
    turn_wrist_pickup();
	go_sideways(50, 4);
    //getting pvcs
    //turn(180);
    go_straight(50, 10);     //temp val
    //msleep()
    set_arm_down();
    turn_wrist_pickup();
    go_sideways(-50, 2);
    */
    
    //set_arm_up();
    //turn_wrist_pickup();
    
	//go_sideways(50, 300);
    
    //getting pvcs
    go_straight_until_tophat(100); //old speed was 50 and 80
    //go_sideways(50.0, 4.0);
    go_straight(100.0, 1.9);     //leaving box 5.8 2.9 2.3     speed was 80
	go_sideways(100, 1.9); //2.5 speed was 80
    go_straight(100, 2); //5    speed was 80
    msleep(3000); //miss the first pvc and get the 2nd - 5th. so we dont rush the speed - more accurate   2800  3200
    turn_wrist_pickup();
    set_arm_down();

    go_sideways(-60.0, 2.3);    //to ram into wall on the side //3     2.5
    go_straight(50.0, 0.12);    //and align with the dropper
    set_arm_fully_down();
    msleep(27000); //AG NEW old was 28000    320000     24000 26500   28000
    set_arm_up();
    msleep(3850); //4500   4350 (this was with no change to line 294)

    //lining up for dropping pvcs
    go_straight(-50.0, 1.0);     //temp val
    //line_up_with_black_line_front();
    go_straight(-50.0, 7.6);     //temp val 8.0 seconds old     8.2
	go_sideways(-50,2.7); //4    2.5
    go_straight(-50,1.45); //0.5   0.62    1.1     1.32
    go_sideways(50,2.0); //0.5    1.5
    go_sideways(-50, 1.33); //newwwwwwwwwwww 1.0 //1.5      1.75   1.54   1.06   1.3
    //go_straight(100, 0.58); //NEW THIS IS ONLY FOR SEEDING REMEMBER TO REMOVE TS THIS IS BECAUSE WE HAVE LUNCHH  THGIBHTIOBHTIOHBIOERHB
    //go_straight(-50.0, 1.3); //0.5 seconds     0.99   1.3
    //dropping pvcs
    //go_straight(-100, 0.5); new commetn out                                                                      bbbb
    //go_straight(100, 0.06); // NEWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
    //go_straight(100, 0.04); //new to adjust for new arm
    go_straight(100, 0.8);
    turn_wrist_drop();
    go_straight(100, 0.008); //new to adjust for new arm //0.06
    go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
	    go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
        go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
        go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
    go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
    go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
    go_sideways(100, 0.1); //holy shake
    go_sideways(-100, 0.1);
    go_straight(100, 0.06);
    go_straight(-100, 0.06);
    msleep(2200);
    turn(-10); //newwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
    go_straight(100, 0.05); //newwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
    //slowly_set_servo_position(PVC_WRIST, PVC_WRIST_PICKUP_VALUE); //slowly do it yea
    turn_wrist_pickup();
    
    //go back to get more
    turn(-15);
    go_sideways(-100, 3);
    go_straight_until_tophat(100); //new
    msleep(1000); //new
    go_straight(50.0, 4.0);    //5.8
    go_sideways(50.0, 3);
    go_straight(50, 5);
    set_arm_down();
    //go_straight(-100, 0.25); //NEWW
    go_sideways(50.0, 5);
    go_straight(50.0, 0.12);
    msleep(000); //part 2, after dropping the first set
    
    return (0);
    
    set_arm_up();
    go_straight(-100, 0.15);
    turn(10);
    go_sideways(100, 11);
    go_sideways(-100, 1);
    go_straight(-100, 1);
    turn(90);
    go_sideways(-100, 1);
    go_straight(100, 1);
    go_sideways(100, 3);
    go_sideways(-100, 3.5);
    go_straight(100, 0.5);
    go_sideways(100, 1);
    go_straight(-100, 1);
    turn(180);
    /* this was the inital code. the code above is my plan b code
    go_straight(50, 2);
    go_straight(-50.0, 3);
    go_sideways(100, 10); //old speed 50 and 20 seconds
    go_straight(50, 4);
    go_straight(-50, 2);
    go_sideways(-50, 1.5); //0.5
    turn(-90);
    go_straight(-50, 1); //new
    turn_wrist_drop();
    msleep(3000);
    */
    
}
