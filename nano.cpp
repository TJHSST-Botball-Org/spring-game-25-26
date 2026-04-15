#include "nano.h"
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include "shared.h"

namespace Nano {

    ///////////////////////////////////////////////////////////////
    // Message Types
    ///////////////////////////////////////////////////////////////

    enum class MessageType {
        WFM,
        CG,
        CAX,
        CAZ,
        CAY,

        // Motor
        GET_MOTOR_POSITION,
        CLEAR_MOTOR_POSITION,
        SET_MOTOR_POWER,
        MOVE_AT_VELOCITY,
        MOVE_TO_POSITION,
        MOVE_RELATIVE_POSITION,
        IS_MOTOR_DONE,
        FREEZE,

        // Servo
        SET_SERVO_ENABLED,
        SET_ALL_SERVOS_ENABLED,
        SET_SERVO_POSITION,
        IS_SERVO_ENABLED,

        // Digital / Analog
        GET_ANALOG,
        GET_DIGITAL,
        SET_DIGITAL,

        // Gyro
        GET_GYRO_X,
        GET_GYRO_Y,
        GET_GYRO_Z,
        GET_ACCEL_Z,
        GET_ACCEL_X,
        GET_ACCEL_Y,

        // PID
        GET_PID_GAINS,
        SET_PID_GAINS,

        // PWM
        GET_PWM,
        SET_PWM
    };

    ///////////////////////////////////////////////////////////////
    // Message Struct
    ///////////////////////////////////////////////////////////////

    struct Message {
        unsigned int id;
        MessageType type;

        int a;
        int b;
        int c;
        int d;
        int e;
        int f;
        int g;

        bool flag;

        int response_int;
        bool response_bool;
    };

    ///////////////////////////////////////////////////////////////
    // Globals
    ///////////////////////////////////////////////////////////////

    std::queue<Message> request_queue;
    std::queue<Message> response_queue;
    std::mutex message_mutex;
    int g_z_bias = 0;
    int a_x_bias = 0;
    int a_z_bias = 0;
    int a_y_bias = 0;

    unsigned int next_message_id = 0;
    bool worker_started = false;
    std::thread worker_thread;
    std::thread accel_thread;

    //accel thread - Matthew 3/27


    ///////////////////////////////////////////////////////////////
    // Worker Thread
    ///////////////////////////////////////////////////////////////

    void worker_thread_function() {

        while (true) {

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            message_mutex.lock();

            if (!request_queue.empty()) {

                Message msg = request_queue.front();
                request_queue.pop();

                switch (msg.type) {
                    case MessageType::WFM:
                        msleep(msg.a);
                        break;

                        // MOTOR
                    case MessageType::SET_MOTOR_POWER:
                        motor(msg.a, msg.b);
                        break;

                    case MessageType::CG:
                        {
                            double gyro_z_sum = 0;
                            int num_samples = 100;  // Number of samples for averaging
                            for (int i = 0; i<num_samples; i++){
                                double gz = gyro_y();  // Read gyroscope data          gyro_z()
                                gyro_z_sum += gz;  // Accumulate gyro z readings
                                msleep(10);
                            }
                            double gyro_z_bias = gyro_z_sum / num_samples;  // Calculate bias for gyro z
                            //std::cout << gyro_z_bias;
                            g_z_bias = gyro_z_bias;
                            printf("cg done");
                            response_queue.push(msg);
                            //printf("cg done");
                        }
                        break;
                    case MessageType::CAX:
                        {
                            double accel_x_sum = 0;
                            int num_samples = 500;  // Number of samples for averaging
                            for (int i = 0; i < num_samples; i++) {
                                double ax = accel_x();  // Read accelerometer data
                                accel_x_sum += ax;  // Accumulate accel x readings
                                msleep(10);
                            }
                            double accel_x_bias = accel_x_sum / num_samples;  // Calculate bias for accel x
                            //std::cout << accel_x_bias;
                            a_x_bias = accel_x_bias;
                            printf("caxx done");
                            response_queue.push(msg);
                            //prinf("cax done");
                        }
                        break;
                    case MessageType::CAY:
                        {
                            double accel_y_sum = 0;
                            int num_samples = 500;  // Number of samples for averaging
                            for (int i = 0; i < num_samples; i++) {
                                double ay = accel_y();  // Read accelerometer data
                                accel_y_sum += ay;  // Accumulate accel x readings
                            }
                            double accel_y_bias = accel_y_sum / num_samples;  // Calculate bias for accel x
                            //std::cout << accel_x_bias;
                            a_y_bias = accel_y_bias;
                            response_queue.push(msg);
                        }
                        break;
                    case MessageType::CAZ:
                        {
                            double accel_z_sum = 0;
                            int num_samples = 500;  // Number of samples for averaging
                            for (int i = 0; i < num_samples; i++) {
                                double az = accel_z();  // Read accelerometer data
                                accel_z_sum += az;  // Accumulate accel x readings
                            }
                            double accel_z_bias = accel_z_sum / num_samples;  // Calculate bias for accel x
                            //std::cout << accel_x_bias;
                            a_z_bias = accel_z_bias;
                            response_queue.push(msg);
                        }
                        break;

                    case MessageType::FREEZE:
                        motor(msg.a, 0);
                        break;

                    case MessageType::GET_MOTOR_POSITION:
                        msg.response_int = get_motor_position_counter(msg.a);
                        response_queue.push(msg);
                        break;

                    case MessageType::CLEAR_MOTOR_POSITION:
                        clear_motor_position_counter(msg.a);
                        break;

                    case MessageType::MOVE_AT_VELOCITY:
                        move_at_velocity(msg.a, msg.b);
                        break;

                    case MessageType::MOVE_TO_POSITION:
                        move_to_position(msg.a, msg.b, msg.c);
                        break;

                    case MessageType::MOVE_RELATIVE_POSITION:
                        move_relative_position(msg.a, msg.b, msg.c);
                        break;

                    case MessageType::IS_MOTOR_DONE:
                        msg.response_bool = get_motor_done(msg.a);
                        response_queue.push(msg);
                        break;

                        // SERVO
                    case MessageType::SET_SERVO_ENABLED:
                        set_servo_enabled(msg.a, msg.flag);
                        break;

                    case MessageType::SET_ALL_SERVOS_ENABLED:
                        if (msg.flag) enable_servos();
                        else disable_servos();
                        break;

                    case MessageType::SET_SERVO_POSITION:
                        set_servo_position(msg.a, msg.b);
                        break;

                    case MessageType::IS_SERVO_ENABLED:
                        msg.response_bool = get_servo_enabled(msg.a);
                        response_queue.push(msg);
                        break;

                        // ANALOG / DIGITAL
                    case MessageType::GET_ANALOG:
                        msg.response_int = analog(msg.a);
                        response_queue.push(msg);
                        break;

                    case MessageType::GET_DIGITAL:
                        msg.response_int = digital(msg.a);
                        response_queue.push(msg);
                        break;

                    case MessageType::SET_DIGITAL:
                        set_digital_value(msg.a, msg.b);
                        break;

                        // GYRO
                    case MessageType::GET_GYRO_X:
                        {
                            double last_time = seconds();  // Store the initial time
                            //std::cout << last_time;
                            double o = 0.0;  // Initialize the orientation to 0
                            while(nr){
                                //gyro_calibrate();
                                //float gx = gyro_x();
                                //float gy = gyro_y();
                                double gz = gyro_y() - g_z_bias; // Read gyroscope data             gyro_z()
                                double current_time = seconds();  // Get the current time
                                double delta_time = current_time - last_time;  // Calculate the time difference
                                //std::cout << delta_time;
                                o = o + gz * delta_time;  // Update the orientation based on angular velocity
                                orientation.store(static_cast<double>(o / 8.248)); ; // orientation = o / 8.248
                                last_time = current_time;  // Update last_time to current time
                                //std::cout << orientation_deg;  // Output the estimated orientation
                                //std::cout << "break";
                                msleep(10); // before 10     Sleep 100ms for example to only integrate new gyro values
                            }
                        }
                        break;

                    case MessageType::GET_GYRO_Y:
                        msg.response_int = gyro_y();
                        response_queue.push(msg);
                        break;

                    case MessageType::GET_GYRO_Z:
                        msg.response_int = gyro_z();
                        response_queue.push(msg);
                        break;
                    case MessageType::GET_ACCEL_X:
                        {
                            double last_time = seconds();  // Store the initial time
                            double velocity_x = 0.0;  // Initialize velocity to 0
                            double position_x = 0.0;  // Initialize position to 0 (inches)

                            // Conversion: 1024 units = 1G = 386.1 in/sB2
                            // So 1 unit = 386.1 / 1024 b	 0.377 in/sB2
                            const double UNITS_TO_INCHES_PER_S2 = 386.1 / 1024.0;

                            while(nr) {
                                double ax = accel_x() - a_x_bias;  // Read accelerometer data, remove bias
                                double current_time = seconds();  // Get the current time
                                double delta_time = current_time - last_time;  // Calculate the time difference

                                // Convert to inches/sB2
                                double accel_inches = ax * UNITS_TO_INCHES_PER_S2;

                                // First integration: acceleration b velocity
                                velocity_x = velocity_x + accel_inches * delta_time;

                                // Second integration: velocity b position
                                position_x = position_x + velocity_x * delta_time;

                                pix.store(static_cast<double>(position_x));  // Store position in inches
                                last_time = current_time;  // Update last_time to current time

                                msleep(10);  // Sleep 10ms
                            }
                        }
                        break;
                    case MessageType::GET_ACCEL_Y:
                        {
                            double last_time = seconds();  // Store the initial time
                            double velocity_y = 0.0;  // Initialize velocity to 0
                            double position_y = 0.0;  // Initialize position to 0 (inches)

                            // Conversion: 1024 units = 1G = 386.1 in/sB2
                            // So 1 unit = 386.1 / 1024 b	 0.377 in/sB2
                            const double UNITS_TO_INCHES_PER_S2 = 386.1 / 1024.0;

                            while(nr) {
                                double ay = accel_y() - a_y_bias;  // Read accelerometer data, remove bias
                                double current_time = seconds();  // Get the current time
                                double delta_time = current_time - last_time;  // Calculate the time difference

                                // Convert to inches/sB2
                                double accel_inches = ay * UNITS_TO_INCHES_PER_S2;

                                // First integration: acceleration b velocity
                                velocity_y = velocity_y + accel_inches * delta_time;

                                // Second integration: velocity b position
                                position_y = position_y + velocity_y * delta_time;

                                piy.store(static_cast<double>(position_y));  // Store position in inches
                                last_time = current_time;  // Update last_time to current time

                                msleep(10);  // Sleep 10ms
                            }
                        }
                        break;
                    case MessageType::GET_ACCEL_Z:
                        {
                            double last_time = seconds();  // Store the initial time
                            double velocity_z = 0.0;  // Initialize velocity to 0
                            double position_z = 0.0;  // Initialize position to 0 (inches)

                            // Conversion: 1024 units = 1G = 386.1 in/sB2
                            // So 1 unit = 386.1 / 1024 b	 0.377 in/sB2
                            const double UNITS_TO_INCHES_PER_S2 = 386.1 / 1024.0;

                            while(nr) {
                                double az = accel_z() - a_z_bias;  // Read accelerometer data, remove bias
                                double current_time = seconds();  // Get the current time
                                double delta_time = current_time - last_time;  // Calculate the time difference

                                // Convert to inches/sB2
                                double accel_inches = az * UNITS_TO_INCHES_PER_S2;

                                // First integration: acceleration b velocity
                                velocity_z = velocity_z + accel_inches * delta_time;

                                // Second integration: velocity b position
                                position_z = position_z + velocity_z * delta_time;

                                piz.store(static_cast<double>(position_z));  // Store position in inches
                                last_time = current_time;  // Update last_time to current time

                                msleep(10);  // Sleep 10ms
                            }
                        }
                        break;    

                        // PID
                    case MessageType::SET_PID_GAINS:
                        set_pid_gains(msg.a, msg.b, msg.c, msg.d, msg.e, msg.f, msg.g);
                        break;

                    case MessageType::GET_PID_GAINS: {
                        short p,i,d,pd,id,dd;
                        get_pid_gains(msg.a, &p,&i,&d,&pd,&id,&dd);
                        msg.a = p;
                        msg.b = i;
                        msg.c = d;
                        msg.d = pd;
                        msg.e = id;
                        msg.f = dd;
                        response_queue.push(msg);
                        break;
                    }

                        // PWM
                    case MessageType::SET_PWM:
                        setpwm(msg.a, msg.b);
                        break;

                    case MessageType::GET_PWM:
                        msg.response_int = getpwm(msg.a);
                        response_queue.push(msg);
                        break;

                    default:
                        break;
                }
            }

            message_mutex.unlock();
        }
    }

    ///////////////////////////////////////////////////////////////
    // Messaging Helpers
    ///////////////////////////////////////////////////////////////

    unsigned int send_message(Message msg) {

        message_mutex.lock();

        msg.id = next_message_id++;
        request_queue.push(msg);

        message_mutex.unlock();
        return msg.id;
    }

    Message send_and_wait(Message msg) {

        unsigned int id = send_message(msg);

        while (true) {

            std::this_thread::sleep_for(std::chrono::milliseconds(3));

            message_mutex.lock();

            if (!response_queue.empty()) {

                Message response = response_queue.front();

                if (response.id == id) {
                    response_queue.pop();
                    message_mutex.unlock();
                    return response;
                }
            }

            message_mutex.unlock();
        }
    }

    ///////////////////////////////////////////////////////////////
    // System
    ///////////////////////////////////////////////////////////////

    void start_nano() {
        if (!worker_started) {
            std::cout << "Starting Nano!" << std::endl;
            worker_thread = std::thread(worker_thread_function);
            worker_started = true;
        }
    }

    ///////////////////////////////////////////////////////////////
    // BaseRobot
    ///////////////////////////////////////////////////////////////

    BaseRobot::BaseRobot() {
        start_nano();
    }

    void BaseRobot::wait_for_milliseconds(int ms) {
        Message msg{};
        msg.type = MessageType::WFM;
        msg.a=ms;
        send_message(msg);
    }

    void BaseRobot::calibrate_gyro(){
        Message msg{};
        msg.type = MessageType::CG;
        send_and_wait(msg);  
        printf("cg done");
    }
    void BaseRobot::calibrate_accel_z(){
        Message msg{};
        msg.type = MessageType::CAZ;
        send_message(msg);  
    }
    void BaseRobot::calibrate_accel_y(){
        Message msg{};
        msg.type = MessageType::CAY;
        send_message(msg);  
    }
    void BaseRobot::calibrate_accel_x(){
        Message msg{};
        msg.type = MessageType::CAX;
        send_and_wait(msg);  
        printf("caxx done");
    }


    // Motor
    void BaseRobot::set_motor_power(int m,int p){
        Message msg{};
        msg.type = MessageType::SET_MOTOR_POWER;
        msg.a=m; msg.b=p;
        send_message(msg);
    }

    void BaseRobot::freeze(int m){
        Message msg{};
        msg.type = MessageType::FREEZE;
        msg.a=m;
        send_message(msg);
    }

    int BaseRobot::get_motor_position_counter(int m){
        Message msg{};
        msg.type = MessageType::GET_MOTOR_POSITION;
        msg.a=m;
        return send_and_wait(msg).response_int;
    }

    bool BaseRobot::is_motor_done(int m){
        Message msg{};
        msg.type = MessageType::IS_MOTOR_DONE;
        msg.a=m;
        return send_and_wait(msg).response_bool;
    }

    // Analog/Digital
    int BaseRobot::get_analog(int p){
        Message msg{};
        msg.type = MessageType::GET_ANALOG;
        msg.a=p;
        return send_and_wait(msg).response_int;
    }

    int BaseRobot::get_digital(int p){
        Message msg{};
        msg.type = MessageType::GET_DIGITAL;
        msg.a=p;
        return send_and_wait(msg).response_int;
    }

    void BaseRobot::set_digital(int p,int v){
        Message msg{};
        msg.type = MessageType::SET_DIGITAL;
        msg.a=p; msg.b=v;
        send_message(msg);
    }

    // Gyro
    void BaseRobot::get_gyro_x(){
        Message msg{}; msg.type=MessageType::GET_GYRO_X;
        send_message(msg);
    }
    int BaseRobot::get_gyro_y(){
        Message msg{}; msg.type=MessageType::GET_GYRO_Y;
        return send_and_wait(msg).response_int;
    }
    int BaseRobot::get_gyro_z(){
        Message msg{}; msg.type=MessageType::GET_GYRO_Z;
        return send_and_wait(msg).response_int;
    }

    void BaseRobot::get_accel_x(){
        Message msg{}; msg.type=MessageType::GET_ACCEL_X;
        send_message(msg);
    }
    void BaseRobot::get_accel_y(){
        Message msg{}; msg.type=MessageType::GET_ACCEL_Y;
        send_message(msg);
    }
    void BaseRobot::get_accel_z(){
        Message msg{}; msg.type=MessageType::GET_ACCEL_Z;
        send_message(msg);
    }

    // PWM
    int BaseRobot::getpwm(int m){
        Message msg{}; msg.type=MessageType::GET_PWM; msg.a=m;
        return send_and_wait(msg).response_int;
    }
    void BaseRobot::setpwm(int m,int v){
        Message msg{}; msg.type=MessageType::SET_PWM; msg.a=m; msg.b=v;
        send_message(msg);
    }

} // namespace Nano
