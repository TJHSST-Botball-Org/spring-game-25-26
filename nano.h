/*
Nano is botball library. extends libkipr, providing wrapper functions and  extra functionality
thread safe

you gotta be familiar with object oriented programming to an extent, and basic
programming to understand this 

docs will help you the rest of the way
*/

#pragma once

#include <kipr/wombat.h>
#include <mutex>
#include <thread>
#include <queue>
#include <tuple>
#include <iostream>

namespace Nano {

    // Start the Nano library (spawns worker thread)
    void start_nano();

    // Millisecond delay
    void wait_for_milliseconds(int milliseconds);

    // Message system for thread-safe commands
    enum class MessageType {
        GET_MOTOR_POSITION,
        CLEAR_MOTOR_POSITION,
        SET_MOTOR_POWER,
        MOVE_AT_VELOCITY,
        MOVE_TO_POSITION,
        MOVE_RELATIVE_POSITION,
        IS_MOTOR_DONE,
        FREEZE_MOTOR,
        GET_ANALOG,
        GET_DIGITAL,
        SET_DIGITAL
    };

    class BaseRobot {
    public:
        BaseRobot();

        // Motors
        int get_motor_position_counter(int motor);
        void clear_motor_position_counter(int motor);
        void set_motor_power(int motor, int percent);
        void move_at_velocity(int motor, int velocity);
        void move_to_position(int motor, int speed, int goal_ticks);
        void move_relative_position(int motor, int speed, int delta_ticks);
        bool is_motor_done(int motor);
        void freeze(int motor);

        // Analog/Digital
        int get_analog(int port);
        int get_digital(int port);
        void set_digital(int port, int value);
    };

    // Simple Mutex wrapper
    class Mutex {
    public:
        Mutex() = default;
        ~Mutex() = default;
        void lock();
        void unlock();
    private:
        std::mutex _m;
        friend class Thread;
    };

    // Simple Thread wrapper
    class Thread {
    public:
        template<typename Func>
        Thread(Func function, bool start_automatically = true);

        void start();
        void wait_for_thread();
        void stop();

    private:
        std::thread _t;
        bool _started = false;
    };

} // namespace Nano
