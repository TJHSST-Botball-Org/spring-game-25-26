#include "nano.h"
#include <queue>
#include <tuple>
#include <iostream>

namespace Nano {

// Forward declaration of worker thread
void worker_thread_function();

// Thread-safe queues
std::queue<std::tuple<unsigned int, MessageType, std::tuple<int,int>>> request_message_queue;
std::queue<std::tuple<unsigned int, int>> response_message_queue;
unsigned int next_message_id = 0;
Mutex message_mutex;

// Worker thread
Thread worker_thread(worker_thread_function, false);
bool has_already_started_worker_thread = false;

// Worker thread implementation
void worker_thread_function() {
    while (true) {
        wait_for_milliseconds(2);
        message_mutex.lock();

        if (!request_message_queue.empty()) {
            auto request = request_message_queue.front();
            request_message_queue.pop();

            unsigned int id = std::get<0>(request);
            MessageType type = std::get<1>(request);
            auto content_tuple = std::get<2>(request); 
            int motor_port = std::get<0>(content_tuple);
            int value = std::get<1>(content_tuple);

            switch(type) {
                case MessageType::SET_MOTOR_POWER:
                case MessageType::FREEZE_MOTOR:
                    motor(motor_port, value); // value = 0 for freeze
                    break;

                // Add more cases here for other motor/servo/analog/digital messages
                default:
                    break;
            }
        }

        message_mutex.unlock();
    }
}

// Messaging helpers
unsigned int send_message(MessageType type, int port, int value=0) {
    message_mutex.lock();
    unsigned int msg_id = next_message_id++;
    request_message_queue.push(std::make_tuple(msg_id, type, std::make_tuple(port, value)));
    message_mutex.unlock();
    return msg_id;
}

// Public API
void start_nano() {
    if (!has_already_started_worker_thread) {
        std::cout << "Starting Nano!" << std::endl;
        worker_thread.start();
        has_already_started_worker_thread = true;
    }
}

void wait_for_milliseconds(int ms) {
    msleep(ms);
}

// BaseRobot methods
BaseRobot::BaseRobot() {
    start_nano();
}

void BaseRobot::set_motor_power(int motor, int power) {
    send_message(MessageType::SET_MOTOR_POWER, motor, power);
}

void BaseRobot::freeze(int motor) {
    send_message(MessageType::FREEZE_MOTOR, motor, 0); // send value=0
}

// Mutex implementation
void Mutex::lock() {
    _m.lock();
}

void Mutex::unlock() {
    _m.unlock();
}

// Thread implementation
template<typename Func>
Thread::Thread(Func function, bool start_automatically) {
    _t = std::thread(function);
    if (start_automatically) start();
}

void Thread::start() {
    if (!_started) _started = true;
}

void Thread::wait_for_thread() {
    if (_t.joinable()) _t.join();
}

void Thread::stop() {
    if (_t.joinable()) _t.detach();
}

} // namespace Nano
