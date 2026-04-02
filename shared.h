#ifndef SHARED_H
#define SHARED_H
#include <atomic>

#include <mutex>

extern std::atomic<double> orientation;
extern std::atomic<double> pix;
extern std::atomic<double> piy;
extern std::atomic<double> piz;
extern std::atomic<bool> nr;
extern std::atomic<bool> nr2;
extern std::atomic<bool> nr3;
#endif
