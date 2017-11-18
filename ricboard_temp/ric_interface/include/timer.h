//
// Created by Eli Eli on 18/11/2017.
//

#ifndef RIC_INTERFACE_TIMER_H
#define RIC_INTERFACE_TIMER_H

#include <chrono>

class Timer
{
private:
    std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point end_;
    int micro_secs_;
    bool started_;

public:
    void reset();
    void startMeasure();
    void endMeasure();
    void startTimer(int micro_secs);
    bool isFinished();
    long long int elaspedTimeSec();
    long long int elapsedTimeMs();
    long long int elapsedTimeNs();
};

#endif //RIC_INTERFACE_TIMER_H
