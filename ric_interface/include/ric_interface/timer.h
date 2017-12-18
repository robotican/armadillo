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
    int trigger_millis_;
    bool started_ = false;

public:
    void reset() { started_ = false; }
    void startTimer(int trigger_millis)
    {
        if (!started_)
        {
            trigger_millis_ = trigger_millis;
            started_ = true;
            begin_ = std::chrono::steady_clock::now();
        }
    }
    bool isFinished()
    {
        end_ = std::chrono::steady_clock::now();
        long long int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - begin_).count();
        if (elapsed >= trigger_millis_)
        {
            started_ = false;
            return true;
        }
        return false;
    }
};

#endif //RIC_INTERFACE_TIMER_H
