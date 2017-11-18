//
// Created by Eli Eli on 18/11/2017.
//

#include "timer.h"

void Timer::reset()
{
    std::chrono::steady_clock::time_point zero;
    begin_ = zero;
    end_ = zero;
    micro_secs_ = 0;
    started_ = false;
}


void Timer::startMeasure()
{
    begin_ = std::chrono::steady_clock::now();
}
void Timer::endMeasure()
{
    end_ = std::chrono::steady_clock::now();
}

void Timer::startTimer(int micro_secs)
{
    if (!started_)
    {
        micro_secs_ = micro_secs;
        startMeasure();
        started_ = true;
    }
}

bool Timer::isFinished()
{
    endMeasure();
    if (elapsedTimeMs() >= micro_secs_)
    {
        started_ = false;
        return true;
    }
    return false;
}

long long int Timer::elaspedTimeSec()
{
    return elapsedTimeMs() / 1000;
}
long long int Timer::elapsedTimeMs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count();
}
long long int Timer::elapsedTimeNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_ - begin_).count();
}
