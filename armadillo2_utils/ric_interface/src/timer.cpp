/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#include <ric_interface/timer.h>

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
    if (elapsedTimeMilliSec() >= micro_secs_)
        return true;
    return false;
}

long long int Timer::elaspedTimeSec()
{
    return std::chrono::duration_cast<std::chrono::seconds>(end_ - begin_).count();
}
long long int Timer::elapsedTimeMilliSec()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_ - begin_).count();
}
long long int Timer::elapsedTimeNanoSec()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_ - begin_).count();
}

long long int Timer::elapsedTimeMicroSec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count();

}
