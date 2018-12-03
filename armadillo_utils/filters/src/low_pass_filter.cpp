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
/* Author: Elchay Rauper*/

#include "../include/filters/low_pass_filter.h"

LowPassFilter::LowPassFilter():
        output(0),
        cutOffFrequency(0){}

LowPassFilter::LowPassFilter(float iCutOffFrequency):
        output(0),
        cutOffFrequency(iCutOffFrequency),
        ePow(0){}

LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime):
        output(0),
        cutOffFrequency(iCutOffFrequency),
        ePow(1-exp(-iDeltaTime * iCutOffFrequency)){}

float LowPassFilter::update(float input){
    return output += (input - output) * ePow;
}

float LowPassFilter::update(float input, float deltaTime){
    setDeltaTime(deltaTime); //Changes ePow accordingly.
    return output += (input - output) * ePow;
}

float LowPassFilter::getOutput(){
    return output;
}

float LowPassFilter::getCutOffFrequency(){
    return cutOffFrequency;
}

void LowPassFilter::setCutOffFrequency(float input){
    if (input >= 0){
        cutOffFrequency = input;
    }
    else{
        cutOffFrequency = 0;
        //std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
    }
}

void LowPassFilter::setDeltaTime(float deltaTime){
    if (deltaTime >= 0){
        ePow = 1-exp(-deltaTime * cutOffFrequency);
    }
    else{
        ePow = 0;
        //std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
    }
}


