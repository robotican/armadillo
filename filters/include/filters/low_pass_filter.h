
#ifndef FILTERS_LOW_PASS_FILTER_H
#define FILTERS_LOW_PASS_FILTER_H

#include <cmath>

class LowPassFilter
{
public:
    //constructors
    LowPassFilter();
    LowPassFilter(float iCutOffFrequency);
    LowPassFilter(float iCutOffFrequency, float iDeltaTime);
    //functions
    float update(float input);
    float update(float input, float deltaTime);
    //get and set funtions
    float getOutput();
    float getCutOffFrequency();
    void setCutOffFrequency(float input);
    void setDeltaTime(float input);
private:
    float output;
    float cutOffFrequency;
    float ePow;
};


#endif //FILTERS_LOW_PASS_FILTER_H
