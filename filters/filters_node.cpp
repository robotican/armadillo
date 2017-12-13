
#include <iostream>
#include "include/filters/low_pass_filter.h"
using namespace std;

int main(int argc, char** argv){
    //Create a low pass filter with 1 hz cuttoff freqency. DetltaTime for each cycle equals 0.01 seonds
    LowPassFilter lpf(1.0, 0.01);
    //Cycles 500 times. With the lpf deltaTime set to 0.01 it will simulate 5 seconds of run time
    for(int i = 0; i < 500; i++){
        cout << lpf.update(1.0) << endl; //Update with 1.0 as input value.
    }
    return 1;
}