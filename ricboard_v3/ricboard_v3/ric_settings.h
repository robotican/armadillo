#ifndef RIC_SETTINGS_H
#define RIC_SETTINGS_H

#define SRVO_NEUTRAL 1500
#define SRVO_MIN 1000
#define SRVO_MAX 2000

#define INDICATOR_LED 13
#define BAUDRATE 500000//115200
#define SEND_KA_INTERVAL 300 //ms
#define GET_KA_INTERVAL 1000 //ms
#define SEND_READINGS_INTERVAL 30 //ms
#define UTLRASONICE_INTERVAL 90 //ms - this is the fastest update rate of urf according to datasheet
#define SERVO_PIN 3
#define ULTRASONIC_PIN 14

#endif //RIC_SETTINGS_H
