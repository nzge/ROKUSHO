#include <Arduino.h>
#include <constants.h>
#include <servo.h>

Servo::Servo(int pwm_pin, int angle_pin)
{
    _pwm_pin = pwm_pin;
    _angle_pin = angle_pin;
    pinMode(_pwm_pin, OUTPUT);
    pinMode(_angle_pin, INPUT);
}

Servo::Servo(int pwm_pin)
{
    _pwm_pin = pwm_pin;
    _angle_pin = -1;
    pinMode(_pwm_pin, OUTPUT);
}

void Servo::set_angle(int angle)
{
    int pwm_command = map(angle, 0, 180, 0, 255);
    analogWrite(_pwm_pin, pwm_command);
}

float Servo::measure_angle()
{
    if (_angle_pin != -1) {
        int angle = analogRead(_angle_pin);
        return map(angle, 0, 1023, 0, 180);
    } else {
        // Return some default value or handle the case where angle measurement is not possible
        return -1.0; 
    }
}


float Servo::get_set_angle()
{
    return current_angle;
}