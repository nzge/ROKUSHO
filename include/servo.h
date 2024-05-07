#ifndef SERVO_H
#define SERVO_H

class Servo {
    public:
        Servo(int pwm_pin, int angle_pin);
        Servo(int pwm_pin);
        void set_angle(int angle);
        float measure_angle();
        float get_set_angle();

    private:
        int current_angle;
        int _angle_pin;
        int _pwm_pin;
};
#endif