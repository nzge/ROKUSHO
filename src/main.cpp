#include <Arduino.h>

#include <motor.h>
#include <constants.h>

#include <QTRSensors.h>
#include <controller.h>

QTRSensors lf;
Controller controller(Kp, Kd);
Motor motors[] = {Motor(m_pin[0][0], m_pin[0][1]), 
Motor(m_pin[1][0], m_pin[1][1]), 
Motor(m_pin[2][0], m_pin[2][1]), 
Motor(m_pin[3][0], m_pin[3][1])};

// Simple clamping function
template <typename T>
T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) {
    return minVal;
  } else if (value > maxVal) {
    return maxVal;
  } else {
    return value;
  }
}

void setup() {

  Serial.begin(9600);

  digitalWrite(calibration_LED_pin, HIGH);
  for (uint8_t i = 0; i < calibration_iterations; i++)
  {
    delay(20);
  }
  digitalWrite(calibration_LED_pin, LOW);
}



void loop() {

  int left_speed = clamp(base_speed, -max_speed, max_speed);
  int right_speed = clamp(-base_speed, -max_speed, max_speed);

  motors[0].set_speed(left_speed);
  motors[1].set_speed(right_speed);

  Serial.print(left_speed);
  Serial.print(",");
  Serial.println(right_speed);
}


