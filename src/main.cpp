#include <Arduino.h>

#include <motor.h>
#include <servo.h>
#include <constants.h>

#include <QTRSensors.h>
#include <controller.h>

QTRSensors lf;
Controller base_controller(Kp, Kd);
Motor motors[] = {Motor(m_pin[0][0], m_pin[0][1]), 
Motor(m_pin[1][0], m_pin[1][1]), 
Motor(m_pin[2][0], m_pin[2][1]), 
Motor(m_pin[3][0], m_pin[3][1])};

Servo bar_servo = Servo(s_pin[0][0], s_pin[0][1]); 
Servo claw_servo = Servo(s_pin[1][0], s_pin[1][1]);

bool picked_up = false;

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

void go_forward_for_time(int duration) {
  unsigned long start_time = millis();
  while (millis() - start_time < duration) {
    motors[0].set_speed(base_speed);
    motors[1].set_speed(base_speed);
    motors[2].set_speed(-base_speed);
    motors[3].set_speed(-base_speed);
  }
  motors[0].set_speed(0);
  motors[1].set_speed(0);
  motors[2].set_speed(0);
  motors[3].set_speed(0);
}

void turn_right() {
  uint16_t sensors[num_line_sensors];
  int16_t previous_position = lf.readLineBlack(sensors);
  
  motors[0].set_speed(turn_speed);
  motors[1].set_speed(turn_speed);
  motors[2].set_speed(-turn_speed);
  motors[3].set_speed(-turn_speed);

  bool is_decreasing = true;
  while (is_decreasing) {
    Serial.println("Decreasing position");
    int16_t position = lf.readLineBlack(sensors);

    // Check if the readlineblack value is decreasing
    if (position >= previous_position) {
      is_decreasing = false;
    }
    previous_position = position;
  }

  // Now the average should be around the center, then snap to the right when it hits the line
  while (true) {
    Serial.println("Can't find position");
    int16_t position = lf.readLineBlack(sensors);
    if (position > line_center_position + 3000) {
      break;
    }
  }

  // Now just need to control to be onto the line
  while (true) {
    Serial.println("Final Turn Stretch");
    int16_t position = lf.readLineBlack(sensors);
    if (abs(position - line_center_position) < 500) {
      break;
    }
  }
  motors[0].set_speed(0);
  motors[1].set_speed(0);
  motors[2].set_speed(0);
  motors[3].set_speed(0);
}

void line_follow_until_junction(Controller controller) {
  while (true)
  {
    uint16_t sensors[num_line_sensors];
    int16_t position = lf.readLineBlack(sensors);
    
    // If all sensors see very low reflectance, take some appropriate action
    // for this situation.
    bool allSensorsLowReflectance = true;
    for (int i = 0; i < num_line_sensors; i++) {
      if (sensors[i] > 750) {
        allSensorsLowReflectance = false;
        break;
      }
    }
    
    if (allSensorsLowReflectance) return;

    int error = position - line_center_position;
    float controller_output = controller.update(error);

    int left_speed = clamp(base_speed + controller_output, -max_speed, max_speed);
    int right_speed = clamp(base_speed - controller_output, -max_speed, max_speed);

    motors[0].set_speed(-left_speed);
    motors[1].set_speed(-left_speed);
    motors[2].set_speed(-right_speed);
    motors[3].set_speed(right_speed);

    Serial.print(position);
    Serial.print(",");
    Serial.println(controller_output);
  }
}

// void setup() {
//   Serial.begin(9600);
//   lf.setTypeAnalog();
//   lf.setSensorPins(line_follower_pins, num_line_sensors);

//   digitalWrite(calibration_LED_pin, HIGH);
//   for (uint8_t i = 0; i < calibration_iterations; i++)
//   {
//     lf.calibrate();
//     delay(20);
//     Serial.println(i);
//   }
//   digitalWrite(calibration_LED_pin, LOW);
// }

// void loop() {
//   line_follow_until_junction(base_controller);
// }


#define LIMIT_SWITCH_PIN 2 

void setup() {
  
  bar_servo.set_angle(bar_servoPos);
  claw_servo.set_angle(claw_servoPos);

  Serial.begin(9600);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

}


int SWEEP_DELAY = 10;

bool switchPressed = false; // Flag to track switch state change

int bar_traj[3]= {90, 90, 180}; 
int bar_servoPos = 0;

int claw_traj[4]= {90, 90, 90, 118}; 
int claw_servoPos = 0;


void grabDisc(){
   
  //grab patty
  for (int angle = claw_traj[1]; angle <= claw_traj[2]; angle++) {
    claw_servo.set_angle(angle);   // Set servo angle
    delay(SWEEP_DELAY);     // Wait before changing angle
  }

  delay(500);

  //bring bar towards platform
  for (int angle = bar_traj[1]; angle <= bar_traj[2]; angle++) {
    bar_servo.set_angle(angle);   // Set servo angle
    delay(SWEEP_DELAY);     // Wait before changing angle
  }
  
  delay(500); 

  //release patty
  for (int angle = claw_traj[2]; angle >= claw_traj[1]; angle--) {
    bar_servo.set_angle(angle);   // Set servo angle
    delay(SWEEP_DELAY);     // Wait before changing angle
  }

  delay(500);

  //bring bar back
  for (int angle = bar_traj[2]; angle >= bar_traj[1]; angle--) {
    bar_servo.set_angle(angle);   // Set servo angle
    delay(SWEEP_DELAY);     // Wait before changing angle
  }

}

void dropoffDisc(){
    //bring bar towards patty
    for (int angle = bar_traj[1]; angle <= bar_traj[2]; angle++) {
      bar_servo.set_angle(angle);   // Set servo angle
      delay(SWEEP_DELAY);     // Wait before changing angle
    }
    
    delay(500);

    //grab patty
    for (int angle = claw_traj[1]; angle <= claw_traj[2]; angle++) {
      claw_servo.set_angle(angle);   // Set servo angle
      delay(SWEEP_DELAY);     // Wait before changing angle
    }

    delay(500);

    //bring bar back
    for (int angle = bar_traj[2]; angle >= bar_traj[1]; angle--) {
      bar_servo.set_angle(angle);   // Set servo angle
      delay(SWEEP_DELAY);     // Wait before changing angle
    }
    
    delay(500);

    //release patty
    for (int angle = claw_traj[2]; angle >= claw_traj[1]; angle--) {
      bar_servo.set_angle(angle);   // Set servo angle
      delay(SWEEP_DELAY);     // Wait before changing angle
    }
}

void loop() {
  
  int limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);

  // Check if the switch is pressed and the flag is not set
  if (limitSwitchState == LOW && !switchPressed) {
    
    unsigned long time_prev0 = millis();

    //backup from wall
    while(millis() - time_prev0>=100) {
      motors[0].set_speed(-base_speed);
      motors[1].set_speed(-base_speed);
      motors[2].set_speed(base_speed);
      motors[3].set_speed(base_speed);
    }
    switchPressed = true;
  }

  if (limitSwitchState == HIGH && switchPressed) {

    grabDisc();
    
  }
}
