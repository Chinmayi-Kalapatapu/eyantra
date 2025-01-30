// Function Name: getCurrentPosition()
// Input: none
// Output: Current position of the robot (average distance traveled in mm).
// Logic: Calculate average travel distance using encoder pulse counts for both wheels.
// Example Call: getCurrentPosition();
float getCurrentPosition() {
  float left_wheel_rotations = wheel_pulse_count_left / (float)ENCODER_PULSES_PER_REVOLUTION; // Left wheel rotations
  float right_wheel_rotations = wheel_pulse_count_right / (float)ENCODER_PULSES_PER_REVOLUTION; // Right wheel rotations

  float left_wheel_distance = 2 * 3.14159 * WHEEL_RADIUS * left_wheel_rotations; // Left wheel distance
  float right_wheel_distance = 2 * 3.14159 * WHEEL_RADIUS * right_wheel_rotations; // Right wheel distance

  return (left_wheel_distance + right_wheel_distance) / 2.0; // Average distance
}

// Function Name: controlMotors
// Input: pidOutput - The output of the PID controller, which determines the motor speed and direction.
// Output: None
// Logic: Sets the motor speed and direction based on the PID output. If pidOutput is positive, motors move forward; if negative, motors move backward. Stops the motors if pidOutput is zero.
// Example Call: controlMotors(pidOutput);
void controlMotors(float pidOutput) {
  int motorSpeed = constrain(abs(pidOutput), 0, 100); // Ensure speed is in valid PWM range

  if (pidOutput > 0) {
    // Tilted forward, move motors forward
    forward(motorSpeed);
  } 
  else if (pidOutput < 0) {
    // Tilted backward, move motors backward
    backward(motorSpeed);
  } 
  else {
    // Stop motors when stable
    stop();
  }
}

// Function Name: forward
// Input: motorSpeed - The speed at which the motors should move forward (range: 0 to 255).
// Output: None
// Logic: Moves both wheels forward at the specified speed.
// Example Call: forward(motorSpeed);
void forward(int motorSpeed) {
  analogWrite(EN1, motorSpeed);
  analogWrite(EN2, motorSpeed);
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
}

// Function Name: backward
// Input: motorSpeed - The speed at which the motors should move backward (range: 0 to 255).
// Output: None
// Logic: Moves both wheels backward at the specified speed.
// Example Call: backward(motorSpeed);
void backward(int motorSpeed) {
  analogWrite(EN1, motorSpeed);
  analogWrite(EN2, motorSpeed);
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
} 

// Function Name: stop
// Input: None
// Output: None
// Logic: Stops both wheels by setting the enable pins to 0 and turning off the control pins.
// Example Call: stop();
void stop() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, LOW);
}
 
// New controller
float controller_simple_why_complicate_life(){
  unsigned long currentTime = millis();
  current_position = getCurrentPosition();
  Serial.println("current_position: " + String(current_position, 2));

  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert ms to seconds
  if (deltaTime <= 0) deltaTime = 0.001;

  float tilt_error = tilt_offset - pitch; // Error in tilt
  tilt_error_print = tilt_error;
  integral_tilt += tilt_error * deltaTime; // Integral term for tilt
  float derivative_tilt = (tilt_error - previousTiltError) / deltaTime; // Derivative term for tilt
  float PID_tilt_control = kp_tilt * tilt_error + ki_tilt * integral_tilt + kd_tilt * derivative_tilt; // Inner loop

  previousTiltError = tilt_error;
  previousTime = currentTime;
  float velocity_error = (desired_pos - current_position) / deltaTime;
  float pos_error = (desired_pos - current_position);
  PID_tilt_control = PID_tilt_control + velocity_error*kd_pos + pos_error*kp_pos;
  if ((pitch > 20) || (pitch < -25)){
        PID_tilt_control = 0;
        integral_tilt = 0;
  }

  return PID_tilt_control;
}
