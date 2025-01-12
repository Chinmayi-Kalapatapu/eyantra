#include "constants.h"


void setup() {
  initializeMotors();
  setupEncoder();

  Serial.begin(9600); // For debugging
  bluetooth.begin(9600);     // For HC-05 (default baud rate is 9600)
  Serial.println("Staring Serial Print");
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmissio
}

void loop() {
  read_imu();
  //   float pidOutput = computePID(pitch); // Compute PID output
  float pidOutput = controller_simple_why_complicate_life();
  bluetooth_send();

  
  controlMotors(pidOutput); // Control motors based on PID output

  // Debugging
  // Serial.print("tilt Angle: ");
  // Serial.print(tilt_angle);
  // Serial.print(" | roll Angle: ");
  // Serial.print(roll);
  // Serial.print(" | yaw Angle: ");
  // Serial.print(yaw);
  Serial.print(" | pitch Angle: ");
  Serial.println(pitch);
  Serial.print(" | tilt error: ");
  Serial.print(tilt_error_print);
  Serial.print(" | PID Output: ");
  Serial.println(pidOutput);

  // Write data to CSV format via serial
  //write_csv();
}