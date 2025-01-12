#include <Servo.h>

// Create a servo object
Servo myServo;
Servo myServo2;

// Define pin for the servo motor
const int servoPin = 11;
const int servoPin2 = 10;

// Initial servo position
int servoPosition = 45; // Center position
int servoPosition2 = 45;

void setup() {
  myServo.attach(servoPin);
  myServo2.attach(servoPin2);
  myServo.write(servoPosition);
  myServo2.write(servoPosition2);
  Serial.begin(9600);
}

void loop() {
  // Check if there is any input from the serial monitor
  if (Serial.available()) {
    // Read the command
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any trailing whitespace

    if (command == "o") {
      // Move clockwise
      if (servoPosition < 180) {
        servoPosition = 75; // Increase position
        myServo.write(servoPosition);
        Serial.print("Servo position: ");
        Serial.println(servoPosition);
      } else {
        Serial.println("Servo is already at the maximum clockwise position.");
      }
    } else if (command == "c") {
      // Move anticlockwise
      if (servoPosition > 0) {
        servoPosition = 35; // Decrease position
        myServo.write(servoPosition);
        Serial.print("Servo position: ");
        Serial.println(servoPosition);
      } else {
        Serial.println("Servo is already at the maximum anticlockwise position.");
      }
    } else {
      Serial.println("Invalid command. Use 'cw' for clockwise or 'ccw' for anticlockwise.");
    }

    if (command == "u") {
        // Move clockwise
        if (servoPosition2 < 180) {
          servoPosition2 += servoPosition2; // Increase position
          myServo2.write(servoPosition2);
          Serial.print("Servo position: ");
          Serial.println(servoPosition2);
        } else {
          Serial.println("Servo is already at the maximum clockwise position.");
        }
      } else if (command == "d") {
        // Move anticlockwise
        if (servoPosition2 > 0) {
          servoPosition2 -= servoPosition2; // Decrease position
          myServo2.write(servoPosition2);
          Serial.print("Servo position: ");
          Serial.println(servoPosition2);
        } else {
          Serial.println("Servo is already at the maximum anticlockwise position.");
        }
      } else {
        Serial.println("Invalid command. Use 'cw' for clockwise or 'ccw' for anticlockwise.");
      }
  }

  // Add a short delay to allow the servo to reach the position
  delay(100);
}
