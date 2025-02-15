  SoftwareSerial bluetooth(A1, A0); // RX, TX


// Check for data from Bluetooth
  if (bluetooth.available()) {
    char btData = bluetooth.read();
    Serial.print("Received from Bluetooth: ");
    Serial.println(btData);
  }

  // Check for data from Serial Monitor
  if (Serial.available()) {
    char serialData = Serial.read();
    bluetooth.print(serialData); // Send data to Bluetooth
    Serial.print("Sent to Bluetooth: ");
    Serial.println(serialData);
  }
