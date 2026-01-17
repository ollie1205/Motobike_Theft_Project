void setup() {
  Serial.begin(115200);
  
  // Waits for serial monitor
  while (!Serial) { 
    delay(10); 
  }

  Serial1.begin(9600); 
  Serial.println("GPS Test Started...");
}

void loop() {
  //Reads from GPS, sends to serial monitor
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
