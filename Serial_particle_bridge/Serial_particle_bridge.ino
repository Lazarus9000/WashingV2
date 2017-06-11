const byte numChars = 500;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false;

void setup() {
 Serial.begin(115200);
 pinMode(1, OUTPUT); 
 Serial.println("<Arduino is ready>");
}

void loop() {
 recvWithEndMarker();
 showNewData();
}

void recvWithEndMarker() {
 static byte ndx = 0;
 char endMarker = '\n';
 char endMarker2 = '\r';
 char rc;
 
 // if (Serial.available() > 0) {
           while (Serial.available() > 0 && newData == false) {
 rc = Serial.read();

 if (rc != endMarker && rc != endMarker2) {
 receivedChars[ndx] = rc;
 ndx++;
 if (ndx >= numChars) {
 ndx = numChars - 1;
 }
 }
 else {
 receivedChars[ndx] = '\0'; // terminate the string
 ndx = 0;
 newData = true;
 }
 }
}

void showNewData() {
 if (newData == true) {
 Serial.print("This just in ... ");
 Serial.println(receivedChars);
 newData = false;
 
 //Consider replacing the static function name with something that can be set from the connected serial device
 
 Particle.publish("writeDB", receivedChars, 60, PRIVATE);
   digitalWrite(1, HIGH);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is acive low on the ESP-01)
  delay(10);                      // Wait for a second
  digitalWrite(1, LOW);  // Turn the LED off by making the voltage HIGH
  memset(receivedChars, 0, sizeof(receivedChars));
 }
}
