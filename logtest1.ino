int value = 877;
char buffer[5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


Serial.print("#S|LOGTEST|[");
Serial.print(itoa((value), buffer, 10));
Serial.println(",1234,12,754]#");

}

void loop() {
  // put your main code here, to run repeatedly:
  value = analogRead(A0);
Serial.print("#S|LOGTEST|[");
Serial.print(itoa((value), buffer, 10));
Serial.println(",1234,12,754]#");
}
