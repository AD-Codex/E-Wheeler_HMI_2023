void setup() {
  // put your setup code here, to run once:
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(A0 , 120);
  analogWrite(A1 , 150);

}
