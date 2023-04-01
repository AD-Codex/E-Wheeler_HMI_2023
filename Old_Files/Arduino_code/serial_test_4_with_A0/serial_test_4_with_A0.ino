
int current_read = A0;
int voltage_read = A1;

char msg[40];

char V0[10];
char I0[10];

char temp1;
String raw1,raw2,raw3;

//test paramater Values
//float val1=5.04; 
//float val2=25.6; //voltage


void setup() {
  pinMode( current_read, INPUT);
  pinMode( voltage_read, INPUT);
  Serial.begin(9600);
  
}

void loop() {
  int sensorValue = analogRead(current_read);
  float val1 = sensorValue * (5.0 / 1023.0);
  int sensorValue2 = analogRead(voltage_read);
  float val2 = sensorValue2 * (25.0 / 1023.0);
  
  dtostrf(val1,5,3,I0);
  //Serial.println(msg);
  dtostrf(val2,5,3,V0);
  //Serial.println(msg);

  
  //sprintf(msg, "%s;%s",V0,I0);
  sprintf(msg, "%s;%s", "20.52", "1.76");
  
  Serial.println(msg);
  delay(1000);
}
