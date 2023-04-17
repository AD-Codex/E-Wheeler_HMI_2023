
char msg[40];

char V0[10];
char I0[10];
char R0[10];
char An0[10];
char temp[10];
String raw1,raw2,raw3;

//test paramater Values
//float val1=5.04; 
//float val2=25.6; //voltage


void setup() {
  Serial.begin(9600);
  
}

void loop() {
  // current read
  int sensorValue = analogRead(A0);
  float val1 = sensorValue * (5.0 / 1024.0);
  
  int sensorValue2 = analogRead(A1);
  float val2 = sensorValue2 * (5.0 / 1024.0);
  
  int sensorValue3 = analogRead(A2);
  float val3 = sensorValue3 * (5.0 / 1023.0);
  
  int sensorValue4 = analogRead(A3);
  float val4 = sensorValue4 * (5.0 / 1023.0);
  
  int sensorValue5 = analogRead(A4);
  float val5 = sensorValue5 * (5.0 / 1023.0);
  
  dtostrf(val1,5,3,I0);
  //Serial.println(msg);
  dtostrf(val2,5,3,V0);
  //Serial.println(msg);  
  dtostrf(val3,5,3,temp);
  dtostrf(val4,5,3,An0);
  dtostrf(val5,5,3,R0);

//  temp = "57.4";
//  An0 = "5.34";
//  R0 = "36";
  
  //sprintf(msg, "%s;%s;%s;%s;%s;",V0,I0,temp,An0,R0);
  sprintf(msg, "%s;%s;%s;%s;%s;",V0,I0,"57","4.34","34634");
  //sprintf(msg, "voltage %s , current %s", V0, I0);
  //sprintf(msg, "%s;%s",0.012,0.024);
  //Serial.println("test");
  Serial.println(msg);
  delay(1000);
}
