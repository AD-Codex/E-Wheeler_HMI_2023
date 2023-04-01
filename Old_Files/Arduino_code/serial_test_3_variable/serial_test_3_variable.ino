
char msg[40];

char V0[10];
char I0[10];

char temp1;
String raw1,raw2,raw3;

//test paramater Values
float val1=5.04; 
float val2=25.6; //voltage


void setup() {
  Serial.begin(9600);
  
}

void loop() {
  dtostrf(val1,5,3,I0);
  //Serial.println(msg);
  dtostrf(val2,5,3,V0);
  //Serial.println(msg);

  
  sprintf(msg, "%s;%s",V0,I0);
  //Serial.println("test");
  Serial.println(msg);
  delay(1000);
}
