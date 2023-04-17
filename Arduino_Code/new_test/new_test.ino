
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

float I_value = 0;
float V_value = 0;
float I_value_sum;
float V_value_sum;
int count = 0;
int cycle_count = 0;
int SdelayC = 0;
float ccount;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), PulseP_Event, FALLING);
  
  delay(500);
  
}

void loop() {

  for (int i=0; i<10; i=i+1) {
    int sensorValue = analogRead(A0);
    float val1 = sensorValue * (5.0 / 1023.0) + 0.03;
    int sensorValue2 = analogRead(A1);
    float val2 = sensorValue2 * (5.0 / 1023.0);
    
    I_value_sum = I_value_sum + (val1 - 1.664)*1000/16;
    V_value_sum = V_value_sum + val2*15.88;
    count = count +1;
    delay(100);
  }
  SdelayC = SdelayC +1;

  if ( count == 20) {
    I_value = I_value_sum / 20 - 7;
    V_value = V_value_sum / 20 - 4.2;
    I_value_sum = 0;
    V_value_sum = 0;
    count = 0;
  }
  
  int sensorValue3 = analogRead(A2);
  float val3 = sensorValue3 * (5.0 / 1023.0);
  int sensorValue4 = analogRead(A3);
  float val4 = sensorValue4 * (5.0 / 1023.0);

//  if ( SdelayC == 5) {/
//    ccount = (cycle_count * 1.38*3.6)/5;/
//    cycle_count = 0.00;
//    SdelayC = 0;
//  }

  
  
  dtostrf(I_value,5,3,I0);
  dtostrf(V_value,5,3,V0); 
  dtostrf(val3,5,3,temp);
  dtostrf(val4,5,3,An0);
  dtostrf(cycle_count,5,3,R0);

//  temp = "57.4";
//  An0 = "5.34";
//  R0 = "36";
  
//  sprintf(msg, "%s;%s;%s;%s;%s;",V0,I0,temp,An0,R0);
  sprintf(msg, "%s;%s;%s;%s;%s;",V0,I0,"57","4.34",R0);
  Serial.println(msg);

}


void PulseP_Event() {
  cycle_count = cycle_count + 1;
}
