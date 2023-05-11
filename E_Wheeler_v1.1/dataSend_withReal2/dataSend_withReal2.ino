
char msg[40];

char V0[10];
char I0[10];
char An0[10];
char temp[10];
String raw1,raw2,raw3;


float I_value = 0;
float V_value = 0;
float I_value_sum;
float V_value_sum;
int count = 0;

void setup() {
  pinMode( 7, OUTPUT);
  Serial.begin(9600);  
  delay(500);
  
}

void loop() {

  
  for (int i=0; i<10; i=i+1) {
    int sensorValue = analogRead(A2);
    float val1 = sensorValue * (5.0 / 1023.0);
    //Serial.println( sensorValue);
    
    int sensorValue2 = analogRead(A1);
    float val2 = sensorValue2 * (5.0 / 1023.0);
    //Serial.println( sensorValue2);
    V_value_sum = V_value_sum + sensorValue2 * 0.08151;
    
    I_value_sum = I_value_sum + (val1 - 1.48)*1000/16;
    //Serial.print((val1 - 1.636)*1000/16);
    //Serial.print(" ");
    
    count = count +1;
    
    delay(100);
  }
  

  if ( count == 20) {
    I_value = I_value_sum / 20 ;
    V_value = V_value_sum / 20 ;
    if ( I_value < 0) {
      I_value = 0.0;
    }
    //Serial.println( I_value);
    I_value_sum = 0;
    V_value_sum = 0;
    count = 0;
  }
  
  int sensorValue3 = analogRead(A3);
  float val3 = sensorValue3 * (5.0 / 1023.0);


  
  
  dtostrf(I_value,5,3,I0);
  dtostrf(V_value,5,3,V0); 
  dtostrf(val3,5,3,temp);
//  dtostrf(val4,5,3,An0);

//  temp = "57.4";
//  An0 = "5.34";
//  R0 = "36";
  
  //sprintf(msg, "%s;%s;%s;%s;","60.00","10.00","50.00",An0);
  sprintf(msg, ";%s;%s;%s;",V0,I0,"57");
  Serial.println(msg);

}
