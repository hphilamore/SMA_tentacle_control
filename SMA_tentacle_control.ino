#include <math.h>    // (no semicolon)

int t1 = 0;   // declares variable t1
int t2 = 0;   // declares variable t2

int incomingByte = 0; // for incoming serial data
const int BUFFER_SIZE = 3;
char buf[BUFFER_SIZE];

const int BUFFER_SIZE_ = 2;
char buf_[BUFFER_SIZE_];

bool val;
int n_bytes;

int pwm_v = 120; 

double angle;

//int A_on = A1;
//int A_pump = 12;
//int A_valve = 11;

static const uint8_t button_pins[] = {A1,A2,A3};
int valve_pins[] = {12, 10, 6};
int pump_pins[] = {11, 9, 5};  
//int button_vals_old[] = {0, 0, 0};
int button_vals[3];// = {0, 0, 0};

int pos0[3] = {0, 0, 0};
int pos1[3] = {1, 0, 0};
int pos2[3] = {1, 1, 0};
int pos3[3] = {0, 1, 0};
int pos4[3] = {0, 1, 1};
int pos5[3] = {0, 0, 1};
int pos6[3] = {1, 0, 1};

int* pos = pos0;



void setup() {
  //Serial.begin(9600);
  Serial.begin(57600);

  // Setup pins for input
  //  pinMode(A_on, INPUT); 
  //  pinMode(A_pump, OUTPUT);
  //  pinMode(A_valve, OUTPUT); 
  for (int i = 0; i < 3; i++) {
    pinMode(button_pins[i], INPUT);
    pinMode(valve_pins[i], OUTPUT);
    pinMode(pump_pins[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++) {
    button_vals[i] = digitalRead(button_pins[i]); 
  }
  
  t2= 50; //50; //100; //500;                     // duty cycle on (lower = faster deflate)
  t1= 1000-t2;                 // duty cycke off
}

void loop() {

  //button_control_pwm();
  //button_control();
  //serial_control_atan2();
  serial_control_SMA();
  //atan2_test();
  //deflate_all();

 }





//void basic_pump(){
//   // control of a single pump basic on and off
//    if(digitalRead(A_on) == HIGH){ 
//      digitalWrite(A_pump, HIGH); 
//      Serial.println("high");
//      }
//    else{
//     digitalWrite(A_pump, LOW); 
//     } 
//}
  

//void basic_chamber(){
//   control of a single pump basic on and off
//  while(digitalRead(A_on) == HIGH){   // when button pressed...
//      digitalWrite(A_valve,LOW);      // normally open valve = open, (PNP transistor)
//      digitalWrite(A_pump, HIGH);     // switch pump on
//      Serial.println("inflate");
//      }
//  
//   // when button released...
//  Serial.println("deflate");
//  digitalWrite(A_valve, HIGH);  // normally open valve = closed, (PNP transistor)
//  digitalWrite(A_pump, LOW);    // switch pump off
//}


//void button_control_pwm(){
//  // control of a single pneumatic chamber with adjustable speed using PWM
// 
//  while(digitalRead(A_on) == HIGH){   // when button pressed...
//      digitalWrite(A_valve,HIGH);      // normally open valve = closed, (PNP transistor)
//      digitalWrite(A_pump, HIGH);     // switch pump on
//      Serial.println("inflate");
//      }
//
//  // when button released...
//  Serial.println("deflate");
//  //digitalWrite(A_valve, HIGH);  // normally open valve = open, (PNP transistor)
//  digitalWrite(A_pump, LOW);    // switch pump off
//  while(digitalRead(A_on) == LOW){   // when button pressed...
//      valve_pwm();
//      }
//}

//void valve_pwm(){
//  digitalWrite(A_valve, LOW); // open release valve 
//  delayMicroseconds(t1);       // waits for t1 uS 
//  digitalWrite(A_valve, HIGH);  // close release valve
//  delayMicroseconds(t2);       // waits for t2 uS 
//}





void button_control(){  
  // control of 3 pneumatic chambers using push buttons 
  for (int i = 0; i < 3; i++) {
    button_vals[i] = digitalRead(button_pins[i]);
    //digitalWrite(pump_pins[i], bool(button_vals[i]));
    analogWrite(pump_pins[i], button_vals[i]*pwm_v);
    digitalWrite(valve_pins[i], bool(button_vals[i]));
    }    
}








//void button_control_pwm(){  
//  // control of 3 pneumatic chambers using push buttons 
//  
//
////  for (int i = 0; i < 3; i++) {
////      button_vals_old[i] = button_vals[i];
////  }
////
////
////  Serial.println("oh!");
//    
//  
//  
//    // while button values are unchanged 
//    while(button_vals[0]==button_vals_old[0] and button_vals[1]==button_vals_old[1] and button_vals[2]==button_vals_old[2]){
//      Serial.print(button_vals_old[0]);
//      Serial.print(button_vals_old[1]);
//      Serial.println(button_vals_old[2]);
//      Serial.print(button_vals[0]);
//      Serial.print(button_vals[1]);
//      Serial.println(button_vals[2]);
//      Serial.println();
//  //  
//  //    Serial.println("the same");
//  
//        for (int i = 0; i < 3; i++) {
//          button_vals[i] = digitalRead(button_pins[i]); 
//          //digitalWrite(pump_pins[i], bool(button_vals[i]));      // pump on if button pressed 
//          analogWrite(pump_pins[i], 255);      // pump on if button pressed 
//          //delay(1000);
//          //delayMicroseconds(t1);            // waits for t1 uS 
////          digitalWrite(pump_pins[i], LOW);      // pump off
////          delayMicroseconds(t2);            // waits for t2 uS 
////          delay(1000);
//        }
//  
//        
//    }
//
////    Serial.print(button_vals_old[0]);
////    Serial.print(button_vals_old[1]);
////    Serial.println(button_vals_old[2]);
////    Serial.print(button_vals[0]);
////    Serial.print(button_vals[1]);
////    Serial.println(button_vals[2]);
////    Serial.println();
//  
//    // update button values
//    for (int i = 0; i < 3; i++) {
//        button_vals_old[i] = button_vals[i]; 
//        digitalWrite(valve_pins[i], bool(button_vals[i]));
//        
//        // release air from any chambers where button value is 0
////        if (button_vals[i] == 0 ){
////          digitalWrite(valve_pins[i], LOW);      // close release valve
////        }
////        else{
////          digitalWrite(valve_pins[i], HIGH);      // close release valve
////        }
//    } 
//}
//
//
//  
//
//
//
////  for (int i = 0; i < 3; i++) {
////    button_vals[i] = digitalRead(button_pins[i]); 
////  }
////  Serial.print(button_vals_old[0]);
////  Serial.print(button_vals_old[1]);
////  Serial.println(button_vals_old[2]);
////  Serial.print(button_vals[0]);
////  Serial.print(button_vals[1]);
////  Serial.println(button_vals[2]);
////  Serial.println();
////  //size_t n = sizeof(button_vals);
////  
////    
////  if(button_vals[0]!=button_vals_old[0] or button_vals[1]!=button_vals_old[1] or button_vals[2]!=button_vals_old[2]){
////    for (int i = 0; i < 3; i++) {
////      button_vals_old[i] = button_vals[i];
////    }
////    
////    Serial.println("they don't match");
////    //Serial.println("");
////    
////  }
////  delay(500);
//
//  
//  
//  
////  for (int i = 0; i < 3; i++) {
////    button_vals[i] = digitalRead(button_pins[i]); 
////  }
////  
////  for (int i = 0; i < 3; i++) {
////    button_vals[i] = digitalRead(button_pins[i]);
////    digitalWrite(pump_pins[i], bool(button_vals[i]));
////    digitalWrite(valve_pins[i], bool(button_vals[i])); 
////  }
//
////}



//void serial_control(){
//  // simple control of 3 pneumatic chambers over serial
//  if (Serial.available()) {
//        
//    n_bytes = Serial.readBytes(buf, BUFFER_SIZE);
//    //digitalWrite(LED_BUILTIN, bool(buf[1]));   // turn the LED on (HIGH is the voltage level)
//
//    for (int i = 0; i < 3; i++){
//      //digitalWrite(LED_pins[i], bool(buf[i])); 
//      if (buf[i] > 50) { val = HIGH;}
//      else{ val = LOW;}
//      //digitalWrite(LED_pins[i], val);
//      digitalWrite(pump_pins[i], val);
//      digitalWrite(valve_pins[i], val); 
//      }
//    }
//  
//}



void serial_control_SMA(){
  // simple control of 3 pneumatic chambers over serial
  if (Serial.available()) {
        
    n_bytes = Serial.readBytes(buf, BUFFER_SIZE);
    //digitalWrite(LED_BUILTIN, bool(buf[1]));   // turn the LED on (HIGH is the voltage level)

    for (int i = 0; i < 3; i++){
      //digitalWrite(LED_pins[i], bool(buf[i])); 
      if (buf[0] < 50) { 
        digitalWrite(pump_pins[0], HIGH);
        digitalWrite(pump_pins[1], LOW); 
        }
      else{ 
        digitalWrite(pump_pins[0], LOW);
        digitalWrite(pump_pins[1], HIGH); 
        }
    }
  }
  
}



void serial_control_atan2(){
  // simple control of 3 pneumatic chambers over serial
  if (Serial.available()) {
        
    n_bytes = Serial.readBytes(buf, BUFFER_SIZE);
    //digitalWrite(LED_BUILTIN, bool(buf[1]));   // turn the LED on (HIGH is the voltage level)

    angle  = atan2(buf_[0], buf_[1]);

    
    if (-PI < angle <= -2*PI/3)       {pos = pos1;}
    else if (-2*PI/3 < angle <= -PI/3){pos = pos2;}
    else if (-PI/3 < angle <= 0)      {pos = pos3;}
    else if (0 < angle < PI/3)        {pos = pos4;}
    else if (PI/3 < angle < 2*PI/3)   {pos = pos5;}
    else                              {pos = pos6;}
    

    

//    for (int i = 0; i < 3; i++){
//      //digitalWrite(LED_pins[i], bool(buf[i])); 
//      if (buf[i] > 50) { val = HIGH;}
//      else{ val = LOW;}
//      //digitalWrite(LED_pins[i], val);
//      digitalWrite(pump_pins[i], val);
//      digitalWrite(valve_pins[i], val); 
//      }
//    }

    for (int i = 0; i < 3; i++) {
      //button_vals[i] = digitalRead(button_pins[i]);
      //digitalWrite(pump_pins[i], bool(button_vals[i]));
      analogWrite(pump_pins[i], pos[i]*pwm_v);
      digitalWrite(valve_pins[i], pos[i]);
     } 
  
  }
}




void atan2_test(){
  // simple control of 3 pneumatic chambers over serial

    byte a = (30);
    byte b = (70);


    angle  = atan2(a, b);

    Serial.println(angle);

    
    if (-PI < angle <= -2*PI/3)       {pos = pos1;}
    else if (-2*PI/3 < angle <= -PI/3){pos = pos2;}
    else if (-PI/3 < angle <= 0)      {pos = pos3;}
    else if (0 < angle < PI/3)        {pos = pos4;}
    else if (PI/3 < angle < 2*PI/3)   {pos = pos5;}
    else                              {pos = pos6;}
    
    Serial.println(pos[0]);
    Serial.println(pos[1]);
    Serial.println(pos[2]);
    Serial.println();
    

//    for (int i = 0; i < 3; i++){
//      //digitalWrite(LED_pins[i], bool(buf[i])); 
//      if (buf[i] > 50) { val = HIGH;}
//      else{ val = LOW;}
//      //digitalWrite(LED_pins[i], val);
//      digitalWrite(pump_pins[i], val);
//      digitalWrite(valve_pins[i], val); 
//      }
//    }

    for (int i = 0; i < 3; i++) {
      //button_vals[i] = digitalRead(button_pins[i]);
      //digitalWrite(pump_pins[i], bool(button_vals[i]));
      analogWrite(pump_pins[i], pos[i]*pwm_v);
      digitalWrite(valve_pins[i], pos[i]);
     } 
  
}







void deflate_all(){
  for (int i = 0; i < 3; i++){
      //digitalWrite(LED_pins[i], bool(buf[i])); 
      digitalWrite(pump_pins[i], LOW);
      digitalWrite(valve_pins[i], LOW); 
  }
}
