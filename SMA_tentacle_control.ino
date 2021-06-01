 /* 
 
 Hardware setup: Push button inputs (repeat for pins A0 / A1 / A2 / A3)
 
 |---- Vcc
 |
 // Push button switch (normally open) 
 |
 |---- A0 / A1 / A2 / A3
 |
 |---- 10kohm 
 |
 |---- GND



 Hardware setup: Transistors controlling output to SMAs (repeat for pins D3/ D5/ D9/ D11 [ and D6/ D10 ] and and SMAs a / b / c / d [ and e/ f ])
 
 |---- D3 / D5 / D9 / D11 // D6 / D10
 |
 |---- Gate BC547C (NPN General Purpose Transistor) 
 |
 |---- GND

            

 |---- Vcc
 |
 |---- Collector BC547C (NPN General Purpose Transistor)


 |---- Emitter BC547C (NPN General Purpose Transistor)
 |
 |---- SMA a / b / c / d // e / f positive terminal 


 |---- SMA a & b & c & d // e & f negative terminal (terminals for all SMAs connected together) 
 |
 |---- GND 
 


 Hardware setup: Other 

 |---- Pin D2  
 |
 |---- LED
 |
 |---- GND


 
 |---- GND (arduino)  
 |
 |---- GND (supply) 



 |---- 5V (supply) or Arduino 5V 
 |
 |---- Vcc 
 */
 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int T = 1;
 
const int BUFFER_SIZE_tail = 3;
//const uint8_t pins_buttons[] = {A1,A2,A3};

//const int n_SMAs_tail = 2;
//const int pins_tail[] = {9, 11};

//const int n_SMAs_tail = 3;
//const int pins_tail[] = {5, 9, 11};

//const int BUFFER_SIZE_tail = 4;
const int n_buttons = 4;
const uint8_t pins_buttons[] = {A0,A1,A2,A3};


// const int pins_tail[] = {3, 5, 9, 11};
// const int pins_tail[] = {5, 6, 9, 11};
//const int pins_tail[] = {3, 5, 6, 11};
const int n_SMAs_tail = 4;
const int pins_tail[] = {3, 5, 9, 11}; 

//const int n_SMAs_tail = 6;
//const int pins_tail[] = {3, 5, 6, 9, 10, 11};

class SMA_tentacle {
  
  private:
    const int LED_pin = 2;
    uint8_t *_in_pins;
    int *_out_pins;
    char *_buffer;
    int N_SMAs;
    int N_buttons;
    int n_bytes;
    int BUFFER_SIZE;
    int error;
    int prev_error;

    // PID coefficients
    int Kp = 5;    // 4 SMAs   10; // 8; // 10; //20; // 10;// 8; //10; // 20; //30; // 50; // 100; // 70; // 50; //26; 
    //int Kp = 10; // 6 SMAs
    int Ki = 2;
    int Kd = 3; //5;
    
    int P;
    int D;    
    int I; 
    
    
  public:
    // constructor
    SMA_tentacle(int out_pins[], const int N_SMAs, const int BUFFER_SIZE, uint8_t in_pins[], const int N_buttons) {
      this->N_SMAs = N_SMAs; 
      this-> BUFFER_SIZE = BUFFER_SIZE;
      this-> N_buttons = N_buttons;
      
      _out_pins = (int *)malloc(sizeof(int) * N_SMAs);         
      _buffer = (char *)malloc(sizeof(char) * BUFFER_SIZE);      // array to store serial inputs
      _in_pins = (uint8_t *)malloc(sizeof(uint8_t) * N_buttons); // array to store physical inputs same length as buffer (for testing)
      
      for(int i=0; i<N_SMAs; i++){
        _out_pins[i] = out_pins[i];
      }

      for(int i=0; i<N_buttons; i++){
        _in_pins[i] = in_pins[i];
      }
      
      init();
    }
    
    void init() {
      for(int i=0; i<N_SMAs; i++){
        pinMode(_out_pins[i], OUTPUT);
        digitalWrite(_out_pins[i], LOW);
      } 
      pinMode(LED_pin, OUTPUT);
      digitalWrite(LED_pin, LOW);
      for(int i=0; i<N_buttons; i++){
        pinMode(_in_pins[i], INPUT);
      }     
    }
    
    void button4_control_SMA4(){
       // use push buttons to control tentacle
       for(int i=0; i<N_buttons; i++){
        if(digitalRead(_in_pins[i]) == HIGH){ 
          digitalWrite(_out_pins[i], HIGH); 
        }
          
        else{
         digitalWrite(_out_pins[i], LOW);
         Serial.print("low");          
         } 
       
       Serial.println("");
       }
    }


  void button4_control_SMA6(){
        // use push buttons to control tentacle
        // LEFT
        if(digitalRead(_in_pins[0]) == HIGH){ 
            for(int i=0; i<3; i++){
              digitalWrite(_out_pins[i], HIGH);
            }
          }
          else{
            for(int i=0; i<3; i++){
              digitalWrite(_out_pins[i], LOW);
            }
          }

          // RIGHT
          if(digitalRead(_in_pins[1]) == HIGH){ 
            for(int i=3; i<6; i++){
              digitalWrite(_out_pins[i], HIGH);
            }
          }
          else{
            for(int i=3; i<6; i++){
              digitalWrite(_out_pins[i], LOW);
            }
          }
    }


    void serial_control_SMA(){
      // control tentacle over serial by simply choosing to move left or right based on value received 
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
          //if (_buffer[0] < 50) {               // if number sent over serial < 50 move tentacle left 
          if (_buffer[0] < _buffer[2]) {         // if number sent over serial < 50 move tentacle left
            for(int i=0; i<(N_SMAs/2); i++){digitalWrite(_out_pins[i], HIGH);}
            for(int i=(N_SMAs/2); i<N_SMAs; i++){digitalWrite(_out_pins[i], LOW);} 
//            digitalWrite(_out_pins[0], LOW);
//            digitalWrite(_out_pins[1], LOW);
//            digitalWrite(_out_pins[2], HIGH); 
//            digitalWrite(_out_pins[3], HIGH); 
            }
          else{                                  // otherwise move right 
            for(int i=0; i<(N_SMAs/2); i++){digitalWrite(_out_pins[i], LOW);}
            for(int i=(N_SMAs/2); i<N_SMAs; i++){digitalWrite(_out_pins[i], HIGH);} 
//            digitalWrite(_out_pins[0], HIGH);
//            digitalWrite(_out_pins[1], HIGH); 
//            digitalWrite(_out_pins[2], LOW);
//            digitalWrite(_out_pins[3], LOW); 
            }
        }
    }



//    void PID_serial_control_SMA(){
//      // control tentacle over serial P / PD / PID controller
//      if (Serial.available()) {          
//        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
//        error = _buffer[0] - _buffer[1]; // human horiz position - robot horiz position
//
//        
//        //if( abs(error) < 5){error = 0;} // lower cap on error  
// 
//        // PID variables  
//        P = error;
//        D = error - prev_error;    
//        I = I + error;  
//        static int PIDvalue = abs((Kp*P + Kd*D + Ki*I));  
//        //static int PIDvalue = abs((Kp*P + Kd*D));
//        //static int PIDvalue = abs(Kp*P);
//
//        
//        // If voltage = max --> LED alert! 
//        if(PIDvalue >= 255){ 
//          PIDvalue = 255; // cap on voltage out to prevent overflow
//          digitalWrite(LED_pin, HIGH); 
//          } 
//         else{
//          digitalWrite(LED_pin, LOW); 
//          }
//
//            // *************************************************************
////          // P or PD CONTROLLER WITH DEAD ZONE TO PREVENT WOBBLE
////          // If in dead zone, both actuators off...
////          if  (40 < _buffer[0] && _buffer[0] < 60 && 40 < _buffer[1] && _buffer[1] < 60 ) {
////            digitalWrite(_out_pins[0], LOW);
////            digitalWrite(_out_pins[1], LOW); 
////            //digitalWrite(LED_pin, HIGH); 
////          }
////          // ...otherwise move left...
////          else if (_buffer[0] < _buffer[1]) { // if human position < robot position move tentacle left
////            analogWrite(_out_pins[0], 0);
////            analogWrite(_out_pins[1], PIDvalue); 
////            //digitalWrite(LED_pin, LOW); 
////            }
////          
////          // ... or right. 
////          else{                  // otherwise move right 
////            analogWrite(_out_pins[0], PIDvalue);
////            analogWrite(_out_pins[1], 0); 
////            //digitalWrite(LED_pin, LOW); 
////            }
//          // *************************************************************
//
//
//
//          // *************************************************************
//          // P / PD / PID CONTROLLER WITHOUT DEAD ZONE
//          // Move left...
//          if (_buffer[0] < _buffer[1]) { // if human position < robot position move tentacle left
//            analogWrite(_out_pins[0], 0);
//            analogWrite(_out_pins[1], PIDvalue); 
//            //digitalWrite(LED_pin, LOW); 
//            }
//          
//          // ... or right. 
//          else{                  // otherwise move right 
//            analogWrite(_out_pins[0], PIDvalue);
//            analogWrite(_out_pins[1], 0); 
//            //digitalWrite(LED_pin, LOW); 
//            }
//          // *************************************************************
//            
//        prev_error = error; 
//        }
//    }


    void PID_serial_control_SMA(){
      // control tentacle over serial P / PD / PID controller
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
        error = _buffer[0] - _buffer[2]; // human horiz position - robot horiz position
        lcd.setCursor(1,0);
        lcd.print("error");
        //lcd.setCursor(6,0);
        lcd.print(error);

        
        //if( abs(error) < 5){error = 0;} // lower cap on error  
 
        // PID variables  
        P = error;
        D = error - prev_error;    
        I = I + error;  
        //static int PIDvalue = abs((Kp*P + Kd*D + Ki*I));  
        //static int PIDvalue = abs((Kp*P + Kd*D));
        int PIDvalue = abs(Kp*P);
        lcd.setCursor(1,1);
        lcd.print("PIDvalue");
        //lcd.setCursor(6,1);
        lcd.print(PIDvalue);

        
        // If voltage = max --> LED alert! 
        if(PIDvalue >= 255){ 
          PIDvalue = 255; // cap on voltage out to prevent overflow
          //digitalWrite(LED_pin, HIGH); 
          } 
//         else{
//          digitalWrite(LED_pin, LOW); 
//          }

        lcd.setCursor(1,2);
        lcd.print("PIDvalue");
        //lcd.setCursor(6,1);
        lcd.print(PIDvalue);
//
//            // *************************************************************
////          // P or PD CONTROLLER WITH DEAD ZONE TO PREVENT WOBBLE
////          // If in dead zone, both actuators off...
////          if  (40 < _buffer[0] && _buffer[0] < 60 && 40 < _buffer[1] && _buffer[1] < 60 ) {
////            digitalWrite(_out_pins[0], LOW);
////            digitalWrite(_out_pins[1], LOW); 
////            //digitalWrite(LED_pin, HIGH); 
////          }
////          // ...otherwise move left...
////          else if (_buffer[0] < _buffer[1]) { // if human position < robot position move tentacle left
////            analogWrite(_out_pins[0], 0);
////            analogWrite(_out_pins[1], PIDvalue); 
////            //digitalWrite(LED_pin, LOW); 
////            }
////          
////          // ... or right. 
////          else{                  // otherwise move right 
////            analogWrite(_out_pins[0], PIDvalue);
////            analogWrite(_out_pins[1], 0); 
////            //digitalWrite(LED_pin, LOW); 
////            }
//          // *************************************************************
//
//
//
//          // *************************************************************
          // P / PD / PID CONTROLLER WITHOUT DEAD ZONE
          // Move left...
          if (_buffer[0] < _buffer[2]) { // if human position < robot position move tentacle left
            for(int i=0; i<(N_SMAs/2); i++){analogWrite(_out_pins[i], PIDvalue);}
            for(int i=(N_SMAs/2); i<N_SMAs; i++){analogWrite(_out_pins[i], 0);}  
            }
          
          // ... or right. 
          else{                  // otherwise move right 
            for(int i=0; i<(N_SMAs/2); i++){analogWrite(_out_pins[i], 0);}
            for(int i=(N_SMAs/2); i<N_SMAs; i++){analogWrite(_out_pins[i], PIDvalue);}  
            }
//          // *************************************************************
//            
//        prev_error = error; 
        }
    }



    void serial_to_LCD(int positions){
      // control tentacle over serial P / PD / PID controller
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
        //error = _buffer[0] - _buffer[1]; // human horiz position - robot horiz position
        
        int x = _buffer[0];
        int y = _buffer[1];
        //int C = _buffer[2];

        lcd.setCursor(0,0);
        lcd.print(x);
        lcd.setCursor(6,0);
        lcd.print(y);
        float A = atan2(double(y), double(x));
        lcd.setCursor(0,1);
        lcd.print(A);

        lcd.setCursor(6,1);

        if (positions==4){
          int SMA_on;
          
          if((0 <= A) and (A < PI/2)){
              lcd.print("A");
              SMA_on = 0; 
          }
          else if((PI/2 <= A) and (A <= PI)){
              lcd.print("B");
              SMA_on = 1; 
          }
          else if((-PI <= A) and (A < -PI/2)){
              lcd.print("C");
              SMA_on = 2; 
          }
          else{   
              lcd.print("D");
              SMA_on = 3; 
          }
  
          for(int i=0; i<N_SMAs; i++){
            if(i== SMA_on){
              analogWrite(_out_pins[i], 100);
            }
            else{
              analogWrite(_out_pins[i], 0);
            }
          }
        }

        else if(positions==8){
          int SMA_on[4]={0, 0, 0, 0};
        
          
          if ((sq(x)/sq(12) + sq(y)/sq(12))  <= 1){ // position inside circle
              //https://math.stackexchange.com/questions/76457/check-if-a-point-is-within-an-ellipse
              //https://math.stackexchange.com/questions/198764/how-to-know-if-a-point-is-inside-a-circle#:~:text=The%20point%20%E2%9F%A8xp%2Cy,if%20d2%3Er2.
              lcd.print("O");
              //int SMA_on[] = {1, 0, 0, 1};
          }
          else if((-PI/4 <= A) and (A < PI/8)){
              lcd.print("A");
              //int SMA_on[] = {1, 0, 0, 1};
              SMA_on[0]=1; 
              SMA_on[3]=1; 
          }
          else if((PI/8 <= A) and (A < 3*PI/8)){
              lcd.print("B");
              //int SMA_on[] = {1, 0, 0, 0}; 
              SMA_on[0]=1; 
              
          }
          else if((3*PI/8 <= A) and (A < 5*PI/8)){
              lcd.print("C");
              //int SMA_on[] = {1, 1, 0, 0}; 
              SMA_on[0]=1; 
              SMA_on[1]=1; 
          }
          else if((5*PI/8 <= A) and (A < 7*PI/8)){
              lcd.print("D");
              //int SMA_on[] = {0, 1, 0, 0}; 
              SMA_on[1]=1;
          }
          else if(((7*PI/8 <= A) and (A <= PI)) or (-PI <= A) and (A < -7*PI/8)){
              lcd.print("E");
              //int SMA_on[] = {0, 1, 1, 0}; 
              SMA_on[1]=1; 
              SMA_on[2]=1;
          }
          else if((-7*PI/8 <= A) and (A < -5*PI/8)){
              lcd.print("F");
              //int SMA_on[] = {0, 0, 1, 0}; 
              SMA_on[2]=1;
          }
          else if((-5*PI/8 <= A) and (A < -3*PI/8)){
              lcd.print("G");
              //int SMA_on[] = {0, 0, 1, 1}; 
              SMA_on[2]=1; 
              SMA_on[3]=1;
          }
          else if((-3*PI/8 <= A) and (A < -PI/8)){
              lcd.print("H");
              //int SMA_on[] = {0, 0, 0, 1};  
              SMA_on[3]=1;
          }  
          


          for(int i=0; i<N_SMAs; i++){
            if(SMA_on[i]== 1){
              analogWrite(_out_pins[i], 100);
            }
            else{
              analogWrite(_out_pins[i], 0);
            }
          }
        }
      } 
    }

    
}; 


SMA_tentacle sma_tentacle(pins_tail, n_SMAs_tail, BUFFER_SIZE_tail, pins_buttons, n_buttons);

void setup() {
  Serial.begin(57600);

//  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
//  lcd.setCursor(1,0);
//  lcd.print("hello everyone");
//  lcd.setCursor(1,1);
//  lcd.print("konichiwaa");
}

void loop() {
//  T = T + 1;
//  lcd.setCursor(1,1);
//  lcd.print(T);
  //delay(1000);
  
  
//  if(n_SMAs_tail == 4){
//    sma_tentacle.button4_control_SMA4();
//  }
//
// else if(n_SMAs_tail == 6){
//    sma_tentacle.button4_control_SMA6();
//  }
  
    //sma_tentacle.button_control_SMA();
    //sma_tentacle.serial_control_SMA();
    //sma_tentacle.PID_serial_control_SMA();
    sma_tentacle.serial_to_LCD(8);
}
