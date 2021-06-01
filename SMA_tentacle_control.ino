 /* 
  *  
  *  Control of a SMA-driven tentacle using command inputs over serial / using push buttons / using breathing sensor (variable resistor)
  *  To set up the code, select whether a 4 or 6 SMA tentacle will be used by setting variables n_SMAs and pins_SMAs
 
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


 Hardware setup: Breathing sensor waistband (FSR) connected to A0

 |---- 3v3
 |
 // FSR (waistband)
 |
 |---- A6
 |
 |---- 10kohm 
 |
 |---- GND


 Hardware setup: LCD (optional)
 
 Arduino  ---- LCD 
 5V       ---- Vcc
 GND      ---- GDN
 SCL (A5) ---- SCL
 SDA (A4) ---- SDA 


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

LiquidCrystal_I2C lcd(0x27,20,4);              // set the LCD address to 0x27 for a 16 chars and 2 line display
 
const int BUFFER_SIZE = 3;                     // number of inputs to recive over serial (x pos human, y pos human, x pos robot)
const int n_buttons = 4;                       // number of push buttons 
const uint8_t pins_buttons[] = {A0,A1,A2,A3};  // button pins 
const uint8_t pin_breathing = A6;              // pin for breathing sensor
const int pin_LED = 2;                         // pin for LED

// 4 SMA actuator 
const int n_SMAs = 4;
const int pins_SMAs[] = {3, 5, 9, 11}; 

// 6 SMA actuator 
//const int n_SMAs = 6;
//const int pins_SMAs[] = {3, 5, 6, 9, 10, 11};

class SMA_tentacle {
  
  private:
    
    uint8_t *_in_pins;    // push buttons
    int *_out_pins;       // SMAs
    char *_buffer;        // info received over serial
    int N_SMAs;           // number of SMAs
    int BUFFER_SIZE;      // size of buffer received over serial
    int N_buttons;        // number of push buttons 
    int pin_LED;          // LED
    int n_bytes;          // bytes received over serial
    

    // Breathing sensor
    uint8_t pin_breathing;         // pin for breathing sensor 
    
    int _breathing_max;            // Max and min readings when doing a large inhale/exhale
    int _breathing_min;
    float _breathing_midpoint;
    
    int _previous_breathing_value; // For control of SMAs based on inhale/exhale direction
    int _current_breathing_value;


    // PID coefficients and variables 
    int Kp = 5;    // 4 SMAs            10; // 8; // 10; //20; // 10;// 8; //10; // 20; //30; // 50; // 100; // 70; // 50; //26; 
    //int Kp = 10; // 6 SMAs
    int Ki = 2;
    int Kd = 3; //5;
    int P;
    int D;    
    int I; 
    int error;
    int prev_error;
    
    
  public:
    // constructor
    SMA_tentacle(uint8_t in_pins[], int out_pins[], const int N_SMAs, const int BUFFER_SIZE, const int N_buttons, uint8_t pin_breathing, int pin_LED) {
      this-> N_SMAs = N_SMAs; 
      this-> BUFFER_SIZE = BUFFER_SIZE;  
      this-> N_buttons = N_buttons;
      this-> pin_breathing = pin_breathing;
      this-> pin_LED = pin_LED; 
      
      _out_pins = (int *)malloc(sizeof(int) * N_SMAs);           // array to store SMA outputs       
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
      /* set pin initial states */
      
      // all SMAs de-activated 
      for(int i=0; i<N_SMAs; i++){
        pinMode(_out_pins[i], OUTPUT);
        digitalWrite(_out_pins[i], LOW);
      } 

      // LED off
      pinMode(pin_LED, OUTPUT);      
      digitalWrite(pin_LED, LOW);

      for(int i=0; i<N_buttons; i++){
        pinMode(_in_pins[i], INPUT);
      }     
    }


    
    

    void button4_control_SMA(){
       /* use 4 push buttons to control tentacle */
       if (n_SMAs == 4){
        this-> button4_control_SMA4();
        }
          
       else if (n_SMAs == 6){
         this-> button4_control_SMA6();
         } 
       
    }
    
    void button4_control_SMA4(){
       /* use 4 push buttons to control tentacle with 4 SMAs at 90 degree spacing */
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
        /* use 4 push buttons to control tentacle with 6 SMAs, 3 on each side (left and right) */
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
      /* 
       *  Move tentacle left or right based on the horizontal position of the human (sent over serial) relative to:
       * A) the centre of the field of view 
       * B) the horizontal position of the robot
      */

      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
          //if (_buffer[0] < 50) {               // if horizontal position of human < centre of field of view, move tentacle left 
          if (_buffer[0] < _buffer[2]) {         // if horizontal position of human < horizontal position of robot, move tentacle left 
            for(int i=0; i<(N_SMAs/2); i++){digitalWrite(_out_pins[i], HIGH);}      // left SMAs on 
            for(int i=(N_SMAs/2); i<N_SMAs; i++){digitalWrite(_out_pins[i], LOW);}  // right SMAs off 
            }
          else{                                  // otherwise move right 
            for(int i=0; i<(N_SMAs/2); i++){digitalWrite(_out_pins[i], LOW);}
            for(int i=(N_SMAs/2); i<N_SMAs; i++){digitalWrite(_out_pins[i], HIGH);} 
            }
        }
    }


    void PID_serial_control_SMA(){
      /*       
       * Move tentacle left or right based on error between horizontal position of human and horizontal position of robot using P / PD / PID controller 
      */
      // control tentacle over serial P / PD / PID controller
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
        error = _buffer[0] - _buffer[2];                     // human horiz position - robot horiz position
        
        //if( abs(error) < 5){error = 0;}                    // lower cap on error  
 
        // PID variables  
        P = error;
        D = error - prev_error;    
        I = I + error;  
        int PIDvalue = abs(Kp*P);                            // P
        //static int PIDvalue = abs((Kp*P + Kd*D));          // PD
        //static int PIDvalue = abs((Kp*P + Kd*D + Ki*I));   // PID
        
        // Display info
        lcd.setCursor(1,0);
        lcd.print("error");
        lcd.print(error);lcd.setCursor(1,1);
        lcd.print("PIDvalue");
        lcd.print(PIDvalue);

        
        if(PIDvalue >= 255){PIDvalue = 255;}                 // upper cap on voltage out to prevent overflow 

        lcd.setCursor(1,2);
        lcd.print("PIDvalue");
        lcd.print(PIDvalue);

        // if human position < robot position move tentacle left
        if (_buffer[0] < _buffer[2]) {                     
          for(int i=0; i<(N_SMAs/2); i++){analogWrite(_out_pins[i], PIDvalue);}
          for(int i=(N_SMAs/2); i<N_SMAs; i++){analogWrite(_out_pins[i], 0);}  
          }
        
        // otherwise move right 
        else{                  
          for(int i=0; i<(N_SMAs/2); i++){analogWrite(_out_pins[i], 0);}
          for(int i=(N_SMAs/2); i<N_SMAs; i++){analogWrite(_out_pins[i], PIDvalue);}  
          }
                       
        prev_error = error; 
        }
    }



    void open_loop_bang_bang_2D(int positions){
      /* 
       *  Move tentacle in 2D plane based on x-y coordinates of human 
       *  Open loop controller that moves tentacle towards 1 of 4 / 8 positions (value given in function call) 
      */
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
        
        int x = _buffer[0];                     // horizontal position human 
        int y = _buffer[1];                     // vertical position human
        //int C = _buffer[2];                   // (horizontal position of robot [unused])

        float A = atan2(double(y), double(x));  // angle between x-y posotion of human and orthogonal axes with origin at centre of field of view 

        // Display information 
        lcd.setCursor(0,0);
        lcd.print(x);
        lcd.setCursor(6,0);
        lcd.print(y);
        lcd.setCursor(0,1);
        lcd.print(A);
        lcd.setCursor(6,1);


        // Move tentacle to 1 of 4 positions by actuating 1 SMA at a time 
        if (positions==4){
          
          int SMA_on;                                  // initialise variable 

          if ((sq(x)/sq(12) + sq(y)/sq(12))  <= 1){    // x-y position inside centre circle
              SMA_on = 4;
          }
          
          else if((0 <= A) and (A < PI/2)){
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

        // Move tentacle to 1 of 8 positions by actuating 1/2 SMAs at a time 
        else if(positions==8){
          int SMA_on[4]={0, 0, 0, 0};
        
          if ((sq(x)/sq(12) + sq(y)/sq(12))  <= 1){ // position inside circle
              lcd.print("O");
          }
          else if((-PI/4 <= A) and (A < PI/8)){
              lcd.print("A");
              SMA_on[0]=1; 
              SMA_on[3]=1; 
          }
          else if((PI/8 <= A) and (A < 3*PI/8)){
              lcd.print("B"); 
              SMA_on[0]=1; 
              
          }
          else if((3*PI/8 <= A) and (A < 5*PI/8)){
              lcd.print("C");
              SMA_on[0]=1; 
              SMA_on[1]=1; 
          }
          else if((5*PI/8 <= A) and (A < 7*PI/8)){
              lcd.print("D");
              SMA_on[1]=1;
          }
          else if(((7*PI/8 <= A) and (A <= PI)) or (-PI <= A) and (A < -7*PI/8)){
              lcd.print("E"); 
              SMA_on[1]=1; 
              SMA_on[2]=1;
          }
          else if((-7*PI/8 <= A) and (A < -5*PI/8)){
              lcd.print("F");
              SMA_on[2]=1;
          }
          else if((-5*PI/8 <= A) and (A < -3*PI/8)){
              lcd.print("G"); 
              SMA_on[2]=1; 
              SMA_on[3]=1;
          }
          else if((-3*PI/8 <= A) and (A < -PI/8)){
              lcd.print("H"); 
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


    void breathing_init(){
      /* 
       *  Initialising breathing sensor to get min and max 
       *  Run this function in main program setup() function ONLY if using function breathing_control_min_max_SMA2()
      */
      // Find min and max of breathing while LED is on
      digitalWrite(pin_LED, HIGH);
      
      int test_breathing_min = analogRead(pin_breathing);
      int test_breathing_max = analogRead(pin_breathing);
      int test_breathing_value = analogRead(pin_breathing);
      
      // Take min and max readings in 5*i ms window
      for (int i = 0; i <= 2000; i++) {
        test_breathing_value = analogRead(pin_breathing);
        test_breathing_min = min(test_breathing_value, test_breathing_min);
        test_breathing_max = max(test_breathing_value, test_breathing_max);
        delay(5);
      }

      digitalWrite(pin_LED, LOW);
      // Set min and max 
      _breathing_min = test_breathing_min;
      _breathing_max = test_breathing_max;
      _breathing_midpoint = test_breathing_min + (test_breathing_max - test_breathing_min)/2;

//      Serial.print("Breathing minimum reading: "); 
//      Serial.println(_breathing_min); 
//      Serial.print("Breathing maximum reading: "); 
//      Serial.println(_breathing_max); 
//      Serial.print("Breathing midpoint reading: "); 
//      Serial.println(_breathing_midpoint); 
    }


    
    void breathing_control_min_max_SMA2(){
      /* 
       *  Simple breathing control: Get midpoint of inhale + exhale and turn SMA1 on if less than that, SMA2 on if more      
       *  LED will come on for 5 seconds at the beginning: inhale and exhale deeply in this time!
      */ 
      
       // use breathing sensor waistband to control tentacle
       int breathing_value = analogRead(pin_breathing);
       Serial.println(breathing_value);
       if(breathing_value > _breathing_midpoint){
          digitalWrite(_out_pins[0], LOW);
          digitalWrite(_out_pins[1], HIGH);
          Serial.println("Inhale");
       } else {
          digitalWrite(_out_pins[1], LOW);
          digitalWrite(_out_pins[0], HIGH);
          Serial.println("Exhale");
       } 
    }

    
    void breathing_control_SMA2(){
      /*
       * Breathing control by whether FSR reading is increasing (SMA1 on) or decreasing (SMA2 on)     
       * Does not need breathing initialisation
       */

      // This value defines the response: too small and it is noisy, too large and slow response
      int response_resolution = 40;
      
      int _current_breathing_value = analogRead(pin_breathing);
      Serial.print("Previous: ");
      Serial.println(_previous_breathing_value);
      Serial.print("Current: ");
      Serial.println(_current_breathing_value);

      // If the FSR reading has noticably changed since previous readings 
      // then update SMAs accordingly and reset previous_breathing_value
       if(_current_breathing_value - _previous_breathing_value > response_resolution){
          digitalWrite(_out_pins[0], LOW);
          digitalWrite(_out_pins[1], HIGH);
          Serial.println("Inhale");
          _previous_breathing_value = _current_breathing_value;
       } 
       else if(_current_breathing_value - _previous_breathing_value < -response_resolution) {
          digitalWrite(_out_pins[1], LOW);
          digitalWrite(_out_pins[0], HIGH);
          Serial.println("Exhale");
          _previous_breathing_value = _current_breathing_value;
       } 
       else {
       }
    }

    
}; 


SMA_tentacle sma_tentacle(pins_buttons, pins_SMAs, n_SMAs, BUFFER_SIZE, n_buttons, pin_breathing, pin_LED);

void setup() {
  Serial.begin(57600);
  delay(1000);              // pause while setting up 
  lcd.init();               // initialize the lcd 
  lcd.backlight();
  
  //sma_tentacle.breathing_init(); // Initialise the breathing sensor - record min and max values
                                   // Only needed if using function breathing_control_min_max_SMA2()
}

void loop() {
    //sma_tentacle.button_control_SMA();
    //sma_tentacle.serial_control_SMA();
    //sma_tentacle.PID_serial_control_SMA();
    sma_tentacle.open_loop_bang_bang_2D(8);

    // Run the basic breathing sensor control of SMAs (needs initialisation to get min and max)
    //sma_tentacle.breathing_control_min_max_SMA2();
  
    // Run the breathing sensor control of SMAs (does not need initialisation)
    //sma_tentacle.breathing_control_SMA2();
}
