 /* 
  *  
  *  Control of a SMA-driven tentacle using command inputs over serial / using push buttons / using breathing sensor (variable resistor)
  *  To set up the code, select whether a 4 or 6 SMA tentacle will be used by setting variables n_SMAs and pins_SMAs
  *  Serial input expected: horizontal and vertical coordinates in orthogonal axes with centre at (0,0) 
 
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


 |---- Vcc (5V on arduino or external 5V power supply) 
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



//****************************************************************************************************************************************
/*
 *   Variables to set
 */
const int operating_mode = 0;                   // Choice of : 0 = push button, 1 = breathing sensor with init, 2 = breathing sensor, 3 = serial communication
const int n_DOF = 1;                            // degrees of freedom of tentacle motion i.e. number of antagonist SMA pairs (choice of  1 or 2 )
const int n_SMAs = 4;                           // number of SMA actuators in tentacle (choice of 4 or 6)

//****************************************************************************************************************************************




LiquidCrystal_I2C lcd(0x27,20,4);              // set the LCD address to 0x27 for a 16 chars and 2 line display

const int BUFFER_SIZE = 3;                     // number of inputs to recive over serial (x pos human, y pos human, x pos robot)
const int n_buttons = 4;                       // number of push buttons 
const uint8_t pins_buttons[] = {A0,A1,A2,A3};  // button pins 
const uint8_t pin_breathing = A6;              // pin for breathing sensor
const uint8_t pin_vib_motor = 10;              // pin for vibrating motor
const int pin_LED = 2;                         // pin for LED
const int pins_4SMAs[] = {3, 5, 9, 11};                   // 4 SMAs
const int pins_6SMAs[] = {3, 5, 6, 9, 10, 11};                   // 6 SMAs



////const int n_SMAs = 4;
////const int pins_SMAs[] = {3, 5, 9, 11}; 
//
//// 6 SMA actuator 
//const int n_SMAs = 6;
//const int pins_SMAs[] = {3, 5, 6, 9, 10, 11};




class SMA_tentacle {
  
  private:
    
    uint8_t *_in_pins;    // push buttons

    int _N_SMAs;           // number of SMAs
    int _N_DOF;           // number of degrees of freedom
    int *_out_pins;       // SMAs
    //int *_out_pins=0;       // SMAs
    //int _out_pins = (int *)malloc(sizeof(int) * _N_SMAs);           // array to store SMA outputs
    char *_buffer;        // info received over serial
    
    int _BUFFER_SIZE;      // size of buffer received over serial
    int _N_buttons;        // number of push buttons 
    int _pin_LED;          // LED
    int _pin_vib_motor = 10;    // vibrating motor 
    int _n_bytes;          // bytes received over serial

    // used for timeout
    unsigned long _time_now;
    unsigned long _time_then;
    float _x_now;
    float _x_then;
    
    // Breathing sensor
    uint8_t _pin_breathing;         // pin for breathing sensor 
    
    int _breathing_max;            // Max and min readings when doing a large inhale/exhale
    int _breathing_min;
    float _breathing_midpoint;
    
    int _previous_breathing_value; // For control of SMAs based on inhale/exhale direction
    int _current_breathing_value;


    // PID coefficients and variables 
    int _Kp;
    int _Ki = 2;
    int _Kd = 3; //5;
    int _P;
    int _D;    
    int _I; 
    int _error;
    int _prev_error;
    
    
  public:
    // constructor
    SMA_tentacle(uint8_t in_pins[], const int N_SMAs, const int N_DOF, const int BUFFER_SIZE, const int N_buttons, uint8_t pin_breathing, int pin_LED, int pin_vib_motor) {
      this-> _N_SMAs = N_SMAs;
      this-> _N_DOF = N_DOF; 
      this-> _BUFFER_SIZE = BUFFER_SIZE;  
      this-> _N_buttons = N_buttons;
      this-> _pin_breathing = pin_breathing;
      this-> _pin_LED = pin_LED;
       

      if(_N_SMAs == 4){
        this-> _Kp = 5;    // 4 SMAs
        }
      else{
        this-> _Kp = 10;  // 6 SMAs
        } 
      
             
      _buffer = (char *)malloc(sizeof(char) * BUFFER_SIZE);      // array to store serial inputs
     
      
      
      _out_pins = (int *)malloc(sizeof(int) * _N_SMAs);           // array to store SMA outputs

      for(int i=0; i<_N_SMAs; i++){                               // populate with SMA pins
        if(_N_SMAs == 4){                     // 4 SMAs
          _out_pins[i] = pins_4SMAs[i];
        }
        else{                                 // 6 SMAs
          _out_pins[i] = pins_6SMAs[i];
        }
      }

      _in_pins = (uint8_t *)malloc(sizeof(uint8_t) * _N_buttons); // array to store physical inputs from buttons
      for(int i=0; i<_N_buttons; i++){                            // populate with button pins
        _in_pins[i] = in_pins[i];
      }
      
      init();
    }
    
    void init() {
      /* set pin initial states */
      
      // all SMAs de-activated 
      for(int i=0; i<_N_SMAs; i++){
        pinMode(_out_pins[i], OUTPUT);
        digitalWrite(_out_pins[i], LOW);
      } 

      // LED off
      pinMode(_pin_LED, OUTPUT);      
      digitalWrite(_pin_LED, LOW);

      // vib_motor off
      pinMode(_pin_vib_motor, OUTPUT);      
      digitalWrite(_pin_vib_motor, LOW);

      for(int i=0; i<_N_buttons; i++){
        pinMode(_in_pins[i], INPUT);
      }     
    }


    
    

    void button_control(){
       /* use 2 or 4 push buttons to control tentacle */
       if (_N_DOF == 1){
         Serial.println("side to side");
         this-> button_control_1DOF();
        }
          
       else if (_N_DOF ==2 ){
         Serial.println("joystick");
         this-> button_control_2DOF();
         } 
       
    }


    void serial_control(){
       /* use serial communication to control tentacle */
       if (_N_DOF == 1){
         Serial.println("side to side");
         this-> PID_serial_control_1DOF();
         //this->timeout();                // return to centre if inactive
         //this->vibrate();                // with vibrating motor 
        }
          
       else if (_N_DOF ==2 ){
         Serial.println("joystick");
         this-> open_loop_serial_control_2DOF(8);
         } 
       
    }
    
    void button_control_2DOF(){
       /* 
       Use 4 push buttons to control motion of 2DOF (2 antagonist SMA pair) tentacle.
       Each button corresponds to an SMA actuator.
       When the button is pushed, the corresponding SMA actuates. 
       */
       
       for(int i=0; i<_N_buttons; i++){
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


  void button_control_1DOF(){
        /* 
       Use 2 push buttons to control side to side motion of 1DOF (1 antagonist SMA pair) tentacle.
       Pushing one button actuaets SMAs on left side, pushing the other actuates the right 
       */
        // LEFT
        if(digitalRead(_in_pins[0]) == HIGH){ 
            for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], HIGH);}    // left SMAs on 
            Serial.println("LEFT");
        }
        else{
          for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], LOW);}       // left SMAs off 
        }

        // RIGHT
        if(digitalRead(_in_pins[1]) == HIGH){ 
          for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], HIGH);} // right SMAs on
          Serial.println("RIGHT");
        }
        else{
          for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], LOW);}  // right SMAs off 
        }
    }


    void serial_control_1DOF(){
       /* 
       Control side to side motion of 1DOF (1 antagonist SMA pair) tentacle using difference between horizontal position 
       of robot and horizontal position to track (e.g. position of human), sent over serial.
       
       Serial data expected: 
       serial buffer 0 = horizontal position to track (e.g. horizontal position of human) 
       serial buffer 1 = vertical position to track (e.g. vertical position of human) 
       serial buffer 2 = horizontal position of robot
       */

      if (Serial.available()) {          
        _n_bytes = Serial.readBytes(_buffer, _BUFFER_SIZE);
          if (_buffer[0] < _buffer[2]) {                                              // if horizontal position to track < horizontal position of robot, move tentacle left 
            for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], HIGH);}       // left SMAs on 
            for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], LOW);}  // right SMAs off 
            }
          else{                                                                       // otherwise move right 
            for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], LOW);}        // left SMAs off
            for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], HIGH);} // right SMAs on
            }
        }
    }


    void PID_serial_control_1DOF(){
       /* 
       Control side to side motion of 1DOF (1 antagonist SMA pair) tentacle using PID controller. 
       Error for P / PD / PID control equal to difference between horizontal position of robot and horizontal 
       position to track (e.g. position of human), sent over serial. 
        
       Serial data expected: 
       serial buffer 0 = horizontal position to track (e.g. horizontal position of human) 
       serial buffer 1 = vertical position to track (e.g. vertical position of human) 
       serial buffer 2 = horizontal position of robot
       */

      if (Serial.available()) {          
        _n_bytes = Serial.readBytes(_buffer, _BUFFER_SIZE);
        _error = _buffer[0] - _buffer[2];                     // human horiz position - robot horiz position
        
        //if( abs(error) < 5){error = 0;}                    // lower cap on error  
 
        // PID variables  
        _P = _error;
        _D = _error - _prev_error;    
        _I = _I + _error;  
        int PIDvalue = abs(_Kp*_P);                          // P
        //static int PIDvalue = abs((Kp*P + Kd*D));          // PD
        //static int PIDvalue = abs((Kp*P + Kd*D + Ki*I));   // PID

        lcd.setCursor(1,0);
        lcd.print(_buffer[0]);
        lcd.setCursor(6,0);
        lcd.print(_buffer[1]);
        lcd.setCursor(1,1);
        lcd.print(_buffer[2]);

        if(PIDvalue >= 255){PIDvalue = 255;}                 // upper cap on voltage out to prevent overflow 

        // if human position < robot position move tentacle left
        if (_buffer[0] < _buffer[2]) {                     
          for(int i=0; i<(_N_SMAs/2); i++){analogWrite(_out_pins[i], PIDvalue);}
          for(int i=(_N_SMAs/2); i<_N_SMAs; i++){analogWrite(_out_pins[i], 0);}  
          }
        
        // otherwise move right 
        else{                  
          for(int i=0; i<(_N_SMAs/2); i++){analogWrite(_out_pins[i], 0);}
          for(int i=(_N_SMAs/2); i<_N_SMAs; i++){analogWrite(_out_pins[i], PIDvalue);}  
          }
                       
        _prev_error = _error; 
        }
    }

    void vibrate(){
      /*
       Scales amplitude of vibration motor voltage to amplitude of horizontal position of robot, sent over serial.
       
       Serial data expected: 
       serial buffer 0 = horizontal position to track (e.g. horizontal position of human) 
       serial buffer 1 = vertical position to track (e.g. vertical position of human) 
       serial buffer 2 = horizontal position of robot
       
       Assumes that horizontal position values sent over serial are in range (-50, 50)  
      */
     
       float vib = map(abs(int(_buffer[2])), 0, 50, 0, 255);
       
       if(vib>20){
        analogWrite(_pin_vib_motor, vib);
       }
       
       else{
        digitalWrite(_pin_vib_motor, LOW);
        } 
    }

    void timeout( unsigned long time_out){
      /*
       Turns off SMAs to try and return actuator to centre if position to track (sent over serial)
       hasn't changed within a threshold time period. 
       TODO : don't just turn off SMAs, make robot return to centre if inactive for too long
       
       Serial data expected: 
       serial buffer 0 = horizontal position to track (e.g. horizontal position of human) 
       serial buffer 1 = vertical position to track (e.g. vertical position of human) 
       serial buffer 2 = horizontal position of robot
       */
      
      time_out = 10000;
      int delta_x = 3;

      // if first run, set prev. x pos to current x pos
      if(_time_then == _time_now){                           
        _x_then = _buffer[0];
       }

       _time_now = millis();                           // time now
       _x_now = _buffer[0];                            // position to track now

       if (abs(_x_now - _x_then) < delta_x){            // check if position to track has changed... 
        
            if ((_time_now - _time_then) > time_out){   // position to track not changed for > timeout --> turn off all SMAs, reset timer
              for(int i=0; i<(_N_SMAs); i++){digitalWrite(_out_pins[i], LOW);}
            }
       }
       
       else{                                          // update timer if position to track changed
        _time_then = _time_now;                   
        _x_then = _x_now;                     
       }
       
    }


    void open_loop_serial_control_2DOF(int positions){
      /* 
       Control motion of 2DOF (2 antagonist SMA pair) tentacle using open loop control. 
       x-y coordinates of position to track, sent over serial, are discretised into 
       4 / 8 positions (value given in function call).
       Open loop controller actuates 1 or 2 SMAs based on the discrete position. All 
      other SMAs are relaxed.  
      */
      if (Serial.available()) {    
        
        _n_bytes = Serial.readBytes(_buffer, _BUFFER_SIZE);
        
        int x = _buffer[0];                     // horizontal position human 
        int y = _buffer[1];                     // vertical position human
        int C = _buffer[2];                     // (horizontal position of robot [unused])

        float A = atan2(double(y), double(x));  // angle between x-y position to track and orthogonal axes, with origin at centre of field of view 

        // Display information 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(x);
        lcd.setCursor(6,0);
        lcd.print(y);
        lcd.setCursor(0,1);
        lcd.print(C);

        
        // Move tentacle to 1 of 4/8 discrete positions by actuating 1/2 SMA at a time 
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
  
          for(int i=0; i<_N_SMAs; i++){
            if(i== SMA_on){
              analogWrite(_out_pins[i], 100);
            }
            else{
              analogWrite(_out_pins[i], 0);
            }
          }
          delay(50);
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
              //SMA_on[3]=1; 
          }
          else if((PI/8 <= A) and (A < 3*PI/8)){
              lcd.print("B"); 
              SMA_on[0]=1;
              SMA_on[1]=1;  
              
          }
          else if((3*PI/8 <= A) and (A < 5*PI/8)){
              lcd.print("C");
              //SMA_on[0]=1; 
              SMA_on[1]=1; 
          }
          else if((5*PI/8 <= A) and (A < 7*PI/8)){
              lcd.print("D");
              SMA_on[1]=1;
              SMA_on[2]=1;
          }
          else if(((7*PI/8 <= A) and (A <= PI)) or (-PI <= A) and (A < -7*PI/8)){
              lcd.print("E"); 
              //SMA_on[1]=1; 
              SMA_on[2]=1;
          }
          else if((-7*PI/8 <= A) and (A < -5*PI/8)){
              lcd.print("F");
              SMA_on[2]=1;
              SMA_on[3]=1;
          }
          else if((-5*PI/8 <= A) and (A < -3*PI/8)){
              lcd.print("G"); 
              //SMA_on[2]=1; 
              SMA_on[3]=1;
          }
          else if((-3*PI/8 <= A) and (A < -PI/8)){
              lcd.print("H"); 
              SMA_on[3]=1;
              SMA_on[0]=1;
          }  
          
          for(int i=0; i<_N_SMAs; i++){
            if(SMA_on[i]== 1){
              analogWrite(_out_pins[i], 240);
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


    
    void breathing_control_min_max(){
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

    
    void breathing_control(){
      /*
       * Breathing control by whether FSR reading is increasing (SMA1 on) or decreasing (SMA2 on)     
       * Does not need breathing initialisation
       */

      // This value defines the response: too small and it is noisy, too large and slow response
      int response_resolution = 40;
      
      int _current_breathing_value = analogRead(pin_breathing);
      // Display information 
      lcd.setCursor(0,0);
      lcd.print("Previous: ");
      lcd.print(_previous_breathing_value);
      lcd.setCursor(0,1);
      lcd.print("Current: ");
      lcd.print(_current_breathing_value);

      // If the FSR reading has noticably changed since previous readings 
      // then update SMAs accordingly and reset previous_breathing_value
       if(_current_breathing_value - _previous_breathing_value > response_resolution){
        for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], LOW);}         // left SMAs on 
        for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], HIGH);}  // right SMAs off 
          Serial.println("Inhale");
          _previous_breathing_value = _current_breathing_value;
       } 
       else if(_current_breathing_value - _previous_breathing_value < -response_resolution) {
          for(int i=0; i<(_N_SMAs/2); i++){digitalWrite(_out_pins[i], HIGH);}      // left SMAs on 
          for(int i=(_N_SMAs/2); i<_N_SMAs; i++){digitalWrite(_out_pins[i], LOW);} // right SMAs off 
          Serial.println("Exhale");
          _previous_breathing_value = _current_breathing_value;
       } 
       else {
       }
    }
   
}; 




SMA_tentacle sma_tentacle(pins_buttons, n_SMAs, n_DOF, BUFFER_SIZE, n_buttons, pin_breathing, pin_LED, pin_vib_motor);

void setup() {
  Serial.begin(57600);
  delay(1000);              // pause while setting up 
  lcd.init();               // initialize the lcd 
  lcd.backlight();
  
  //sma_tentacle.breathing_init(); // Initialise the breathing sensor - record min and max values
                                   // Only needed if using function breathing_control_min_max_SMA2()
}


void loop() {
    
    
    if(operating_mode == 0){
      sma_tentacle.button_control();
    }

    else if(operating_mode == 1){
      sma_tentacle.breathing_control_min_max();
    }

    else if(operating_mode == 2){
      sma_tentacle.breathing_control();
    }

    else if(operating_mode == 3){
      sma_tentacle.serial_control();
    }

    else{
      Serial.println("No operating mode set");
    }

    
}
