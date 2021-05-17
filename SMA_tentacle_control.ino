const int BUFFER_SIZE_tail = 3;
const int n_SMAs_tail = 2;
const int pins_tail[] = {9, 11};

const uint8_t pins_buttons[] = {A1,A2,A3};

class SMA_tentacle {
  
  private:
    const int LED_pin = 2;
    uint8_t *_in_pins;
    int *_out_pins;
    char *_buffer;
    int N_SMAs;
    int n_bytes;
    int BUFFER_SIZE;
    int error;
    int prev_error;

    // PID coefficients
    int Kp = 10; // 8; // 10; //20; // 10;// 8; //10; // 20; //30; // 50; // 100; // 70; // 50; //26; 
    int Ki = 2;
    int Kd = 3; //5;
    
    int P;
    int D;    
    int I; 
    
    
  public:
    // constructor
    SMA_tentacle(int out_pins[], const int N_SMAs, const int BUFFER_SIZE, uint8_t in_pins[]) {
      this->N_SMAs = N_SMAs; 
      this-> BUFFER_SIZE = BUFFER_SIZE;
      
      _out_pins = (int *)malloc(sizeof(int) * N_SMAs); 
      _buffer = (char *)malloc(sizeof(char) * BUFFER_SIZE);        // array to store serial inputs
      _in_pins = (uint8_t *)malloc(sizeof(uint8_t) * BUFFER_SIZE); // array to store physical inputs same length as buffer (for testing)
      
      for(int i=0; i<N_SMAs; i++){
        _out_pins[i] = out_pins[i];
        _in_pins[i] = in_pins[i];
      }
      
      init();
    }
    
    void init() {
      for(int i=0; i<N_SMAs; i++){
        pinMode(_out_pins[i], OUTPUT);
        digitalWrite(_out_pins[i], LOW);
        pinMode(_in_pins[i], INPUT);
        pinMode(LED_pin, OUTPUT);
        digitalWrite(LED_pin, LOW);
      }      
    }
    
    void button_control_SMA(){
       // use push buttons to control tentacle
       for(int i=0; i<N_SMAs; i++){
        if(digitalRead(_in_pins[i]) == HIGH){ 
          digitalWrite(_out_pins[i], HIGH); 
          Serial.print("high");
          }
        else{
         digitalWrite(_out_pins[i], LOW);
         Serial.print("low"); 
         } 
       }
       Serial.println("");
    }

    void serial_control_SMA(){
      // control tentacle over serial by simply choosing to move left or right based on value received 
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
          //if (_buffer[0] < 50) {               // if number sent over serial < 50 move tentacle left 
          if (_buffer[0] < _buffer[1]) {         // if number sent over serial < 50 move tentacle left
            digitalWrite(_out_pins[0], LOW);
            digitalWrite(_out_pins[1], HIGH); 
            }
          else{                                  // otherwise move right 
            digitalWrite(_out_pins[0], HIGH);
            digitalWrite(_out_pins[1], LOW); 
            }
        }
    }


    void PID_serial_control_SMA(){
      // control tentacle over serial P / PD / PID controller
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
        error = _buffer[0] - _buffer[1]; // human horiz position - robot horiz position

        
        //if( abs(error) < 5){error = 0;} // lower cap on error  
 
        // PID variables  
        P = error;
        D = error - prev_error;    
        I = I + error;  
        static int PIDvalue = abs((Kp*P + Kd*D + Ki*I));  
        //static int PIDvalue = abs((Kp*P + Kd*D));
        //static int PIDvalue = abs(Kp*P);

        
        // If voltage = max --> LED alert! 
        if(PIDvalue >= 255){ 
          PIDvalue = 255; // cap on voltage out to prevent overflow
          digitalWrite(LED_pin, HIGH); 
          } 
         else{
          digitalWrite(LED_pin, LOW); 
          }

            // *************************************************************
//          // P or PD CONTROLLER WITH DEAD ZONE TO PREVENT WOBBLE
//          // If in dead zone, both actuators off...
//          if  (40 < _buffer[0] && _buffer[0] < 60 && 40 < _buffer[1] && _buffer[1] < 60 ) {
//            digitalWrite(_out_pins[0], LOW);
//            digitalWrite(_out_pins[1], LOW); 
//            //digitalWrite(LED_pin, HIGH); 
//          }
//          // ...otherwise move left...
//          else if (_buffer[0] < _buffer[1]) { // if human position < robot position move tentacle left
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
          // *************************************************************



          // *************************************************************
          // P / PD / PID CONTROLLER WITHOUT DEAD ZONE
          // Move left...
          if (_buffer[0] < _buffer[1]) { // if human position < robot position move tentacle left
            analogWrite(_out_pins[0], 0);
            analogWrite(_out_pins[1], PIDvalue); 
            //digitalWrite(LED_pin, LOW); 
            }
          
          // ... or right. 
          else{                  // otherwise move right 
            analogWrite(_out_pins[0], PIDvalue);
            analogWrite(_out_pins[1], 0); 
            //digitalWrite(LED_pin, LOW); 
            }
          // *************************************************************
            
        prev_error = error; 
        }
    }
    
}; 


SMA_tentacle sma_tentacle(pins_tail, n_SMAs_tail, BUFFER_SIZE_tail, pins_buttons);

void setup() {
  Serial.begin(57600);
}

void loop() {
    //sma_tentacle.button_control_SMA();
    //sma_tentacle.serial_control_SMA();
    sma_tentacle.PID_serial_control_SMA();
}
