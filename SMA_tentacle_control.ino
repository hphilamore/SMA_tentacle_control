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
    int Kp = 10;// 8; //10; // 20; //30; // 50; // 100; // 70; // 50; //26; 
    int Ki;
    int Kd = 3; //5;
    
    
  public:
    // constructor
    SMA_tentacle(int out_pins[], const int N_SMAs, const int BUFFER_SIZE, uint8_t in_pins[]) {
      this->N_SMAs = N_SMAs; // Use 'this->' to make attribute of the class equal to local variable 'pin' created by constructor 
      this-> BUFFER_SIZE = BUFFER_SIZE;
      
      _out_pins = (int *)malloc(sizeof(int) * N_SMAs); // https://forum.arduino.cc/t/pass-array-size-to-class-constructor/57381
      _buffer = (char *)malloc(sizeof(char) * BUFFER_SIZE); // buffer to store serial inputs
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
    
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
          //if (_buffer[0] < 50) { // if number sent over serial < 50 move tentacle left 
          if (_buffer[0] < _buffer[1]) { // if number sent over serial < 50 move tentacle left
            digitalWrite(_out_pins[0], LOW);
            digitalWrite(_out_pins[1], HIGH); 
            }
          else{                  // otherwise move right 
            digitalWrite(_out_pins[0], HIGH);
            digitalWrite(_out_pins[1], LOW); 
            }
        }
    }


    void PID_serial_control_SMA(){
    
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
//        _buffer[0] = floor(_buffer[0]/5)*5;
//        _buffer[1] = floor(_buffer[1]/5)*5;
        error = _buffer[0] - _buffer[1]; // human horiz position - robot horiz position
        
        if( abs(error) < 5){ 
          error = 0;
          //digitalWrite(LED_pin, HIGH); 
          } 
//         else{
//          digitalWrite(LED_pin, LOW); 
//          }  
          
        static int P = error;
        static int D = error - prev_error;        
        //static int PIDvalue = abs((Kp*P + Kd*D)); // + (Kd*D);
        static int PIDvalue = abs((Kp*P));// + (Ki*I) + (Kd*D);
        
        if(PIDvalue >= 255){ 
          PIDvalue = 255; // cap on voltage out to prevent overflow
          //digitalWrite(LED_pin, HIGH); 
          } 
//         else{
//          digitalWrite(LED_pin, LOW); 
//          }

          //if (_buffer[0] < 50) { // if number sent over serial < 50 move tentacle left 
          if (_buffer[0] < _buffer[1]) { // if number sent over serial < 50 move tentacle left
          //if (error < 0 ){
            analogWrite(_out_pins[0], 0);
            analogWrite(_out_pins[1], PIDvalue); 
            }
          else{                  // otherwise move right 
            analogWrite(_out_pins[0], PIDvalue);
            analogWrite(_out_pins[1], 0); 
            }
        prev_error = error; 
        }
    }

    void PID_test_setup(){
    
        int A = 44;
        int B = 84;

        static int error = A - B; // human horiz position - robot horiz position 
        Serial.print(" A = ");
        Serial.print(A); 
        Serial.print(" B = ");
        Serial.print(B); 
        Serial.print(" error = ");
        Serial.print(error); 
        static int P = error;
        
        static int PIDvalue = abs((Kp*P));// + (Ki*I) + (Kd*D);
        Serial.print(" PID = ");
        Serial.println(PIDvalue); 

        
          //if (_buffer[0] < 50) { // if number sent over serial < 50 move tentacle left 
          //if (_buffer[0] < _buffer[1]) { // if number sent over serial < 50 move tentacle left
          if (error < 0 ){
            analogWrite(_out_pins[0], 0);
            analogWrite(_out_pins[1], PIDvalue); 
            }
          else{                  // otherwise move right 
            analogWrite(_out_pins[0], PIDvalue);
            analogWrite(_out_pins[1], 0); 
            }

    }
    
    void off() {
      for(int i=0; i<N_SMAs; i++){
        digitalWrite(_out_pins[i], LOW);
        delay(1000);
      } 
    }
}; // semicolon at the end of the class


SMA_tentacle sma_tentacle(pins_tail, n_SMAs_tail, BUFFER_SIZE_tail, pins_buttons);
void setup() {
  Serial.begin(57600);
}

void loop() {
    //sma_tentacle.button_control_SMA();
    //sma_tentacle.serial_control_SMA();
    sma_tentacle.PID_serial_control_SMA();
    //sma_tentacle.PID_test_setup();
    //sma_tentacle.off();

}
