const int BUFFER_SIZE_tail = 3;
const int n_SMAs_tail = 2;
const int pins_tail[] = {9, 11};
const uint8_t pins_buttons[] = {A1,A2,A3};

class SMA_tentacle {
  
  private:
    uint8_t *_in_pins;
    int *_out_pins;
    char *_buffer;
    int N_SMAs;
    int n_bytes;
    int BUFFER_SIZE;
    
    
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
        //pinMode(_in_pins[i], INPUT);
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
          if (_buffer[0] < 50) { // if number sent over serial < 50 move tentacle left 
          //if (_buffer[0] < _buffer[1]) { // if number sent over serial < 50 move tentacle left
            digitalWrite(_out_pins[0], LOW);
            digitalWrite(_out_pins[1], HIGH); 
            }
          else{                  // otherwise move right 
            digitalWrite(_out_pins[0], HIGH);
            digitalWrite(_out_pins[1], LOW); 
            }
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
    sma_tentacle.serial_control_SMA();
    //sma_tentacle.off();

}
