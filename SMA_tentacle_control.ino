const int BUFFER_SIZE_tail = 2;
const int n_SMAs_tail = 2;
int pins_tail[] = {9, 11};

class SMA_tentacle {
  
  private:
    int *_out_pins;
    char *_buffer;
    int N_SMAs;
    int n_bytes;
    int BUFFER_SIZE;
    
    
  public:
    // constructor
    SMA_tentacle(int out_pins[], int N_SMAs, const int BUFFER_SIZE) {
      this->N_SMAs = N_SMAs; // Use 'this->' to make attribute of the class equal to local variable 'pin' created by constructor 
      this-> BUFFER_SIZE = BUFFER_SIZE;
      
      _out_pins = (int *)malloc(sizeof(int) * N_SMAs); // https://forum.arduino.cc/t/pass-array-size-to-class-constructor/57381
      _buffer = (char *)malloc(sizeof(char) * BUFFER_SIZE);
      
      for(int i=0; i<N_SMAs; i++){
        _out_pins[i] = out_pins[i];
      }
      
      init();
    }
    
    void init() {
      for(int i=0; i<N_SMAs; i++){
        pinMode(_out_pins[i], OUTPUT);
        digitalWrite(_out_pins[i], LOW);
      }      
    }
    
//    void serial_control_SMA() {
//      for(int i=0; i<N_SMAs; i++){
//        digitalWrite(_out_pins[i], HIGH);
//        delay(1000);
//      } 
//    }

    void serial_control_SMA(){
      if (Serial.available()) {          
        n_bytes = Serial.readBytes(_buffer, BUFFER_SIZE);
          if (_buffer[0] < 50) { // if number sent over serial < 50 move tentacle to one side 
            digitalWrite(_out_pins[0], HIGH);
            digitalWrite(_out_pins[1], LOW); 
            }
          else{                  // otherwise move to other side 
            digitalWrite(_out_pins[0], LOW);
            digitalWrite(_out_pins[1], HIGH); 
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


SMA_tentacle sma_tentacle(pins_tail, n_SMAs_tail, BUFFER_SIZE_tail);
void setup() {
  Serial.begin(57600);
}

void loop() {
    sma_tentacle.serial_control_SMA();
    //sma_tentacle.off();

}
