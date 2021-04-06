/*
NOTES:
messages are received in process_serial
actions for each message are in the mainloop
*/

#define PIN2  (1<<2)
#define PIN3  (1<<3)
#define SCK_HIGH() PORTD |= PIN2  // OR with port d
#define SCK_LOW() PORTD &= ~PIN2  // AND NOT BYTE with port d
#define DT_READ() ((PIND & PIN3) == PIN3) // 0 or 1
#define BAUD_RATE 115200

////////// Global variables ////////////
//  Table 3 Input Channel and Gain Selection
//  gain  factor  range
//  1     128     20mV
//  3     64      40mV
//  2     32      80mV
const uint8_t hx711_gain = 1;
uint32_t force_value;      // Raw sensor value
uint32_t raw_adc_value;    // Shift in buffer

uint32_t lastTime = millis();
uint32_t currTime = millis();
uint32_t sampleInterval = 0; // ms

boolean sampling = false;

int32_t mot_steps = 0;      // count commanded steps
uint8_t mot_dir = 0;        // sets direction pin
uint16_t mot_speed = 0;     // step delay time
uint16_t displacement = 0;  // extensometer read

////////// Protocol ////////////
const uint8_t MSG_HEADER[] = { 0x74, 0x74 };  // ASCII for tt
uint8_t msgType;      // holds message type of returned by process_serial
uint8_t checksum[2];

enum _messageType {
  MSGT_NONE,
  MSGT_MOT_TARE,        // zero the step counter
  MSGT_MOT_HOME,        // move to towards tare location
  MSGT_START_SAMPLING,  // start motor
  MSGT_STOP_SAMPLING,   // stop motor
  MSGT_SET_SPEED,       // motor speed, ie elongation rate
  MSGT_SAMPLE,          // outgoing data packet
};


// Messages below contain more information than just message type
struct msg_set_mot_speed {
  uint16_t mot_speed;
  uint8_t mot_dir;
};

struct msg_sample {         // from arduino
  uint32_t load;            // latest hx711 reading
  uint32_t steps;           // commanding steps since tare
  uint16_t displacement;    // not used, possibly an extensometer reading
};

struct _in_message {
  uint8_t type;
  union {
    msg_set_mot_speed set_mot_speed;
    // add more messages here
  };
};

struct _out_message {
  uint8_t type;
  union {
    msg_sample sample;
    // add more messages here
  };
};

_in_message in_msg;
_out_message out_msg;

void calcChecksum(uint8_t* CK, void* data, int16_t n) {
  // CK[0] = 0; CK[1] = 0;
  memset (CK, 0 ,2);     // Clear two 8-bit unsigned integers (char)
  for ( int16_t i = 0; i < n; i++ ) {
    CK[0] += ((uint8_t*)data)[i];
    CK[1] += CK[0];
  }
}

// Process the serial messages and return the message type
uint8_t process_serial() {
  static uint8_t state = 0;               // Static to perserve value between function calls
  static uint8_t payloadSize;             // set once msgType is known
  static uint8_t _checksum[2];            // Two 8-bit unsigned integers
  static uint8_t _msgType = MSGT_NONE;

  // Check if there is serial data
  while ( Serial.available() ) {
    byte c = Serial.read();
    
    // Check for header
    if ( state < 2 ) {
      if ( MSG_HEADER[state] == c ) {
        state++;     // Header detected
      } else {
        state = 0;   // No header yet
      }
    }
    // Header was detected and we now buffer the payload and calculated the checksum
    // Place bytes in state-2 because struct does not include the header
    else {
      // Buffer the payload
      if ( (state-2) < sizeof(_in_message) ) {
        ((uint8_t *)(&in_msg))[state-2] = c;
      }
      
      state++;

      // Message type byte
      if ( state == 2 ) {
        _msgType = c;
        payloadSize = 0;  // reset size
        // We know what the massage is so we can set the size
        if ( _msgType ==  MSGT_MOT_TARE ) {
          payloadSize = 1;    // Size of message
        }
        else if ( _msgType ==  MSGT_MOT_HOME ) {
          payloadSize = 1;    // Size of message
        }
        else if ( _msgType ==  MSGT_START_SAMPLING ) {
          payloadSize = 1;    // Size of message
        }
        else if ( _msgType ==  MSGT_STOP_SAMPLING ) {
          payloadSize = 1;    // Size of message
        }
        else if ( _msgType == MSGT_SET_SPEED ) {
          payloadSize = sizeof(msg_set_mot_speed) + 1;    // Size of message
        }
        else {      // Unknown byte
          state = 0;    // Reset waiting for new header
          _msgType = MSGT_NONE;
        }
      }

      // Calculate the checksum
      if ( state == (payloadSize+2) ) {
        calcChecksum(_checksum, &in_msg, payloadSize);
      }
      else if ( state == (payloadSize+3) ) {      // CK_A
        if ( c != _checksum[0] ) state = 0;        // Failed checksum
      }
      else if ( state == (payloadSize+4) ) {      // CK_B
        state = 0;
        if ( c == _checksum[1] ) return _msgType;   // Conclude checksum
      }
      else if ( state > (payloadSize+4) ) {       // Error somewhere, restart
        state = 0;
        _msgType = MSGT_NONE;
      }
    }
  }
  return MSGT_NONE;
}

void send_msg(_out_message* data) {
  uint8_t outSize = sizeof(out_msg);
  calcChecksum(checksum, &out_msg, outSize);
  Serial.write(MSG_HEADER[0]);
  Serial.write(MSG_HEADER[1]);
  for (int i = 0; i < outSize; i++) {
    Serial.write(((uint8_t*)&out_msg)[i]);
  }
  Serial.write(checksum[0]);
  Serial.write(checksum[1]);
}

////////// HX711 ////////////
void setup_hx711() {
  DDRD |= PIN2;   // Set pin D2 to output pin
  SCK_LOW();
  DDRD &= ~PIN3;  // Set pin D3 to input pin
  PORTD |= PIN3;  // Set pin D3 pull up resistor high
}

bool read_hx711() {
  // DT goes LOW when data is ready
  if ( DT_READ() ) {
    return false;
  }
  raw_adc_value = 0;      // Clear
  
  // Each PD_SCK pulse shifts out one bit, starting with
  // the MSB bit first, until all 24 bits are shifted out.
  for (uint8_t i = 0; i < 24; i++) {
    SCK_HIGH();
    delayMicroseconds(1);
    raw_adc_value = raw_adc_value << 1;
    raw_adc_value |= DT_READ();
    SCK_LOW();
    delayMicroseconds(1);
  }

  // Set the channel and the gain factor for the next reading using the clock pin.
  for (uint8_t i = 0; i < hx711_gain; i++) {
    SCK_HIGH();
    delayMicroseconds(1);
    SCK_LOW();
    delayMicroseconds(1);
  }

  // FIX THIS. Overflow handling.
  // Max 80 00 00
  // Min 7F FF FF
  if ( raw_adc_value & 0x00800000 ){
    raw_adc_value |= 0xFF000000;
  }
  
  force_value = raw_adc_value;
  
  return true;
}

///////// main ////////////
void setup() {
  Serial.begin(BAUD_RATE);   // For debugging
  setup_hx711();

  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() {

  // check for incoming serial message
  msgType = process_serial();
  if ( msgType == MSGT_MOT_TARE ) {
    // Reset the step counter
    mot_steps = 0;
    digitalWrite(12, HIGH);
    delay(50);
    digitalWrite(12, LOW);
  }
  else if ( msgType == MSGT_MOT_HOME ) {
    // Command move to home    
    digitalWrite(11, HIGH);
    delay(50);
    digitalWrite(11, LOW);
  }
  else if ( msgType == MSGT_START_SAMPLING ) {
    sampling = true;
    // Command moving at mot_speed
    // Increase sampling rate
    digitalWrite(10, HIGH);
  }
  else if ( msgType == MSGT_STOP_SAMPLING ) {
    sampling = false;
    // Command stop the motor
    // Decrease sampling rate
    digitalWrite(10, LOW);
  }
  else if ( msgType == MSGT_SET_SPEED ) {
    mot_speed = in_msg.set_mot_speed.mot_speed;
    mot_dir = in_msg.set_mot_speed.mot_dir;
    digitalWrite(9, HIGH);
    delay(50);
    digitalWrite(9, LOW);
  }

  // Check for new force reading
  if ( read_hx711() ) {
    digitalWrite(8, HIGH);
    delay(50);
    digitalWrite(8, LOW);
  }

  // Read extensometer


  // If sampling, messages are sent as fast as possible
  // or at sampleInterval when not sampling
  if ( sampling || ( (currTime=millis())-lastTime > sampleInterval ) ){
    out_msg.type = MSGT_SAMPLE;
//    out_msg.sample.load = force_value;
    out_msg.sample.load = force_value;
    
    mot_steps = mot_steps + 10;
    displacement = displacement + 1;

    out_msg.sample.steps = mot_steps;
    out_msg.sample.displacement = displacement;
    lastTime = currTime;
  } else {
    return;
  }


  // Send message
  send_msg(&out_msg);



  // DEBUGGING
//  Serial.println("");
//  uint8_t* structPtr = (uint8_t*) &in_msg;
//  for (uint8_t i = 0; i < sizeof(in_msg); i++) {
//    Serial.print(*structPtr++, HEX);
//    Serial.print(" ");
//  }

  

//  Serial.println("");
//  Serial.print(force_value);
//  Serial.print("\t");
//  Serial.print(mot_steps);
//  Serial.print("\t");
//  Serial.print(displacement);
//  Serial.print("\t");
//  Serial.print(sizeof(out_msg));
//

  Serial.println("");

  Serial.print(MSG_HEADER[0], HEX);
  Serial.print(" ");
  Serial.print(MSG_HEADER[1], HEX);
  Serial.print(" ");

  uint8_t* structPtr = (uint8_t*) &out_msg;
  for (uint8_t i = 0; i < sizeof(out_msg); i++) {
    Serial.print(*structPtr++, HEX);
    Serial.print(" ");
  }
  Serial.print(checksum[0], HEX);
  Serial.print(" ");
  Serial.print(checksum[1], HEX);
  
  Serial.println("");

}
