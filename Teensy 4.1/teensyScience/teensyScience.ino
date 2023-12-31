#include <ACAN_T4.h>
#include <Servo.h>


/*
 * USING TALONS TO CONTROL ALL MOTORS (ACTYUATOR/CAROUSEL) -> TREAT EACH MOTOR AS A CONTINUOUS SERVO
 *  The carousel has an external limit switches to maintain alignment between test tubes and fan cone.
 */

// Set equal to 1 for serial debugging
#define DEBUG 0

#define SCIENCE_ID 0x03

// which pin is up depends on wiring. Can be reversed
#define ACTUATOR_UP_PIN 16
#define ACTUATOR_DOWN_PIN 17
//#define ACTUATOR_LIMIT_PIN

#define CAROUSEL_PIN 13
#define CAROUSEL_LIMIT_PIN 3

#define FAN_PIN 21 // what 9 + 10
#define MICROSCOPE_UP_PIN 7
#define MICROSCOPE_DOWN_PIN 8
#define MICROSCOPE_ENCODER_PIN 10

// servo objects for talon motor controllers
Servo carousel, fan;

// The current CAN message to be sent out
CANMessage message;

// raw bytes to store from ethernet data
// they are converted to output ranges in updateServos()
uint8_t myHash = 0;
uint8_t serialHash = 0;
int linearActuatorSpeed = 127;
int carouselSpeed = 1;
int fanSpeed = 90;
int microscopePosition = 1;

bool carouselMoving = false;
bool linearActuatorMoving = false;

unsigned long stopTimeout = 0, turnTimeout = 0;

// callback that prints received packets to the serial port
void updateMotors(CANMessage message) {

  // SCIENCE msg: [deviceID, linearActuator, carousel, fan, microscope]
  linearActuatorSpeed = message.data[0];
  carouselSpeed = message.data[1];
  fanSpeed = message.data[2];
  microscopePosition = message.data[3];

  // update linear actuator
  if(linearActuatorSpeed > 127) {
    digitalWrite(ACTUATOR_UP_PIN, HIGH);
  }
  else if (linearActuatorSpeed < 127) {
    digitalWrite(ACTUATOR_DOWN_PIN, HIGH);
  }
  else {
    digitalWrite(ACTUATOR_UP_PIN, LOW);
    digitalWrite(ACTUATOR_DOWN_PIN, LOW);
  }

  //update carousel
  if(carouselSpeed==0) { //if carousel already moving, no need to call write function again
    carousel.write(130);
    carouselMoving = true;
    delay(10);
  }
  else if(carouselSpeed==2) {
    carousel.write(50);
    carouselMoving = true;
    delay(10);
  }
  else if (carouselSpeed == 1) {}
  
  //update claw
  fan.write(fanSpeed);

  //update microscope
  if(microscopePosition == 0) {
    digitalWrite(MICROSCOPE_UP_PIN, HIGH);
  }
  else if(microscopePosition == 2) {
    digitalWrite(MICROSCOPE_DOWN_PIN, HIGH);
  }
  else {
    digitalWrite(MICROSCOPE_UP_PIN, LOW);
    digitalWrite(MICROSCOPE_DOWN_PIN, LOW);
  }
}

void setup()
{
  // set up Serial Monitor
  #if DEBUG
    pinMode (LED_BUILTIN, OUTPUT) ;
    Serial.begin (115200) ;
    while (!Serial) {
      delay (50) ;
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    }
  #endif
  // set up CAN 
  ACAN_T4_Settings settings (100 * 1000) ; // 100 kbit/s - must agree on both ends of CAN
  const uint32_t errorCode = ACAN_T4::can3.begin (settings) ;
  if (0 == errorCode) {
    #if DEBUG
      Serial.println ("can3 ok") ;
    #endif
  }else{
    #if DEBUG
      Serial.print ("Error can3: 0x") ;
      Serial.println (errorCode, HEX) ;
    #endif
    while (1) {
      delay (100) ;
      #if DEBUG
        Serial.println ("Invalid setting") ;
      #endif
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    }
  }

    // setup motors
  pinMode(ACTUATOR_UP_PIN, OUTPUT);
  pinMode(ACTUATOR_DOWN_PIN, OUTPUT);
  pinMode(MICROSCOPE_UP_PIN, OUTPUT);
  pinMode(MICROSCOPE_DOWN_PIN, OUTPUT);
  digitalWrite(ACTUATOR_UP_PIN, LOW);
  digitalWrite(ACTUATOR_DOWN_PIN, LOW);
  digitalWrite(MICROSCOPE_UP_PIN, LOW);
  digitalWrite(MICROSCOPE_DOWN_PIN, LOW);

  carousel.attach(CAROUSEL_PIN, 900, 2100);
  fan.attach(FAN_PIN, 900, 2100);
  // all motors should start in neutral position (resting for motors, 90 degrees for servos)
  carousel.write(90);
  fan.write(90);

  pinMode(CAROUSEL_LIMIT_PIN, INPUT_PULLUP); // digital signal
  pinMode(MICROSCOPE_ENCODER_PIN, INPUT); // analog signal

}

void loop()
{
  // receive CAN message if one is available
  if (ACAN_T4::can3.receive (message)) {
    #if DEBUG // print the message for debugging
    Serial.println("Message received");
      if(message.id == 0x03) {
        Serial.print("ID=");
        Serial.print(message.id);
        Serial.print(" msg = ");
        for(int i=0; i<message.len; ++i) {
          Serial.print(message.data[i]);
          Serial.print(", ");
        }
        Serial.println();
      }
    #endif

    if(message.id == SCIENCE_ID) { // science id = 0x03
      if(message.len == 4) { // CAN performs it's own checksum so no need to check that
        updateMotors(message);
      }
    }
  }

  // If switch is triggered while carousel is moving, stop carousel. This should allign test tube properly
 if(!digitalRead(CAROUSEL_LIMIT_PIN) && carouselMoving && (carouselSpeed == 1)) {
   //Serial.println("limited");
   carousel.write(90);
   carouselMoving = false;
 }

  unsigned long curr_time = millis();
  // stop all motors after 1 second of no messages
  if (curr_time - stopTimeout >= 1000) {
    stopTimeout = millis();
    carousel.write(90);
    fan.write(90);

    #if DEBUG
      Serial.println("Stopped motors");
    #endif
  }
}
