/*
    THIS TEENSY RECEIVES UDP MESSAGES FROM ROUTER AND SENDS
      CAN MESSAGES TO OTHER MICROCONTROLLERS ON ROVER. IT IS
      ALSO RESPONSIBLE FOR CONTROLLING WHEELS

     Pin wiring diagram

     [],pin  front   [],pin
     0,6     0-|-0    3,3
               |
     1,7     0-|-0   -4,4
               |
    -2,8     0-|-0   -5,5
     "-" indicaes wheel's polarity needs to be reversed

     gimbal pan: 
     gimbal tilt: 

     LED pins RGB: 11,10,12

     Pins used by Ethernet:

     Wheel UDP message:
     [0x01, 0x01, wheel0, wheel1, wheel2, wheel3, wheel4, wheel5, wheel6, checkSum]

     LED UDP message:
     [0x01, 0x02, redLED, greenLED, blueLED]

     ARM UDP Message:
     [0x02, bicep, forearm, base, pitch, roll, claw]

     SCIENCE UDP Message:
     [0x03, actuator, carousel, fan, microscope]
*/

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.1"
#endif

#include <NativeEthernet.h> // NativeEthernet.h and NativeEthernetUDP.h are the exact same as the arduino Ethernet.h and EtherbetUDP.h.
#include <NativeEthernetUdp.h> // Since they are the same, you can use arduino ethernet examples and just change the #include to run on Teensy
#include <ACAN_T4.h>
#include <Servo.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 101);
unsigned int localPort = 1001; // Local port to listen on

IPAddress ip_out(192, 168, 1, 101);
unsigned int outPort = 1255;

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetUDP Udp_out;

// Buffer for receiving and sending data over UDP
unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

// CANMessage instances to send and receive messages
CANMessage message;
CANMessage recMessage;

// SET DEBUG == 1: PRINT ETHERNET MSGS, DEBUG == 2: PRINT ARM, DEBUG == 3: PRINT SCIENCE
#define DEBUG 2 // set to 0 to avoid compiling print statements (will save space, don't need to print if running on rover)

// UDP IDs
#define WHEEL 0x01 // Wheels are [0x01, 0x01]
#define LED 0x02   // LEDs are [0x01, 0x02]
#define ARM 0x02

// UDP and CAN ID for science package
#define SCIENCE 0x03

// CAN IDs
#define LOWER_ARM 0x01
#define UPPER_ARM 0x02
#define EBOX 0x04
#define CAN_LED 2
#define UDP_LED 1
#define greenPin 10 // LED pins on Teensy
#define redPin 11
#define bluePin 12

int checkSum = 0; // used to check for errors in recieved message.

// helper variables to store data temporarily
int vertPos = 90; // position of gimbal
int horizPos = 90; 
int lastEncoderVal = 0;
int lastTemperature = 0;
int lastMoisture = 0;
int lastMethane = 0;

// CAN LED essentially informs user when msgs are being received/sent
int canLedMsgCount = 0; // counter for # msgs received to flash the CAN LED

// WHEEEEEELS These objects write PWM signals to TALON motor controllers in wheels
Servo wheel0, wheel1, wheel2, wheel3, wheel4, wheel5, gimbalVert, gimbalHoriz;

unsigned long timeOut = 0; // used to measure time between msgs. If we go a full second without new msgs, stop wheels so rover doesn't run away from us

// super awesome fun proportional wheel control variables
int targetSpeeds[6] = {126, 126, 126, 126, 126, 126}; // neutral
double currentSpeeds[6] = {126, 126, 126, 126, 126, 126};
bool proportionalControl = true;
unsigned long lastLoop = 0; // milli time of last loop
double deltaLoop = 0.0; // seconds since last loop
double Kp = 3.5; // proportional change between target and current
double Kp_thresh = 0.4; // % of wheel speed to apply proportional control under
double error[6]; // stores error of each wheel for PID calculations

// define time in microseconds of width of pulse
const int PWM_LOW = 1000;
const int PWM_HIGH = 2000;
const int PWM_NEUTRAL = 1500; 


/*******************************************************
 *** INITIALIZE MOTOR PINS, WIFI MODULE & CAN MODULE ***
 *******************************************************/

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(LED_BUILTIN, OUTPUT);
  #if DEBUG
    Serial.begin(115200);
    delay(100);
    Serial.println("Starting");
  #endif

  /* PWM FOR MOTORS IS USUALLY AT 50 Hz FREQ OR 20 MILLISECONDS
  * wheel0.attach(6, 1000, 2000) means PWM out of Pin 6, and PWM Varies from 1000 to 2000 microseconds.
  *    This means when u write wheel0.write(0), the PWM output is high for 1 millisecond and low for 19
  *    milliseconds. Similarly, wheel0.write(180) outputs high for 2 milliseconds and low or 18.
  * We should honestly change back to using Analog.write for frequency control....
  */

  wheel0.attach(6, 1000, 2000);
  wheel1.attach(7, 1000, 2000);
  wheel2.attach(8, 1000, 2000);
  wheel3.attach(3, 1000, 2000);
  wheel4.attach(4, 1000, 2000);
  wheel5.attach(5, 1000, 2000);

  wheel0.write(90);
  delay(5);
  wheel1.write(90);
  delay(5);
  wheel2.write(90);
  delay(5);
  wheel3.write(90);
  delay(5);
  wheel4.write(90);
  delay(5);
  wheel5.write(90);

  pinMode(redPin, OUTPUT); // red
  pinMode(greenPin, OUTPUT); // green
  pinMode(bluePin, OUTPUT); // blue
  pinMode(CAN_LED, OUTPUT);

  // Start the Ethernet
  Ethernet.begin(mac, ip);
  #if DEBUG == 1 || DEBUG == 0
    Serial.println("Starting ethernet");
  #endif

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    #if DEBUG == 1 || DEBUG == 0
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    #endif
    while (true) {
      delay(1); // Do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    #if DEBUG == 1 || DEBUG == 0
      Serial.println("Ethernet cable is not connected.");
    #endif
  }

  // Initialize the CAN settings and start
  ACAN_T4_Settings settings (100 * 1000) ; // 100 kbit/s - must agree on both ends of CAN
  const uint32_t errorCode = ACAN_T4::can3.begin (settings) ;
  if (0 == errorCode) {
    #if DEBUG == 2 || DEBUG == 0
      Serial.println ("can3 ok") ;
    #endif
  }else{
    #if DEBUG == 2 || DEBUG == 0
      Serial.print ("Error can3: 0x") ;
      Serial.println (errorCode, HEX) ;
    #endif
    while (1) {
      delay (100) ;
      #if DEBUG == 2 || DEBUG == 0
        Serial.println ("Invalid setting") ;
      #endif
      digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    }
  }

  // Start UDP listening
  Udp.begin(localPort);
  Udp_out.begin(outPort);
}

void printCanMsg(CANMessage &frame){
    for(int x=0;x<frame.len;x++) {
      Serial.print (frame.data[x]); Serial.print(":");
    }
    Serial.println ("");
}

// Clips a value so it stays in encoderAnglerange [low, high]
double clip(double value, double low, double high) {
  if (value < low) value = low;
  if (value > high) value = high;
  return value;
}

/*************************************************
 *** UDP MSGS BACK TO MISSION CONTROL ************
 *************************************************/

void sendEncoder(CANMessage encodermsg) {
  int msg1 = encodermsg.data[1] * 255;
  int msg2 = encodermsg.data[0];
  if (msg1 + msg2 == lastEncoderVal || abs(msg1 + msg2 - lastEncoderVal) < 20) {
    return;
  }
  lastEncoderVal = msg1 + msg2;
  Udp_out.beginPacket(Udp.remoteIP(), outPort);
  byte encoder[2];
  encoder[0] = encodermsg.data[0];
  encoder[1] = encodermsg.data[1];
  Udp_out.write(encoder, 2);
  Udp_out.endPacket();
  return;
}

void sendSensorData(CANMessage sensormsg) {
  // if (lastTemperature != sensormsg.data[0]) lastTemperature = sensormsg.data[0];
  // if (lastMoisture != sensormsg.data[1]) lastMoisture = sensormsg.data[1];
  // if (lastMethane != sensormsg.data[3]) lastMethane = sensormsg.data[3];
  // Serial.println("sensors sent to mission control");
 

  // Send msg back to mission control
  int error = Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //same ip its receiving on, but different port
  if (error == 0) {
    Serial.println("ERROR");
  }
  else {
    // Serial.print("Sendto: ");
    // Serial.print(Udp.remoteIP());
    // Serial.print(" ");
    // Serial.println(Udp.remotePort());
  }
  byte msg[4];
  msg[0] = sensormsg.data[0]; // temperature
  msg[1] = sensormsg.data[1]; // humidity
  msg[2] = sensormsg.data[2]; // methane
  msg[3] = sensormsg.data[3]; // methane
  #if DEBUG == 2 || DEBUG == 0
    Serial.print("Science sensor udp: ");
    for (int i=0; i < 4; i++) {
      Serial.print(msg[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
  #endif
  Udp.write(msg, 4);
  Udp.endPacket();
  return;
}

/*************************************************
 *** CAN MSGS TO ARM/ SCIENCE PACKAGE ************
 *************************************************/

void sendArmCAN(unsigned char msg[], int msgSize) {
  if(msgSize == 8) {
    checkSum = 0;
    for(int i=1; i<7; i++) { 
        checkSum += msg[i];
    }
    checkSum = checkSum & 0xff;

    if(checkSum == uint8_t(msg[7])) {
      message.id = LOWER_ARM; 
      message.len = 3;
      memcpy(message.data, &msg[1], 3); // base, bicep, forearm
      bool ok = ACAN_T4::can3.tryToSend (message) ;
      if(ok && (DEBUG == 2 || DEBUG == 0)) {
        Serial.println("lower sent"); 
      }

      message.id = UPPER_ARM;
      message.len = 3;
      memcpy(message.data, &msg[4], 3); // pitch, roll, claw
      ok = ACAN_T4::can3.tryToSend (message) ;
      if(ok && (DEBUG == 2 || DEBUG == 0)) {
        Serial.println("upper sent"); 
      }
    }
    else {
      #if (DEBUG == 2 || DEBUG == 0)
        Serial.println("checkSum for ARM was incorrect... ignoring this message.");
      #endif
    }
  }
  else {
    #if (DEBUG == 2 || DEBUG == 0)
      Serial.println("checkSum for ARM was incorrect... ignoring this message.");
    #endif
  }
}

void sendScienceCAN(unsigned char msg[], int msgSize) {
  #if (DEBUG == 2 || DEBUG == 0)
    Serial.println("science can time");
  #endif

  if(msgSize == 7) {
    checkSum = 0;
    for(int i=1; i<6; i++) { 
        checkSum += msg[i];
    }
    checkSum = checkSum & 0xff;
    Serial.println(checkSum);
    Serial.println(uint8_t(msg[6]));

    if(checkSum == uint8_t(msg[6])) {
      message.id = SCIENCE; 
      message.len = 5;
      memcpy(message.data, &msg[1], 5); // actuator1, actuator2, drill, slide rail, panoramic
      bool ok = ACAN_T4::can3.tryToSend(message);
      if(ok && DEBUG) {
        Serial.println("Science sent");
      }
      else {
        #if (DEBUG == 2 || DEBUG == 0)
          Serial.println("Error sending science message");
        #endif  
      }
    }
    else {
      #if (DEBUG == 2 || DEBUG == 0)
        Serial.println("checkSum for SCIENCE was incorrect... ignoring this message.");
      #endif
    }
  }
  else {
    #if (DEBUG == 2 || DEBUG == 0)
      //   Serial.println("checkSum for SCIENCE was incorrect... ignoring this message.");
      // Serial.println("length for SCIENCE was incorrect... ignoring this message.");
    #endif
  }
}

/*************************************************
 *** USE WHEEL AND LED MSGS TO DO STUFF **********
 *************************************************/

//LED msg: [0x01, 0x02, red, green, blue]
//WHEEL msg: [0x01, 0x01, w1, w2, w3, w4, w5, w6, checkSum]
void updateWheels(unsigned char msg[], int msgSize) {
  checkSum = 0;

  /**************LED MSG *****************/
  if (msg[1] == LED) {
    if(msgSize == 5) {
      if(uint8_t(msg[2]) > 0) {
        digitalWrite(redPin, HIGH);
        Serial.println("RED HIGH");
      }
      else {
        digitalWrite(redPin, LOW);
      }
      if(uint8_t(msg[3]) > 0) {
        digitalWrite(greenPin, HIGH);
      }
      else {
        digitalWrite(greenPin, LOW);
      }
      if(uint8_t(msg[4]) > 0) {
        digitalWrite(bluePin, HIGH);
      }
      else {
        digitalWrite(bluePin, LOW);
      }
    } 
    else {
      #if DEBUG == 1 || DEBUG == 0
        Serial.println("msg for LEDS was wrong length... ignoring this message.");
      #endif
    }
  }  

  /**************WHEEL MSG *****************/
  else if (msg[1] == WHEEL) {
    if(msgSize == 9) {
      for(int i=2; i<8; i++) { 
        checkSum += msg[i];
      }
      checkSum = checkSum & 0xff;

      // #if DEBUG // commented because its hard to read other print statements, keep in case its needed
        // Serial.print("Calcuated checksum: ");
        // Serial.println(checkSum);
        // Serial.print("Received checksum: ");
        // Serial.println((int)msg[8]);
      // #endif

      if(checkSum == uint8_t(msg[8])) {
        timeOut = millis(); // save time so we know how long it's been between this and next msg
        // set the appropriate target speeds
        for (int i = 0; i < 6; i++) {
          targetSpeeds[i] = (uint8_t)msg[i+2];
        }       
      }          
      else {
        #if DEBUG == 1 || DEBUG == 0
          Serial.println("checkSum for WHEELS was incorrect... ignoring this message.");
        #endif
      }
    }
    else {
      #if DEBUG == 1 || DEBUG == 0
        Serial.println("msg for WHEELS was wrong length... ignoring this message.");
      #endif
    }
  }
  else {
    #if DEBUG == 1 || DEBUG == 0
      Serial.println("msg was for the wrong device... ignoring this message.");
    #endif
  }
}

/*******************************************
 *** LOOP THAT LITERALLY KEEPS REMI ALIVE **
 *******************************************/
 // ^^^^ IF THIS LOOP PAUSES, REMI LITERALLY CANNOT DO ANYTHING !!!
 // WHATS A GOOD BACKUP PLAN>>??
// int count = 0;
float lastLoopScience = 0;
void loop() {
  // count++;
  
  if(ACAN_T4::can3.receive(recMessage)) {
      if(recMessage.id == 0x04) {
        if(recMessage.len == 2) {
          sendEncoder(recMessage);
        }
        if(recMessage.len == 4) {     // temperature, moisture, methane
          // if ((millis() - lastLoopScience)/1000.0 > 1) { 
          //   sendSensorData(recMessage);
          //   // count = 0;
          //   lastLoopScience = millis();
          // }
          sendSensorData(recMessage);
        }
      }
  }

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    canLedMsgCount++;
    if (canLedMsgCount == 1) {
      digitalWrite(CAN_LED, HIGH); // Turn LED on first msg, turn off after 10 recieved, repeat
    }

    #if (DEBUG == 1 || DEBUG == 0) // Output IP address and packet size
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remote = Udp.remoteIP();
      for (int i=0; i < 4; i++) {
        Serial.print(remote[i], DEC);
        if (i < 3) {
          Serial.print(".");
        }
      }
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
    #endif

    // Read the packet into packetBuffer
    Udp.read(packetBuffer, packetSize);
    #if (DEBUG == 1 || DEBUG == 0)
      for(int i=0; i<packetSize; ++i) {
        Serial.print(packetBuffer[i]);
        Serial.print(", ");
      }
      Serial.println();
    #endif

    // do stuff with received msg
    if(packetBuffer[0] == WHEEL) { // means the UDP message was for the wheel system
      updateWheels(packetBuffer, packetSize);
    }
    else if(packetBuffer[0] == ARM) {
      sendArmCAN(packetBuffer, packetSize);
    }
    else if(packetBuffer[0] == SCIENCE) {
      sendScienceCAN(packetBuffer, packetSize);
    }
    else {
      #if (DEBUG == 1 || DEBUG == 0)
        Serial.println("ID unrecognized");
      #endif
    }
    if (canLedMsgCount > 10) { // CAN LED essentially flashes every 10 msgs, which should flash slow enough to see
      digitalWrite(CAN_LED, LOW);
      canLedMsgCount = 0;
    }
  }
  if ( millis() - timeOut >= 1000) { // if the last good msg recieved was longer than 1 sec ago, stop wheels
    timeOut = millis();
    for (int i = 0; i < 6; i++) {
      // wheels[i].write(90);
      // delay(5);
      currentSpeeds[i] = 126;
      targetSpeeds[i] = 126;
    }

    wheel0.write(90);
    wheel1.write(90);
    wheel2.write(90);
    wheel3.write(90);
    wheel4.write(90);
    wheel5.write(90);

    #if DEBUG
      Serial.println("Stopped motors");
    #endif
  } 
  else {
    /*
    * PID Control - Currrently only uses proportional control to slowly ramp the wheels to target speeds.
    *   Mission control data fills targetSpeeds and PID modifies currentSpeeds until the target is reached.
    */
    deltaLoop = (millis() - lastLoop)/1000.0;
    lastLoop = millis();
    for (int i = 0; i < 6; i++) {
      // if using proportional control, look at the difference between the current and desired speed
      if (proportionalControl && abs(currentSpeeds[i] - 126) < (126*Kp_thresh)) {
        error[i] = targetSpeeds[i] - currentSpeeds[i];
        double output = error[i]*Kp*deltaLoop;
        currentSpeeds[i] += output;
      } 
      else {
        currentSpeeds[i] = targetSpeeds[i];
      }
      // if the target is to stop, set the current to that as well
      if (targetSpeeds[i] == 126) currentSpeeds[i] = 126;
      // also clip the current speed value just to be safe
      currentSpeeds[i] = clip(currentSpeeds[i], 0, 252);
    }
    // END OF PID
    
    // actually update wheel speeds
    wheel0.write((int)map(currentSpeeds[0], 252, 0, 0, 180));
    wheel1.write((int)map(currentSpeeds[1], 252, 0, 0, 180));
    wheel2.write((int)map(currentSpeeds[2], 0, 252, 0, 180));
    wheel3.write((int)map(currentSpeeds[3], 252, 0, 0, 180));
    wheel4.write((int)map(currentSpeeds[4], 252, 0, 0, 180));
    wheel5.write((int)map(currentSpeeds[5], 252, 0, 0, 180));
  }
}