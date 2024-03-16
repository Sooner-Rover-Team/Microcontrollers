#include <EtherCard.h>
#include <IPAddress.h>
#include "mcp2515_can.h"
#include <SPI.h>

/*
 * TODO: 
 *  - configure base station side to send the correct numbers.
 *  - fix linActuator limit switches to only move up when down limit is on and only down when up is on.
 */


/*
 * NANO PWM pins: D3, D5, D6, D9, D10, D11
 * 
 * PINS USED BY ETHERNET SHEILD: D10, D11, D12, D13 (it uses SPI)
 * 
 * USING TALONS TO CONTROL ALL MOTORS (ACTYUATOR/CAROUSEL) -> TREAT EACH MOTOR AS A CONTINUOUS SERVO
 *  The carousel and linear actuator are going to have external limit switches to monitor their positions.
 * 
 * LINEAR ACTUATOR - D2/D3
 * CAROUSEL - D5

 * FAN - D6
 * MICROSCOPE - D9
 */

// Set equal to 1 for serial debugging
#define DEBUG 1


const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

// ethernet interface ip address (static ip)
static byte myip[] = {10, 0, 0, 102};
//static byte myip[] = {192, 168, 1, 101};
static int myport = 1002;
// gateway ip address
static byte gwip[] = {10, 0, 0, 1};
//static byte gwip[] = {192, 168, 1, 1};
// ethernet mac address - must be unique on your network
static byte mymac[] = {0xAC, 0xDC, 0x0D, 0xAD, 0x00, 0x00};
// tcp/ip send and receive buffer
byte Ethernet::buffer[500];

// raw bytes to store from ethernet data
// they are converted to output ranges in updateServos()
uint8_t myHash = 0;
uint8_t serialHash = 0;

// CAN shit
unsigned char stmp[5];
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin



unsigned long stopTimeout = 0, turnTimeout = 0;

// callback that prints received packets to the serial port
void udpSerialPrint(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, const char *data, uint16_t len)
{
  IPAddress src(src_ip[0], src_ip[1], src_ip[2], src_ip[3]);
  if((len == 8) && (data[0] == 0xff)) {
    myHash = data[1] + data[2] + data[3] + data[4] + data[5] + data[6];
  }

  CAN.MCP_CAN::sendMsgBuf(0x01, 0, 5, stmp); // id = 0x00, standrad frame, data len = 8, stmp: data buf


  // SCIENCE msg: [startByte, deviceID, linearActuator, carousel, fan, microscope, checkSum]
  // hash = (sum of data bytes) / (num of bytes) - Don't include startByte or deviceID in sum
  
}

void setup()
{
  #if DEBUG
    Serial.begin(9600);
    if (ether.begin(sizeof Ethernet::buffer, mymac, 9) == 0)
      Serial.println(F("Failed to access Ethernet controller"));
  #else
    ether.begin(sizeof Ethernet::buffer, mymac, 9);
  #endif

  ether.staticSetup(myip, gwip);
  #if DEBUG
    ether.printIp("IP:  ", ether.myip);
    ether.printIp("GW:  ", ether.gwip);
    ether.printIp("DNS: ", ether.dnsip);
  #endif

  // register udpSerialPrint() to port
  ether.udpServerListenOnPort(&udpSerialPrint, myport);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
      Serial.println("CAN init fail, retry...");
      delay(100);
  }
  Serial.println("CAN init ok!");

}

void loop()
{
  // this must be called for ethercard functions to work.
  ether.packetLoop(ether.packetReceive());
}

void updateServos() {
     
}

