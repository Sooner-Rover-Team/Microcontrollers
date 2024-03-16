# Microcontrollers
This folder contains all microcontroller code that has been used on SORO in the past. Some examples are provided for testing. We are currently only using Teensy 4.1 microcontrollers.

## Data Communication

### UDP

User-Datagram Protocol (UDP) is the current form of wireless internet communication for sending manual control commands to the rover. These UDP messages are sent to a EBOX Teensy 4.1 microcontroller connected to the rover router. UDP does not require a response msg from the reciever before you can send the message (thats how TCP works), which means we can send data more often in low signal areas.

Message Structures:

- Wheels = [0x01, 0x01, w0, w1, w2, w3, w4, w5, checkSum]
- LEDs = [0x01, 0x02, r, g, b]
- Arm = [0x02, bicep, forearm, base, wrist_pitch, wrist_roll, claw, checkSum]
- Science = [0x03, big_actuator, drill, small_actuator, test_tubes, camera_servo, checkSum]

### CAN

Controller-Area-Network (CAN) is the current form of wired data transfer among microcontrollers on the rover. These CAN messages are shared amongst the entire network, and any microcontroller is capable of send/recv. The CAN msg ID determines the message priority. If two messages are being sent at the same time on the wire bus, the message with the lower ID number will be sent and the higher ID message will wait until the lower ID messages are done sending. This is called CAN arbitration.

Message Structures:

- Controls msg Ebox to Arm: LOWER ARM =(id=0x01, [bicep, forearm, base]), UPPER ARM =(id=0x02, [wrist_pitch, wrist_roll, claw])
- Controls msg Ebox to Science: id=0x03, [big_actuator, drill, small_actuator, test_tubes, camera_servo]
- Sensor msg Sciecne to Ebox: id=0x04, [temperature, humidity, methane1, methane2]