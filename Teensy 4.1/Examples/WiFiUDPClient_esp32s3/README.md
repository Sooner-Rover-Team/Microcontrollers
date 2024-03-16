# manual_operate

This code can drive the rover, move the arm, and operate the science package. It requires an XBOX or PS4/PS5 controller to operate.

## Dependencies
At the moment, python 3.10 or higher, pygame and zmq are required to run manual_operate.py.

Python can be downloaded [here](https://www.python.org/downloads/)

Download the libraries using 
```
pip install pygame 
pip install zmq
```

## Usage

The config.ini file must have the correct ip addresses of the EBOX microcontroller, as it controls the wheels and distributes data to arm/science via CAN-Bus.

Ebox Teensy 4.1 IP: 192.168.1.101
Ebox Teensy 4.1 Port: 1001

The controller configuration can also be adjusted according to the physical controller being used. In config.ini, change `[Controller] CONFIG = ` to `ps` or `xbox`

`config.ini` stores the IP addresses of different devices and the controller configuration

`util.py` stores functions for the UI design and the arm Inverse Kinematics

`Listener.py` handles communication with the GPS via the jetson. Requires the zmq python libraray. To start the GPS publishing, ssh into the remi jetson computer on the rover and start the send_locations.py file in the home directory.

`/extras` stores extra Mission Control functionality that is still work-in-progress

## Data Communication

# UDP

User-Datagram Protocol (UDP) is the current form of wireless internet communication for sending manual control commands to the rover. These UDP messages are sent to a EBOX Teensy 4.1 microcontroller connected to the rover router.

Message Structures:
- Wheels = [0x01, 0x01, w0, w1, w2, w3, w4, w5, checkSum]
- LEDs = [0x01, 0x02, r, g, b]
- Arm = [0x02, bicep, forearm, base, wrist_pitch, wrist_roll, claw, checkSum]
- Science = [0x03, big_actuator, drill, small_actuator, test_tubes, camera_servo, checkSum]

Wheel Speeds are integer values 0-255 where 0 is full reverse, 127 is neutral and 255 is full forward. Each Teensy microcontroller has an 'ID' that we have chosen for it. Ebox Teensy is 0x01, Arm is 0x02, Science is 0x03. If you are sending a UDP message to the arm, the data goes to the EBOX microcontroller, which then converts the data to CAN and sends the arm message through the CAN-Bus on the rover to the arm, same for the science package.

## Controls

There are two distinct modes in the program: drive and operate. Drive obviously just drives the wheels and changes LEDs. Operate can be used to control either the science package or the arm, depending on which is installed (can be toggled by pressing **Select**). **B** switches between drive and operate.

### Drive

The rover drives with tank controls (**Left Stick** moves left wheels, **Right Stick** moves right wheels). The **Left Bumper** will only move the front wheels while the **Right Bumper** will only move the back wheels. This could be useful for getting the rover unstuck. **A** will just make the lights flash. 

### Arm

The arm is controlled with the help of the GUI. The **Left Stick** controls the point of the wrist in space and the actuators will automatically adjust their length to keep the wrist on that point. The **Right Stick** controls the tilt and rotation of the wrist. The operator must be careful not to overtwist the wrist as the cords can get tangled and disconnected if the wrist is rotated too far. **A** opens and closes the claw. **Left Trigger** rotates the base to the left while **Right Trigger** rotates the base to the right.

### Science Package

The Science package has 4 distinct controllable elements. The **Left Stick** moves the drill up and down in space. The **Right Stick** controls the speed of the drill. **Right Trigger** increases the speed of the vacuum while **Left Trigger** decreases it. **Left Bumper** rotates the carousel clockwise while **Right Bumper** rotates the carousel counterclockwise. **A** attempts to move the carousel one seventh of a rotation, but it is based on time elapsed and may not move the same amout each time.
