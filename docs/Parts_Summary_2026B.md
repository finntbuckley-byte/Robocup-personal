<!-- Text copy of Parts_Summary_2026B.pdf (Robocup Documentation 2026 V0.95), extracted with pdftotext -layout on 2026-09-27. Figures (connector/pin diagrams) are NOT in this text - open the PDF for those. -->

Robocup

Documentation
    2026

Parts Summary
 Robocup
                 Documentation 2026 V0.95
      Contents
Parts Summary Page 1   03
Parts Summary Page 2   04
Actuators     05
Sensors     06
Hardware Box 1    07
Hardware Box 2    08
Hardware Box 3    09
Wires     10
Green Box     11
Blue Box     12
Red Box     13
Black Box     14
Additional Materials    14
Services Available    15
Tools Available    15
Robot Example Chase Assembly  16
Parts Supply     23
CPU      25
Raw Teensy Pins    26
Port Expanders    27
I2C Summary    27
Expander I2C Connections   28
CPU Connectors & Board layout  29
Software     32
Software Library    33
Power Supply    34
Hookup Diagrams    36

CPU Board Schematic   59
CPU Board Pins    60
CPU Board Pins Digital   61
Actuators       Robocup
                      Documentation 2026 V0.95
                 Parts Summary

                          Sensors

Hardware Box 1  Hardware Box 2

Hardware Box 3   Wires
Green Box  Robocup
                 Documentation 2026 V0.95
                     Blue Box

Black Box  Red Box
                                   Robocup
                                         Documentation 2026 V0.95

Actuators

ID Qty IK CostDescription                                   Part Number Major Parameters

A 4 2 14.0 Standard servo                                   HX12K      7.4V, 9.4kg.cm

B  4 2 02.5Small servo                                      SG90       5V 1.5kg.cm

C  2 1 22.0DC Motor 90RPM , includes gearbox and encoder    JGY-370    12V, 90RPM

D  4 2 58.0Smart servo                                      DRS-0101   7.4V, 12kg.cm, Gear: 1:266

E  2 2 15.0DC Motor 200RPM, includes gear box               SKU505979 12V, 200RPM

F  2 1 06.6Electromagnet                                    JK-P30/22  12V, 0.4A, 10kg

G  2 2 15.0DC Motor 10RPM, includes gearbox                 SKU365231 12V, 10RPM

H  2 2 70.0 DC Motor 143RPM, includes gearbox and encoder   28PA51G    12V, 143RPM, 3.6A, 5.5kgf.cm

I  1 1 03.8 Solenoid                                        TAU-0530T 12V, 1.5A, 0.4N-7N

J  1 0 04.8Small DC Motor with gearbox                      N20        12V, 0.3A, 148RPM

K  2 1 50.0 Stepper motor with gearbox NEMA17               36PA5.2G/42BYG40-160-4A 1.6A,1.8deg/step

L  2 2 32.0 Stepper motor NEMA17                            42BYGHM809 1.7A, 0.9deg/step

From Green box

M  1 60.0 Large Servo                                       RDS5160    7.4V, 5A, 65kg.cm, 270deg

N  1 01.6 Speaker                                           MP001193 0.5W, 8ohm

O  1 04.0 Fan                                               MC32893    12V, 0.15A
                                         Robocup
                                               Documentation 2026 V0.95

Sensors

ID Qty IKCost Description                                Part Number        Major Parameters

A  2 1 01.0 Ultrasound IO Board                          HC-SR04            4-30cm
                                                         VL53L0XV2          10 to 80cm
B  4 2 01.8 Low cost ultrasound sensors                  VL53L1XV2          20-150cm
                                                         TFmini             1kg, 10kg
C  2 1 05.0 TOF I2C                                      0A41SK             BNO055+BMP280
                                                         GP2Y0A21YK0F       100-500cm
D  2 1 10.0 TOF I2C                                      GP2Y0A02YK0F       
                                                                             10k
E  1 1 50.0 TOF Serial                                   
                                                         HX711              300-5000mm`
F  2 2 08.0 IR Distance Sensor                           SEN0158
                                                         SEN0253
G  2 2 08.0 IR Distance Sensor                           2Y0A710K

H   2 2 08.0 IR Distance Sensor                          TCS34725
                                                         SV-163-1C25
I  2 1 06.0 IR Distance Sensor, Adjustable, Digital Out
                                                         
J  2 1 03.5 Load Cell
                                                         HRLV-MaxSonar-EZ0
K  1 1 02.0 Load Sensor

L  1 1 30.0 IR Camera

M  1 1 36.0 IMU

N  2 2 25.0 IR Distance Sensor

O  2 1 25.0  Inductive Proximity

P  1 1 14.0 Color Sensor

Q  5 2 00.5 Microswitches

R 2 1  Traffic light LED

S  2 1 01.0 Variable resistor

T   1 1 02.2 Rotary Encoder

U   1 1 01.5 Analogue Joystick

V 1 1  RGB LED

W 2 1  Smart LED x16 array

X   1 1 01.5 Digital Joystick

X 2 0  Smart LED Single

X   2 0 60.0 Ultrasonic distance sensor
      Robocup
                                                Documentation 2026 V0.95

Hardware Box 1

ID  Qty Cost Description                   Part Number                    Major Parameters
A1 1  02.2  Anti Backlash Nut                                             T8 screw Lead 8mm
A2 1  01.8  Anti Backlash Nut                                             T8 screw Lead 8mm
B 2  03.5  Trapezoidal lead screw block
C 1   02.0  Linear rail support block            LM8UU, 8mm
D1 3  02.0  Shaft Coupler
D2 2  01.0  Shaft Coupler                        KP08
E1 1  03.5  Rod ends                        KFL08
E2 1  02.5  Rod ends
F 4  01.8  Rigid Flange Coupling           20-XL-10BF
G 2  02.5  Trapezoidal lead screw nut
H 4  01.2  Linear Ball Bearings,           F608ZZ                         200mm long
I1 1  02.2  Servo arm Aluminum 25T         MGN9 C 
I2 1  05.0  Servo arm Aluminum 25T Long
J 2  02.5  Pillow block bearing 
K  2  04.5  Flange pillow block bearing, 
L 4  00.8  Plastic pulley wheels, Nylon
M 4  00.6  Aluminum 90 degree bracket
N 6  02.5  Pulley GT2
O 2  06.0  Pulley XL
P 8  02.5  Aluminum cutouts
Q 6  02.5  Aluminum cutouts
R 5  00.6  Plastic hinge
S 16  01.0  Flanged bearings 
T 12  03.0  Drive track support hardware
U 2  18.0  Linear rail
      Robocup
                                                 Documentation 2026 V0.95

Hardware Box 2

ID Qty Cost  Description                        Part Number                Major Parameters
A1 2                                                                       300, 150, 100mm available
A2 2         Trapezoidal lead screw  300mm              
A3 2                                                                       223.5mm
B 2          Trapezoidal lead screw  150mm                                 2m open ended, closed loop
C 2  
D 9          Trapezoidal lead screw  100mm                                 300, 210, 150, 120, 90, 60, 45, 30mm
E1 1  
E2 1         Robot tracks                       880-8M
E3 1  
F 16         Timing belt.                       320 XL

             Open beam Aluminum profile, Robot main chassis support,  

             Timing belt  open ended,           GT2  

             Timing belt                        GT2

             Timing belt                        GT2

             Open beam Aluminum profile                 
       Robocup
                                                         Documentation 2026 V0.95

Hardware Box 3

ID Qty Cost Description                                     

    Aluminium extrusion profiles, 300mm lengths
    A selection of the following lengths are in the kit...
    A 25mm right angle aluminium
    B 12mm right angle aluminium
    C 12mm box section aluminium
    D 12mm U shaped aluminium
    E 25mm flat bar aluminium
    F 12mm flat bar aluminium
    G 8mm round bar aluminium
    H 6.5mm round bar aluminium

Longer lengths of Aluminium extrusion profiles are available on request.
                                      Robocup
                                            Documentation 2026 V0.95

Wires

ID Qty Cost  Description
A            3 pin cable
B            4 pin cable
C            5 pin cable
D            6 pin cable
E            8 pin cable
G            smart servo IO cable
H            smart servo chain cable
I            RC Servo extender cable
J            11v cable
K            motor extender cable
                                Robocup
                                      Documentation 2026 V0.95

Green Box

ID  Qty Cost Description            

A 2  20.0 Mecanum Wheel

B 2  07.5 Skate wheel

C 2    Lego wheel

D 4  04.0 Omni wheel

E 1  60.0 Large servo

F 1  04.0 Fan

G 1  01.6 Speaker

H 1  01.0 GT2 Belt

I 2  05.0 Small tank tracks

J 1    Belt

K 2  25.0 Main drive wheel

L 2  08.0 Small drive wheel

M 2  16.0 Medium drive wheel

N 1    Weight 750g

O 1    Weight 500g

P 1    Weight 1000g
                                     Robocup
                                           Documentation 2026 V0.95

Blue Box

ID  Qty Cost Description                         

A 1    CPU                           Tennsy 4.0

B 1    Stop Go button

C 1    Power supply board

D 2  20.0 Motor drive board          DFRobot DFR0513 PPM 2x3A DC Motor Driver
                                     TB6560 3A Stepper motor drives
E 2  10.0 Stepper motor drive board

F 1    Cables
                              Robocup
                                    Documentation 2026 V0.95

Red Box

ID  Qty IK Cost Description                 

A 2  1   Motor Drive IO Board

B 1  1   Encoder IO Board

C 1  1   Smart Servo IO Board

D 2  1   Digital level shift IO Board

E 2  1   Serial level shift IO Board

F 4  1 01.5 Solid state relay board

G 4  2   Analogue IO Board
                                  Robocup
                                        Documentation 2026 V0.95

Black Box

ID  Qty Cost Description 

A  2         Robot side plate, see file Body02.zip in 08Models for hole layout     

B 1          Robot top plate, holes spaced 20mm apart, 3mm hole

C 2          Medium tank track

D 2          Main drive pulley, held on flat of drive motor shaft with two grub screws, 2mm hex tool to tighten

Additional Materials

ID Qty Cost  Description                                         

A 2 02.5     Corflute sheet 600*600*3mm
B12 06.0
B22 04.0     MDF sheet 600*600*6mm
C 2  04.0
D 2  04.0    MDF sheet 600*600*4.75mm
E 2  06.0
F 2  09.0    Aluminium sheet 300*300*2mm
G 1  16.0
H 1  20.0    Perspex sheet 300*300*2mm
I 1 15.0
J  3 01.0    Perspex sheet 300*300*4.5mm

             Perspex sheet 300*300*6mm

             Perspex sheet 300*300*10mm

             3D Printing Filament 500g, 5c per gram for PLA

             Additional 1m of open beam Aluminium extrusion

             Magnets, Countersunk Ring Magnets 22mm x 5mm, Hole: 5.2mm, N50 Neodymium Magnet

Larger sizes of materials are available on request
       Robocup
                                                         Documentation 2026 V0.95

   Services Available

Email or see Julian Murphy with your job requests.
julian.murphy@canterbury.ac.nz
Office next to 3D printers, West end of lab.
Office hours 9am to 5pm Monday to Friday

Specialised 3D Printing, supply file in STEP or 3MF file format, Max size 250*210*210mm
Laser cutting, supply file in DXF file format, Max size 1200*900*10mm
Waterjet cutting, supply file in DXF file format, Max size 300*300*20mm
Power guillotine, supply printed diagram or marked out on sheet to cut, straight lines only
Parts repair and or replacement
Parts purchasing, supply links of website to purchase in email

     Tools Available

Mechatronics Lab(24 Hour access)
Soldering iron, Oscilloscope, Signal Generator, Power supply
Parts and robot storage, battery storage and charging.
Nuts and bolts
Tools can be found on the peg board, or in the Yellow boxes

Mechatronics Lab II(24 Hour access)
There is a second space available with 21 stations, in the Flexible Projects Room 220.

3D Printing(24 Hour access)
PRUSA I3 MK3s and MK4 printer with 0.6mm AND 0.4mm nozzles
PRUSA MINI printer with 0.4mm nozzle
PRUSA XL by request for larger items
PRUSA Core one L
Creality K1 Max
Printing material mostly PLA, with 2 printers having PETG, one ASA, and one TPU
One printer has a 0.25mm nozzle fitted

Student Workshop (Open 9-5 week days, Non Supervised)
Hand tools
Bender, roller, hand guillotine, notcher

Main Workshop (Available 9-4, Supervision)
Lathe, mill, drill, drop saw
       Robocup
                                                         Documentation 2026 V0.95

          3D Printing

                   Max size PRUSA Mini 180 * 180 * 180
               Max size PRUSA I3S or MK4 250 * 210 * 220

                    Max size PRUSA XL 360 * 360 * 360
               Max size PRUSA Core One L 300 * 300 * 330

                     Creality K1 Max 300 * 300 * 300

Do not move equipment between AMES lab, student workshop and Mechatronics Lab
Must have attended a training course before use.
Must fill out a job card.
Check printed parts draws for parts.
Standard PLA 0.4mm, some machine 0.6 for faster printing

                 Training is required before access is granted
                             Visit learn to find out how

              learn.canterbury.ac.nz/course/view.php?id=3120
                                      Robocup
                                            Documentation 2026 V0.95

                     Robot Example Chase Assembly

Tools Required:      13mm Spanner

Parts Required:      Hex keys, 6mm, 2.5mm, 2mm
                     1x container of M3 x 6mm button head hex bolts
                     4x M3 Nuts

                     1x top plate

                     2x side plate

                     2x DC geared motor

                     2x pulley

                     2x large track

                     6x open beam 224mm length

                     8x bolt assemblies

Steps
1  Attach motor

  Attach DC motor using supplied 4 x M3 6mm button head bolts using the smallest 2mm hex key to side plate

2 Attach track supports
          Attach bolt 4 x bolt assemblies using 13mm spanner and 6mm hex key as indicated in diagram
          A single fat washer and nylock nut go on the inside of the side plate for each assembly

3 Attach support beams
          Before attaching open beam to side plate slide in two M3 nuts to the top two support beams
          Attach open beam support rods to side plate as indicated in diagram

4 Attach pulley
          Place spacer between side plate and pulley.
          Push on pulley to hard against spacer
          Make sure 2.5mm hex key hole is aligned with flat surface of motor shaft
          Do up 2.5mm grub screw
          Rotate pulley 90 degrees and do up second grub screw

5 Attach belt

  Slide bottom bolt assembly in slotted hole to the left.

  Attach belt, and adjust tension using slotted hole

  You should not have to force the belt on, strength is not required

  Now slide the bolt assembly to the right to tension the belt.

6 Attach top plate using 4 x M3 bolt

Important points:

The DC motor bolts must not be any longer than 6mm. Longer bolts will destroy the motor.

Remember the M3 nuts in the top open beam support.

The slotted hole allows adjustment of belt tension.

Pulley grub screw must be done up tight, and aligned with the flat of the motor shaft.

There is a second pulley grub screw.

Use a spacer to set gap between pulley and top of button head hex bolts.                              
       Robocup
                                                         Documentation 2026 V0.95
Tools

Parts
                               Robocup
                                     Documentation 2026 V0.95

Attach DC Motor to side panel
       Robocup
                                                         Documentation 2026 V0.95
Attach support beams and track support
       Robocup
                                                         Documentation 2026 V0.95
Attach pulley to DC motor, use spacer to set gap between button head bolts and pulley


              Robocup
                    Documentation 2026 V0.95

Attach track

Attach track with track support to the right, once track on frame slide to left to tension track.
               Robocup
                     Documentation 2026 V0.95

Spacer Layout
       Robocup
                                                         Documentation 2026 V0.95

                          Parts Supply

Supplied parts are to be keeped in their current form, i.e. can-not be cut or drilled into.
Some additional parts are available on request.

         3x Mecrum wheels
         2x Large scooter wheels
         More open beam lengths
         Smaller main motor drive pulley
         2nd electromagnet
         2nd 90 rpm encoder worm motor
         2 x 100 rpm worm motor
         2nd geared stepper motor
         3rd and 4th standard sized servo
         4 x rare earth magnets
         2nd linear rail, or shorter linear rail 100mm

Parts purchase requests must be sent to technician by email.
If you believe your part to be broken or faulty, they can be exchanged for a replacement part.

                       Notes about parts

A design requirement is that you use items given to you in the kit in their current form.

Do not disassemble the modules.
If you require a smaller IR distance sensor, smaller versions are available to swap.

Do not disassemble the motors, or servos.
If you want continuous rotation from servos, some are available by request.

Do not add any holes or cut the side plates.
You can bolt on extra metal bits or have a whole new side panel cut for you using the waterjet.

Do not cut the Open beam Al extrusion, additional items are available on request.

Do not cut up provided cables.
If you want shorter or longer cables, you can make your own with provided cables in cabinets below the
batteries storage area.

What can be cut up?
AL extrusion is available for you to cut up
MDF board is available to be laser cut.
Perspex is available to be laser cut.
AL sheet is available to be waterjet cut.
       Robocup
                                                         Documentation 2026 V0.95
Additional steel or Al is available via request, or from the main workshop.

                           Actuators

Stepper motors require a stepper motor driver board. Control signal: 2 digital line, step and direction.
DC Motors require a DC motor driver board, Control signal: 2 digital lines.
Electromagnet and solenoid require an electronic switch driver board. Control signal: 1 digital line.
Servos can connect to CPU through an adapter board to supply higher current. Control signal: 1 digital line, servo signal
Smart servos can connect to CPU through an adapter board to supply higher current. Control signal: 2 serial lines.

Main drive motors can supply position feedback to CPU via 2 digital lines, needs special cable 2mm 6 pin
Smart servos can supply position feedback TO CPU via Rx serial line, needs special cable 2mm 4 pin
90rpm worm motors can supply position feedback via 2 digital lines.
       Robocup
                                                         Documentation 2026 V0.95
                               CPU
The CPU of the board is a Tennsy 4.0 with an ARM Cortex-M7 running at 600 MHz.
For more technical information about the board and microprocessor take a look at
www.pjrc.com/store/teensy40.html

Teensy 4.0 pins accept 0 to 3.3V signals. The pins are NOT 5V tolerant. Do not drive any digital pin higher than 3.3V

To drive any actuator you will need a dedicated driver board.
CPU Ports
The microcontroller has a number of ports that can change their function based on your needs.
Arbitrary decisions have been made and various ports have been assigned a purpose.
SERIAL  There are 3 labelled serial ports. These contain 2 digital lines.  5 pin cable
DIGITAL There are 2 labelled digital ports. These contain 4 digital lines.  8 pin cable
ANALOGUE There are 10 labelled analogue ports. These contain 1 digital line. 3 pin cable
A number of the IO boards contain both the SERIAL and DIGITAL port connectors.
You should only hock ONE of these up at a time.

The CPU board actually consists of 4 separate sub assemblies. To use these, each assembly must be connected to the
CPU using a 4 pin I2C cable. Each assembly also has its own unique I2C address.
Robocup
      Documentation 2026 V0.95

Raw Teensy Pins
       Robocup
                                                         Documentation 2026 V0.95

Port Expanders

The CPU board contains a number of additional features.
One of the features is the use of port expanders.
The board contains 2 port expanders, each at a different I2C address.
The left hand side contains seven 2 pin ports that can be used for limit switches.
They are labelled AIO0 to AIO6
These ports contain pullup resistors.
There are also eight 3 pin ports to be used for digital sensors.
They are labelled AIO8 to AIO15
To use the port expander the IC needs to be connected to the I2C bus, this is done with a 4 pin cable.

I2C Summary

I2C Address                                     Part #
                                                BNO055
0x28 IMU                                        VL53L1X ,VL53L0X
                                                TSC34725
0x29 TOF Sensor     
                                                SSD1306
0x29 Color Sensor                               SX1509 00
                                                SX1509 01
0x30 0x30-0x38 TOF Sensor Reassign
                                                VL53L5CX
0x33 DFRobot TOF Imager Sensor, 8x8 zones
                                                TCA9548
0x3C OLED                                       SX1509 11

0x3E Limit switches AIO0-AIO15   

0x3F TOF Control XSHUT0-XSHUT7 and BIO8-BIO12

0x40 0x40-0x4f TOF Sensor Reassign 2nd space

0x52 TOF Imager Sensor, 8x8 zones             

0x60 0x60-0x6f TOF Sensor Reassign 3rd space
0x70 Multiplexor Control    
0x71 TOF Control XSHUT0-XSHUT15 on expander 

In order to use the TOF and Color sensor in the same design there are 3 strategies that can be taken...
1 Use I2C Multiplexor
2 Use separate I2C busses, the CPU has two I2C buses labelled I2C0 and I2C1
3 Use supplied address translator board
Robocup
      Documentation 2026 V0.95
Robocup
      Documentation 2026 V0.95
Robocup
      Documentation 2026 V0.95
Robocup
      Documentation 2026 V0.95
       Robocup
                                                         Documentation 2026 V0.95
                            Software
All machines in the Mechatronics lab have the software already installed. Shortcut to start the IDE can be found at
C:\_shortcuts\programming
Do not use the default IDE from the start menu, as it does not contain the ARM extensions.
If you want to install on your own laptop there is software to download and install.
www.arduino.cc/en/software
Once installed start the IDE and chose the correct processor from the menu

Select a speed, not everything works at 600MHz, I would suggest 150MHz and increase it later if needed.

To send a program to the micro press the upload button
       Robocup
                                                         Documentation 2026 V0.95

                  Software Library

Some of the provided examples need additional libraries installed to work correctly.
SX1509 IO Expander by Sparkfun, Version 2.0.1 for CPU IO Expansion
VL53l0X by Pololu Version 1.3.1 for TOF Sensor
VL53l1X by Pololu Version 1.3.1 for TOF Sensor
FastLED by Daniel Garcia for Smart LED's
Adafruit BNO055 for IMU
Adafruit TCS34725 for Color Sensor
DFRobot HX711 for Weight Sensor

How to install libraries, once opened Library manager in Arduino IDE, use keyword to narrow down choices.
       Robocup
                                                         Documentation 2026 V0.95

                         Power Supply

The battery MUST be connected to this board. All power sourced for other components comes from this board.
Press and hold the start button for 2 seconds to enable power. Press the stop button for 1 second to cut power.
The Go button can be wired separately to the CPU board and used as a digital input to start your program running.
Points to take note of. The 5V Motor Power connector does not provide 3V on the connector.
Although the 13V and 7.5V supplies have the same connector, the third pin is not connected to the other power supply.
       Robocup
                                                         Documentation 2026 V0.95
                        Power Expander
This board provides a mechanism to get additional power connection points.

                      The Blue GO button

Use a 3 pin cable to connect the blue go button to the CPU board, best spot to connect to is a 3 pin analogue in
port.
       Robocup
                                                         Documentation 2026 V0.95

                       Hookup Diagrams

Example code is available with hookup diagrams. Download the examples from the Learn website, and refer
to the supplied hookup diagrams.
These are reference examples used to learn how things work. You should try to understand the electrical
connections, and follow the example code.
Each of the examples works in isolation to the other examples. To use these examples together the user will
need to study the documentation , plug the sensors or actuators into another port and make the correct port
reassignment in the code.

Sensors
101_IMU
102_Color
103_IR_XYPosition
104_TOF_Short
105_TOF_Long
106_TOF_X8
107_TOF_SerialMini
108_TOF_SerialMicro
109_IR_Distance
110_Ultrasound_Smart
111_Ultrasound_Digital
112_LoadCell
113_Encoder
114_IR_Distance_Digital
115_LimitSwitch
116_MagSensor
117_MagRotary_Sensor
118_Joystick_Digital
119_Joystick_Analogue
120_8421_Encoder
121_Twist_Encoder
122_OpticalFlow_Sensor
123_RFID
                            Robocup
                                  Documentation 2026 V0.95

Actuators
201_SmartServo
202_StepperMotor
203_DCMotor
204_Servo
205_Electromagnet
206_Speaker
207_Solenoid

Indicators
301_Onboard_LED
302_TrafficLight_LED
303_RGB_LED
304_SmartLED_Single
305_SmartLED_Strip
306_SmartLED_Square
307_SmartLED_Rectangle
308_OLED_Display

Debugging
401_Bluetooth_HC05
402_Bluetooth_CH9143
403_Wifi
404_SDCard

Wiring Boards
501_Serial_Level_Shift
502_Digital_Level_Shift
503_Inductive_Interface
504_FET_Driver
505_TOF_Expander
506_Fuse_Indicator
507_Ultrasound_Interface
508_Motor_Interface
509_SmartServo_Interface
510_Encoder_Interface
511_IRSensor_Interface
512_IO_MAP_Interface
513_I2C_Expander_Interface
514_Servo_Isolator
                    Robocup
                          Documentation 2026 V0.95

Power Boards
601_Power_Extender
602_Power_Filter
603_Signal_Filter
       Robocup
                                                         Documentation 2026 V0.95

201_SmartServo

This actuator needs to be connected to a serial port.
Smart servos are tunable and can give position feedback.
Run off 7.5V power supply output.
Each smart servo can be programmed with a unique ID that is used to identify it.
The ID is written on the side of the servo.
The cable connecting the servo to the IO board is a special cable provided in the kit.
There is a program on the PC called HerkuleX Manager that can be used to configure the servos parameters.
Also needed to connected to PC is a USB cable that is available at the front for the room.
       Robocup
                                                         Documentation 2026 V0.95

202_StepperMotor

This actuator can be connected to any digital pin.
Two control signals , step and direction
       Robocup
                                                         Documentation 2026 V0.95

203_DCMotor

This actuator can be connected to any digital pin.
Used to drive any DC motor including the worm gear motor. Two channels.
Uses servo signal as control input, ie a pulse that varies in width from 1ms to 2ms.
1.05ms full speed reverse
1.5ms stop
1.950ms full speed forward
The motor controller will ignore any signal outside what it sees as a valid range
       Robocup
                                                         Documentation 2026 V0.95

204_Servo

This actuator can be connected to any digital pin.
Servos can draw a lot of power, several amps.
So it's best to use the Digital Level shift board to supply the servo with power from an external source.

205_Electromagnet

This actuator can be connected to any digital pin.
This should be connected to CPU using the FET_Driver.
       Robocup
                                                         Documentation 2026 V0.95

206_Speaker

This actuator can be connected to any digital pin, using driver board.
This should be connected to CPU using the FET_Driver board.

207_Solenoid

This actuator can be connected to any digital pin, using driver board.
This should be connected to the CPU using the FET_Driver board.
       Robocup
                                                         Documentation 2026 V0.95

101_IMU

This sensor needs to be connected to the I2C bus.
Sensor used BNO055

102_Color

This sensor needs to be connected to the I2C bus.
Sensor used TCS34725
       Robocup
                                                         Documentation 2026 V0.95

103_IR_XYPosition

This sensor needs to be connected to the I2C bus.
Part number SEN0158
       Robocup
                                                         Documentation 2026 V0.95

TOF

This sensor needs to be connected to the I2C bus.
There are 2 different modules available.

104_TOF_Short

VL53L0X  30-2000mm Blue

105_TOF_Long

VL53L1X  40-4000mm Purple
                       Robocup
                             Documentation 2026 V0.95

Multi Sensor Examples
       Robocup
                                                         Documentation 2026 V0.95

106_TOF_X8

This sensor needs to be connected to the I2C bus. This part is the DFRobot SEN0628. I2C Address 0x33.
Requires the following library to be installed. github.com/DFRobot/DFRobot_MatrixLidar

Additional info can be found at wiki.dfrobot.com/sen0628/
       Robocup
                                                         Documentation 2026 V0.95

107_TOF_Serial

This sensor needs to be connected to a serial bus.
This sensor needs 5V to operate hence the need for the serial level shift board
TFmini  100-12000mm White

108_TOF_SerialMicro

Currently not in kit.
       Robocup
                                                         Documentation 2026 V0.95

109_IR_Distance

This sensor needs to be connected to an analogue input line.
Yellow 0A41SK 40­300mm
White 2Y0A21 100­800mm
Blue  2Y0A02 200-1500mm
Purple2Y0A710 1000-5500mm

It is possible to swap the larger sensor mount for this smaller sensor mount of the same type
       Robocup
                                                         Documentation 2026 V0.95

RJ11 mount connection diagrams

The RJ11 cable fits into the connector labelled CON3.
With the sensor facing you this is the left hand connector.
Maximum of 2 sensors can be connected using the RJ11 cable.

Small RJ11 mount connection diagram
       Robocup
                                                         Documentation 2026 V0.95

110_Ultrasound_Smart

Currently not in kit

111_Ultrasound_Digital

This sensor needs to be connected to any digital line, requires 2 digital lines.
Allows easy connection of low cost Ultrasound sensors to CPU.
Also used for connecting to load cell board.
HC-SR04
       Robocup
                                                         Documentation 2026 V0.95

112_LoadCell

This sensor needs to be connected to any digital line, requires 2 digital lines.
Amplifier board used HX711
Load cells available 1 or 10kg
       Robocup
                                                         Documentation 2026 V0.95

113_Encoder

This sensor needs to be connected to any digital line.
The main drive DC motors contain a magnetic encoder.
It is possible to determine direction and distance the motor has travelled.
This IO board can be also used with the encoder on the worm gear motor.
Some of the motors have the encoder cable permanently attached, some use a separate cable.
       Robocup
                                                         Documentation 2026 V0.95

114_IR_Distance_Digital

This sensor needs to be connected to any digital line.
The sensor is generally wired into the IO expander.
This sensor needs 5V to operate hence the need for the digital level shift board
       Robocup
                                                         Documentation 2026 V0.95

115_LimitSwitch

This sensor needs to be connected to any digital line.
The sensor is generally wired into the IO expander.
Internally there are pullup resistors on the digital lines.
Hence closing the switch will pull the digital line to GND.
To use this IO expander the chip needs to be wired to the CPU.
This is done with the 4pin cable connecting the IO expander to the CPU's I2C bus.
The address of this IO expander is
       Robocup
                                                         Documentation 2026 V0.95

116_MagSensor

117_MagRotary_Sensor

118_Joystick_Digital

No cable is provided, you will have to make your own.
       Robocup
                                                         Documentation 2026 V0.95

119_Joystick_Analogue

No cable is provided, you will have to make your own.
Uses 2 analogue inputs to detect X and Y movement of joystick.

120_8421_Encoder

Can be used as a binary input to the CPU. Uses 4 digital pins

121_Twist_Encoder

Essentially a rotary encoder. Provides with software forward and reverse detection of knob turns.
Uses 2 digital channels.
       Robocup
                                                         Documentation 2026 V0.95

122_OpticalFlow_Sensor

Optical navigation chip. Provides XY motion information. Working range 80mm to infinity
This sensor plugs into the SPI port. Uses the PMW3901 optical flow sensor
You must use the supplied yellow 7 pin XY labeled cable, wires have been reconfigured.
Do not use a generic 7 pin cable.
Requires Bitcraze PMW3901 library to be installed. Install from Arduino library manager
Additional Info...

github.com/bitcraze/Bitcraze_PMW3901
https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.40-280119.pdf

123_RFID

The robot playing area has NFC sensors embedded into the floor.
       Robocup
                                                         Documentation 2026 V0.95

Indicators
301_Onboard_LED

The CPU board has a set of LED's built into the board.
Jumper leads are required to connect these to a pin.

302_TrafficLight_LED

Jumper leads or 4 pin socket are required to connect this module to a pin.

303_RGB_LED

Jumper leads or 4 pin socket are required to connect this module to a pin.
       Robocup
                                                         Documentation 2026 V0.95

304_Smart_LED_Single

Special supplied cable is required.
This module requires 5V power, so requires digital level shifter board.

305_Smart_LED_Strip

Special supplied cable is required.
This module requires 5V power, so requires digital level shifter board.

306_Smart_LED_Square

Special supplied cable is required.
This module requires 5V power, so requires digital level shifter board.

307_Smart_LED_Rectangle

Special supplied cable is required.
This module requires 5V power, so requires digital level shifter board.
       Robocup
                                                         Documentation 2026 V0.95

308_OLED_Display

This module is a 128X64 pixel display and uses I2C for communications and a SSD1306 driver chip.
       Robocup
                                                         Documentation 2026 V0.95

Debugging
401_Bluetooth_HCO5

Pair with your phone and then use a terminal to interact with the CPU board. This board connects to the
Teensy CPU through a selected serial connector, ie SERIAL1, SERIAL2, or SERIAL7.
Appears as HC-05 on bluetooth scanner. On an Android the app is called Serial Bluetooth Terminal.

402_Bluetooth_CH9143

This set of boards is a matched set that sets a serial connection between the 2 boards using a bluetooth
connection. One board connects to the Teensy CPU through a selected serial connector, ie SERIAL1, SERIAL2, or
SERIAL7. The other matched board connects to a computer. On the computer the board will appear as a serial
port, ie COM20. Use a serial terminal program such as Teraterm to use the datalink interactively. It's also
possible to write a Python program to plot, visualise and send commands to your CPU over a wireless link.
                      Robocup
                            Documentation 2026 V0.95

403_Wifi

Currently not in kit

404_SDCard

Currently not in kit
       Robocup
                                                         Documentation 2026 V0.95

Wiring Boards
501_Serial_Level_Shift

3v to 5v level shift. Currently used for NFC or Serial TOF communications board.

502_Digital_Level_Shift

3v to 5v level shift, along with external power supply.
Used for smart LED's and servos.
Allows you to supply larger current to servo by using an external power supply cable.
To use an external power supply the jumper must be in the Ex position.
In the In position the board draws 3V power from the CPU board and converts it to 5V Power. The level of current this
process can supply is limited.
       Robocup
                                                         Documentation 2026 V0.95

503_Inductive_Interface

3v to 12v level shift.
Generally used for inductive sensors. Must be supplied with 11V from the power module.
Channels A and B are for the larger green sensors.
Channels C and D are for smaller black inductive sensors.

504_FET_Driver

Optically isolated driver board used to drive magnets, motors, or speaker.
Can be used to drive motors at different speeds using PWM, but can not reverse the direction of motor.
       Robocup
                                                         Documentation 2026 V0.95

505_TOF_Expander

Additional TOF channels for even more TOF sensors.
This board works just the same as onboard TOF IO board but with a different I2C address of 0x71.

506_FUSE_Indicator

Gives a visual indication on whether a fuse on the CPU board has blown.
The fuse needs to be replaced by the lab technician, who has boards to swap for the broken one.
Power pins on the CPU board can be checked with a multimeter to see if the fuse is blown.
When making the measurement be careful not to short the power pins as this will instantly blow a fuse.
A fuse will blow for a reason. The subsections have a 2A fuse and the main power input has a 5A fuse.
Reasons for a blown fuse...
Misswired cable,
Bolt, nut, or other metal dropped onto bare pins of CPU or other board.
Lose circuit board or sensor wires touching metal chase.
5V or 12V circuits wired directly into CPU (This will also destroy the CPU)
       Robocup
                                                         Documentation 2026 V0.95

507_Ultrasound_Interface

Allows connection of ultrasound sensors to CPU board.

508_Motor_Interface

Allows connection of CPU to stepper motor driver board.

509_SmartServo_Interface

Allows connection of smart servo to CPU. 7.5V power must be supplied via MT30 cable.
Connects to serial port on CPU.
       Robocup
                                                         Documentation 2026 V0.95

510_Encoder_Interface

Allows the encoder signals from main drive motors, or worm gear motors to be interfaced with CPU.

511_IRSensor_Interface

Allows IR sensors that use RJ11 cables to be interfaced with CPU.
A maximum of 2 channels can be connected with the RJ11 cable.

512_IO_MAP_Interface

Allows remapping of header pins to different XH2.54 cable
       Robocup
                                                         Documentation 2026 V0.95

513_I2C_Expander_Interface

Expands the number of I2C sockets from 3 to 9 to allow more items to be connected to the I2C bus.
       Robocup
                                                         Documentation 2026 V0.95

514_Servo_Isolator

It has been discovered that if you hit the servo arm hard with an object that a back EMF is generated which
propagates along the control wire and can cause the CPU to reset. To help prevent this the servo isolator
board was manufactured. This places an optical barrier between the CPU and servo on the control wire.
       Robocup
                                                         Documentation 2026 V0.95
                                 +

Power Boards
601_Power_Extender

Planned for future implementation

602_Power_Filter

Providers a LC filter on the power lines to help remove high frequency motor noise from the power lines.

603_Signal_Filter

Planned for future implementation
