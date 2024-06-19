# gripper-rover
Code for rover controller board using ESP32

Compiler: Arduino IDE, version 1.8.19

Connectivity: bluetooth

Electronics:
- 1x ESP32
- MPU6050 gyro
Basic rover:
- 2x DC motor for differential drive (or 2 motors for each side)
- 2x servo motor, one for gripper and one for arm lift

Swerve rover:
- 2x additional servo motor for rotating the forward direction of the wheels  

Data protocol for movement:
- magic byte
- x1 byte for sideways movement while maintaining direction. Achieved by turning direction momentarily, then returning to previous direction using gyro. Otherwise, with swerve capable hardware, set movement direction while maintaining rover heading.
- y1 byte for forward-reverse movement
- x2 byte for turning direction
- s byte for arm lift servo position in degrees
- b byte for various signals and flags

x1, y1, x2 are centered (no movement) in the value 128. 255 means full power right or forward, 0 means left or reverse.

b has the bits organized as follows:

- bit 7: unused
- bit 6: 1 = gripper open, 0 = no action
- bit 5: 1 = gripper hold, 0 = no action
- bit 4: unused
- bit 3: unused
- bit 2: unused
- bit 1: 1 = function 2 signal
- bit 0: 1 = function 1 signal

Additional commands are single byte, may be followed by data.
- '0' = motor disable
- '1' = motor enable
- 'L' = left turn 90 degrees when gyro is available
- 'R' = right turn 90 degrees
- 'r' = read PID constants
- 'p' followed by floating point number in string representation = set Kp
- 'i' followed by float string = set Ki
- 'd' followed by float string = set Kd
- 's' = save PID constants to EEPROM
- 'W' = move forward with high power
- 'w' = move forward with low power
- 'S' = reverse with low power
- 'X' = reverse with high power
- 'A' = turn left
- 'D' = turn right
- 'P' followed by 3 bytes representing high power setting, low power setting, turning power setting = set power values when going with WSAD commands.
