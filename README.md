# gripper-rover
Code for rover controller board using ESP32

Compiler: Arduino IDE, version 1.8.19

Connectivity: bluetooth

Electronics:
- 1x ESP32
- 2x DC motor for differential drive (or 2 motors for each side)
- 2x servo motor, one for gripper and one for arm lift
- MPU6050 gyro

Data protocol:
- magic byte
- x1 byte for sideways movement while maintaining direction. Achieved by turning direction momentarily, then returning to previous direction using gyro.
- y1 byte for forward-reverse
- x2 byte for turning direction
- s byte for arm lift servo position in degrees
- b byte for various signals and flags
