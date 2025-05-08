kvasir_lidar
============

This repository contains the code concerning the Xiaomi LDS02RR LiDAR for our Kvasir project. In particular, it
includes (at the time of writing):
- An Arduino sketch, written for a VCC-GND YD-RP2040 and utilizing the
  [LDS](https://github.com/kaiaai/LDS) library from Kaia AI, which utilizes the MCU and a DC motor
  driver to control the LiDAR via its UART interface, reporting back the data via a serial
  connection with the computer;
- A debugging script, written in Python, which reads the data from the serial port and plots it in
  real time via Matplotlib.

Pinout diagram
--------------

The pinout diagram for the LiDAR is as follows:

![LiDAR pinout](assets/lidar_pinout.png)

`mot+` and `mot-` connect to the motor driver, `tx` to UART0 RX, `vcc` and `gnd` connect to 
a 5V power supply and `nc`, obviously, is not connected to anything.

In our testing, 12V is an appropriate supply voltage for the motor driver. The internal PID
algorithm of the sketch code adjusts the speed of the motor to maintain a constant 5 Hz rotation
speed, which gives a 1° resolution due to the sampling rate employed.

Roadmap
-------

- [x] Arduino sketch
- [x] Python debugging script
- [ ] ROS2 node to publish LiDAR data (cfr. [LaserScan message](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/LaserScan.html))
- [ ] Rewrite with native Pico SDK (maybe?)
