#Arduino code for the Teensy 3.6 microcontroller
##On host PC
###Setup Arduino 
https://www.arduino.cc/en/Guide/Linux
<br/>
https://www.arduino.cc/download_handler.php?f=/arduino-1.8.1-linux64.tar.xz
<br/>

###Setup Teensyduino 
https://www.pjrc.com/teensy/td_download.html
<br/>
Save the text in the link below as a file named "49-teensy.rules"
<br/>
https://www.pjrc.com/teensy/49-teensy.rules
<br/>
sudo cp 49-teensy.rules /etc/udev/rules.d/


###Install ROS Kinetic packages and ros_lib
sudo apt-get install ros-kinetic-rosserial-arduino ros-kinetic-rosserial -y
<br/>
<br/>
**If there is a ros_lib in the arduino /sketchbook/libraries delete it**
<br/>
**Remake the ros_lib library**
<br/>
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
<br/>
<br/>
Find ArduinoHardware.h in ros_lib edit **line 44** to add support for Teensy 3.5 & 3.6
<br/>
\#if defined(\_\_MK20DX128\_\_) || defined(\_\_MK20DX256\_\_) || defined(\_\_MK64FX512\_\_) || defined(\_\_MK66FX1M0\_\_)
<br/>
https://github.com/ros-drivers/rosserial/blob/jade-devel/rosserial_arduino/src/ros_lib/ArduinoHardware.h
