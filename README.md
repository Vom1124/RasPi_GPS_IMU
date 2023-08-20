# RasPi_GPS_IMU
This is an example to publish GPS and IMU data using gps_rtk2 and any VECTORNAV device, respectively. 
The project was created in ROS2 Foxy running Ubuntu 20.04 in Raspberry Pi 4.  
The ROS2 workspace is written in Foxy version but should be compatible with Humble and other future ROS2 versions.
This workspace contains two packages; namely, gps_rtk2 and imu. The ros2 nodes are individually created for publishing the gps data and the imu data.

Clone this repository using 

    git clone https://github.com/Vom1124/RasPi_GPS_IMU.git


Prerequisites:

  Packages to be installed:

  1) GPS_RTK2: 
      The packages required to be installed additional to the python is pyserial for python serial port communication and pynmea2 for NMEA 0183 message structure, which are to be installed using the following.

         pip3 install pyserial
       for pyserial and
     
          pip3 install pynmea2
       for NMEA 0183 message protocol.
       
     The node uses usb communication using '/dev/ttyACM0'. But check this value by verifying using

           ls /dev/tty*
     
     Usually, it's ttyACM* by default, but can take the value 0,1,2,... depending upon how it configures the USB connection.

  3) VECTORNAV IMU:
     This requires a python package called "vnpy" which needs to be installed from the vnprogib provided by the VECTORNAV manufacturer. Once the library is obtained; travese through the python folder under vnproglib directory and run

           python3 setup.py install

     to install the vnpy package.

     Similar to GPS_RTK2, this uses serial communication as well. Depending on the device type, the system will configure and recognize differently. For this device, the Linux recognized as "/dev/ttyACM*". Check the number once again if it's 0,1,2, ... by displaying all the devices under tty using "ls /dev/tty*" command.

Once all the necessary packages are installed and verified, simply start the ros2 nodes as*

            ros2 run gps_rtk2 gps_read

  for publishing gps data, and 
    
            ros2 run imu imu_read
  
  for publishing imu data.

*NOTE: Don't forget to build using     

            colcon build --symlink-install

and sourcing the terminal before running the nodes. At least build and source once before opening a new terminal to avoid any error. 
