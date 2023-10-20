# RasPi_GPS_IMU
This is an example to publish GPS and IMU data using gps_rtk2 and any VECTORNAV device, respectively. 
The project was created in ROS2 Foxy running Ubuntu 20.04 and executed and tested with ROS2 Humble running Ubuntu 22.04 LTS in Raspberry Pi 4.  
The ROS2 workspace is written in Foxy version but should be compatible with Humble and other future ROS2 versions.
This workspace contains two packages; namely, gps_rtk2 and imu. The ros2 nodes are individually created for publishing the gps data and the imu data.

Clone this repository using 

    git clone https://github.com/Vom1124/RasPi_GPS_IMU.git


### Prerequisites:

  #### Packages to be installed:

  1) GPS_RTK2: 
      The packages required to be installed additional to the python is pyserial for python serial port communication and pynmea2 for NMEA 0183 message structure, which are to be installed using the following.

        a) For pyserial.

         pip3 install pyserial
        b) For NMEA 0183 message protocol.

         pip3 install pynmea2
       
       
     The node uses usb communication using '/dev/ttyACM0'. But check this value by verifying using

         ls /dev/tty*
     
     Usually, it's ttyACM* by default, but can take the value 0,1,2,... depending upon how it configures the USB connection.

     Edit the permission for the connected USB device if not done before or if a new OS is installed

         sudo chmod 0777 /dev/ttyACM0
     This above permission might be required multiple times depending on the USB port used.

  3) VECTORNAV IMU:
     This requires a python package called "vnpy" which needs to be installed from the vnprogib provided by the VECTORNAV manufacturer. Once the library is obtained; travese through the python folder under vnproglib directory and run the below code to install the vnpy package.

         sudo python3 setup.py install
 Secondly, in order to read the IMU data in Euler angles (Yaw, Pitch, and Roll), the Quaernion data output from the VectorNav device needs to be converted from Quaternion to Euler angles. Since the VectorNav already uses the "scalar last" method to represent the Quaternion, simply using the euler_from_quaternion function from the package tf_transformation in python can ease the conversion process, as such this example adopts this method. There are couple of libraries to be installed before importing the tf_transformations package in python.
  
  a) First, the transforms3d package is a pre-requisite to use tf_transformations library which needs to be installed as shown below.

        pip3 install transforms3d

  b) Now, the tf_transformations package itself needs to be installed, which can be done using the code below.

             sudo apt-get install ros-humble-tf-transformations
Once the necessary packages are installed; the devices permission is provided Similar to GPS_RTK2, this uses serial communication as well. Depending on the device type, the system will configure and recognize differently. For this device, the Linux recognized as "/dev/ttyUSB*". Check the number once again if it's 0,1,2, ... by displaying all the devices under tty using "ls /dev/tty*" command.

 Again, edit the permission to the connected USB device to give the communication permission

         sudo chmod 0777 /dev/ttyUSB0
This above permission might be required multiple times depending on the USB port used.

### Starting the nodes:

Once all the necessary packages are installed and verified, simply start the ros2 nodes as*

            ros2 run gps_rtk2 gps_read

  for publishing gps data, and 
    
            ros2 run imu imu_read
  
  for publishing imu data.

*NOTE: Don't forget to build and source the terminal before running the nodes. At least build and source once before opening a new terminal to avoid any error. 

            colcon build --symlink-install

