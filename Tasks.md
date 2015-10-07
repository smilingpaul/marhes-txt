# Tasks #

---

**Microcontroller Only**

  1. Tune encoder odometry - I need to make sure that what the micro is reading from the wheel encoders and sending to ROS is correct.
  1. Create PID loop - I need to create a PID loop to control the linear and angular velocities.

**Microcontroller & ROS**

  1. Battery stop code - Get the A/D converters working so we can measure the cell voltages of the 11.1V Li+ Batteries.  Use this information to warn the user and/or shutdown the TXT1 so the battery doesn't explode.
    1. Get A/D working and divide the voltages to measure them.
    1. ~~Create a packet between micro and ROS to tell voltages/warning/life.~~
    1. ~~Broadcast a battery msg in ROS.~~
  1. Get Kalman Filter Pkg working - There is a Kalman filter package in ROS that should be tested out.  http://www.ros.org/wiki/robot_pose_ekf.  There are a few tasks to implement this.  Some of these involve working with the micro also
    1. Test the pkg with wheel odometry and imu to get odom\_combined in ROS
    1. Create packet between ROS and micro for odom\_combined
    1. ~~Incorporate a switch between combined and encoder odometry in case the imu/other sensors aren't connected.~~
  1. ~~Incorporate a safety msg where ROS continuously sends the micro a packet that it is still alive.  If the micro doesn't receive the packet stop.  This is a safety mechanism if something fails.  The pioneers have this feature.~~
    1. ~~Create Mac Mini Connected Packet between ROS and micro~~
    1. ~~Make the micro have a timeout and stop if connected packet not received~~
  1. Create Vicon odom msg - I have the Vicon system working in ROS with positions, but I need to get velocity information and maybe other information.  I might use this information on the micro, but I'm not sure yet.

**Mechanical**

  1. Test Mac Mini pwr supply w/ new batteries
    1. he batteries are 11.1V Li+ batteries.  They need to be tested with the pwr supply (http://www.mini-box.com/M2-ATX-160w-Intelligent-Automotive-DC-DC-Power-Supply).
    1. Take apart the Mac Mini - http://www.ifixit.com/Teardown/Mac-Mini-Model-A1347-Teardown/3094/1
    1. Some sort of test cable needs to be made to connect the power supply to the mac mini motherboard.  It uses 12V.  The connectors are:
      * Inside 9 Pin - http://search.digikey.com/scripts/DkSearch/dksus.dll?Detail&name=A99975-ND
      * Crimps - http://search.digikey.com/scripts/DkSearch/dksus.dll?Detail&name=A99967CT-ND
    1. Test to see how long the batteries last with the mac mini running from them.
  1. Get all TXT1s working (5 of them)
    1. Get the steering servos installed on both the front and rear wheels.
    1. Setup the steering servos correctly.
    1. Install the wheel encoders correctly inside the wheel hubs with the correct cable.
  1. Create a power/sensor distribution PCB.  We can discuss details later.
  1. How to mount everything
    1. Maybe replace the GPS/IMU and Camera mounts with a metal mount instead of the 3d print job.

**Extras**

  1. Incorporate FreeRTOS to the microcontroller so we have a more real time system.
  1. Use USB or Ethernet instead of serial on the micro.