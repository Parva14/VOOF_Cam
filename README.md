# VOOF_Cam

The folder Voof_ros2 contains the Arduino code for the sensor and voof_dyn is the package that defines the reconfigurable paramters. You can change the default values and their ranges from voof.cfg file in the cfg folder.

Using this code you can dynamically reconfigure paramters from ROS and publish a transform from odom frame to base_link frame on the topic /tf

To use it in ROS, you will require ROSSERIAL and dynamic_reconfigure. After getting these packages, follow these steps to use the sensor:

1: Start the rosmaster : 

      $roscore

2: Run the node: 

      $rosrun voof_dyn server.py

   Now you should see the parameters when by using $rosparam list
   
3: This command will open a GUI from which you can change the ros parameters

       $rosrun rqt_gui rqt_gui -s reconfigure
   
4: This command will start the communication between ros and arduino. Make sure your Serial monitor is OFF while you run this                                  ommand and DO NOT FORGET to kill it while uploading your code. In short, Don't use Arduino's serial port while it is connected with ROS

       $rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
    
 5: Now you can adjust the parameters using the GUI what we got in 3rd step. You can see the transform published in /tf by using 
 
         $rosrun rqt_tf_tree rqt_tf_tree
 
 For any queries, feel free to write at parvapatel1996@gmail.com
