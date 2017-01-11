Description

Launchfile in this package should contain modules needed since boot, as services.

In order to install launch file, we use ros package "robot_upstart".
It creates all scripts and adds it to boot process. 

Install script:

You will need root privileges to install scripts. Just exec something similar to this:
	
rosrun robot_upstart install --job rosdaemon  --user roboais --rosdistro indigo --master http://turtlebot:11311 --setup /home/roboais/catkin_ws/devel/setup.bash  --logdir /home/roboais/.ros/  boot_nodes/launch/kompai_boot_nodes.launch

You can manually launch anc check your daemon (named after job parameter):
service rosdaemon status
/usr/sbin/rosdaemon-start (as root)

Uninstall script:

rosrun robot_upstart uninstall rosdaemon

Notes:
Nodes are launched as unprivileged users. If we need privileges (such as access to usb devices or such) we need to grant privileges to those items.
See env_sensor package for an example of an udev rule granting extra privileges. 
