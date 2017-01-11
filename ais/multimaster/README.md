## launch file to provide multimaster 

Launch file template to use multimaster-fkie package. Multimaster node (launch file) must be running at each master. Before being used at each master, we have to:

- Set env var ROS_MASTER_URI (to its own ip and port like 'http://10.18.83.181:11311' NOT LOOPBACK)
- Set BOTH machine names at BOTH /etc/hosts (p.e. we have turtlebot and ami. Hosts files should contain this lines: '10.18.8.218 turtlebot' and '10.18.83.181 ami')
- Multicast should be enabled on used interface (check addres with 'netstat -g'). Take note of interface's multicast ip, because it must be included into launch files.

Notes:
		Control GUI: rosrun node_manager_fkie node_manager
ToDo:
		Select sync topics: should be something like this:
```
			<param name="sync_topics" type="str" value="[/robotAmbientData,/robotPosition]" />
```
See:
[Multimaster FKIE Manual](http://www.iri.upc.edu/files/scidoc/1607-Multi-master-ROS-systems.pdf)
