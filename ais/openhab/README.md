## OpenHAB runtime 1.8.3

We are using version 1.8.3 of the [OpenHAB runtime core](https://bintray.com/artifact/download/openhab/bin/distribution-1.8.3-runtime.zip). It requires Java installed on your system. Also it may need other dependencies for bindings (p.e. MySQL or MongoDB).

```
sudo apt-get install default-jre 
sudo apt-get install mongodb-server
```
#Installing openhab

Script ```openhab-install.sh``` should download all required bundles and install openhab as a daemon. 

#Installing manually openhab as a daemon:
Script ```openhab``` is used for this task. In order to install it:

```
sudo cp ./openhab /etc/init.d/openhab
sudo chmod a+x /etc/init.d/openhab
sudo update-rc.d openhab defaults
```

Now whenever your Linux machine boots openHAB will be automatically started.

Note: You may find that the script will not run on startup because it requires a parameter passed at the command line. Use the following to edit the rc.local file.
```
sudo nano /etc/rc.local
```
Then add these lines anywhere above the line 'exit 0'
```
# start openhab
/etc/init.d/openhab start
```

###OPTIONAL: Installing zstick usb gateway in a fixed link

By default, ubuntu will create a symlink at ```/dev/ttyACMxx```. We will use an udev rule to have it pointing always to the same file ```/dev/zwave```. 
Create a file named ```/etc/udev/rules.d/99-usb-zwave.rules``` containing this line:
```
KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0658", ATTRS{idProduct}=="0200", SYMLINK+="zwave"
```

Now plug the usb and it will be available at  ```/dev/zwave```. If it does not work, force a reload with
```
udevadm control --reload
```

To make openhab use this, you should change also:
- ```configurations/openhab.cfg``` port value should point to ```/dev/zwave```
- start script should include this line in java call  ```-Dgnu.io.rxtx.SerialPorts=/dev/zwave \```


See also:
[OpenHAB wiki](https://github.com/openhab/openhab/wiki)
