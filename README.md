# Heimann-thermopile-ROS-driver

The thermopile tries to obtain an IP address through DHCP. It it cannot get one, it assumes the 192.168.240.122.

To connect the PC to the thermopile you need a crossover Ethernet cable that is provided with it. In the PC you need to create an Ethernet connection with static IP address. The PC could have the address 192.168.240.1, with netmask 255.255.255.0 and gateway 192.168.240.1. 

The thermopile uses the port 30444 por all incoming and outcoming communications.

The process to start working is as follows:

* First an UDP packet has to be sent with the text "Bind HTPA series device". After this, the thermopile is in a command accepting state. The node **initializeHTPA** does this.

* The only command we are going to use is the one needed to receive temperature readings from the device. That command consists of sendind another UDP packet with only "K" as content. The node **startHTPA** is in charge of that action.

* Right now, we only have an UDP connection sending packages from the port 30444 at 192.168.240.122 (thermopile) to the port 30444 at 192.168.240.1 (computer). The node **publishfromHTPA** publishes the UDP packet as a ROS topic called *HTPAoutput* of type *std_msgs::UInt8MultiArray*.

* The node **convertimagefromHTPApublished** subscribes to the *HTPAoutput* topic and publishes a graphic representation in the topic *HTPAimage* of type *sensor_msgs::Image*. This image can be customized by rqt_reconfigure calling to ```rosrun rqt_reconfigure rqt_reconfigure```.

* To view the image, just run ```rosrun image_view image_view image:=\HTPAimage```

All these steps can be executed just launching the ```heiman.launch``` file provided in this package.
