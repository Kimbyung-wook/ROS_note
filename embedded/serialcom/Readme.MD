# The tutorial of Serial communication with termios.h

This repo has two nodes.
One node does only send messages, and the other does only receives messages from the sender node.

Both node print healty messages each second.

## How to run or test ?

1. Basic code

* roscore
* rosrun serialcom serialreceive
* rosrun serialcom serialsend

2. Use argments

* roscore
* rosrun serialcom serialreceive_arg /dev/ttyUSB0
* rosrun serialcom serialsend_arg /dev/ttyUSB0

3. run via roslaunch with argments

* roslaunch serialcom receiver_arg.launch /dev/ttyUSB0
* rosrun serialcom serialsend_arg /dev/ttyUSB1

4. Use instances for connecting serial ports

* preparing...

Terminal 1 : Sender
> [ INFO] Loop start 
>
> [ INFO] Healty
>
> [ INFO] Send : Time to send 0th
>
> [ INFO] Healty
>
> [ INFO] Send : Time to send 1th
>
> [ INFO] Healty
>
> [ INFO] Send : Time to send 2th
>
> [ INFO] Healty
>
> [ INFO] Send : Time to send 3th

Terminal 2 : Receiver
> [ INFO] Loop start
>
> [ INFO] Healty
>
> [ INFO] Receive 17 : Time to send 0th
>
> [ INFO] Healty
>
> [ INFO] Receive 17 : Time to send 1th
>
> [ INFO] Healty
>
> [ INFO] Receive 17 : Time to send 2th
>
> [ INFO] Healty
>
> [ INFO] Receive 17 : Time to send 3th




This repo is refered from ...
* **Linux C++ serial programming ROS**, Available at https://www.programmersought.com/article/551142478/
* Ahn, J. H., **ros-termios-read**, Availble at https://github.com/jeeho-ahn/ros-termios-read



