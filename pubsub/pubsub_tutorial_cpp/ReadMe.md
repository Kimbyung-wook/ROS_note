# Intro

This code is refered from the docu of ROS tutorials

Available at 

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29


# How to run

### for testing pub-sub

* rosrun pubsub_tutorial_cpp talker
* rosrun pubsub_tutorial_cpp listener

or

* roslaunch pubsub_tutorial_cpp pubsub.launch


### for testing service-client

* rosrun pubsub_tutorial_cpp server
* rosrun pubsub_tutorial_cpp client
