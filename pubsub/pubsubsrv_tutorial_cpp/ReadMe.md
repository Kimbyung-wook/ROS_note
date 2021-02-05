# Intro

코드 기반하여 만들었습니다.

기본 구조는 publisher node가 Service Server를 같이 가지고 있는 구조입니다.

그래서 실행할 때, pubserver와 listener를 켜두고 client로 publishing을 켜고 끌 수 있게 되어있습니다.

그래서 메시지 전달 구조를 보면 다음과 같습니다.

client -> pubserver -> listener

Available at 

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29


# How to run

### for testing pub-sub

* rosrun pubsub_tutorial_cpp pubserver
* rosrun pubsub_tutorial_cpp listener
* rosrun pubsub_tutorial_cpp client 1
