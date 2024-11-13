
### 1. Setup ROS Workspace
```
mmabas77@mmabas77-ROS:~/ros$ mkdir ros_ws
mmabas77@mmabas77-ROS:~/ros$ cd ros_ws/
mmabas77@mmabas77-ROS:~/ros/ros_ws$ mkdir src
```
### 2. Build the Workspace
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ catkin_make
mmabas77@mmabas77-ROS:~/ros/ros_ws$ ls
```
### 3. Move into the Source Directory
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ cd src/
mmabas77@mmabas77-ROS:~/ros/ros_ws/src$ ls
```
### 4. Create a ROS Package
```
mmabas77@mmabas77-ROS:~/ros/ros_ws/src$ catkin_create_pkg helloworld_pkg roscpp rospy std_msgs
mmabas77@mmabas77-ROS:~/ros/ros_ws/src$ cd helloworld_pkg/
mmabas77@mmabas77-ROS:~/ros/ros_ws/src/helloworld_pkg$ ls

```
### 5. Create a Scripts Directory (Python - Optional)
```
mmabas77@mmabas77-ROS:~/ros/ros_ws/src/helloworld_pkg$ mkdir scripts
```
### 6. Create a publisher node

### 7. Create a subscriber node

### 8. Build the Package
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ catkin_make
```

### Source the Workspace (Very important)
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ source devel/setup.bash
```

### 9. Run the Publisher Node
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ rosrun helloworld_pkg publisher_node
```
### 10. Run the Subscriber Node
```
mmabas77@mmabas77-ROS:~/ros/ros_ws$ rosrun helloworld_pkg subscriber_node
```


