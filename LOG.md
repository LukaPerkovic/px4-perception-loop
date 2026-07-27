# Weekly Progress Log

Notes about done work, missed plans, and next targets structured by weeks. Interval for preceeding week marked on each Sunday

----

## 02.08.2026.

### What shipped
Shipped full heartbeat node. It publishes a heartbeat: counter at 1 sec intervals, and subscribes to PX4 Vehicle position. There's quite a few things going on but in short:
1. pkg create (or copy from ros, create command is better)
2. #includes (rclcpp + message types. Some messages need to be downloaded i.e. px4_msgs)
3. class : public rclcpp : Noe
4. constructor: create qos / sub / pub /timer
5. private: callbacks + member handles +  state vars
6. main -> init -> spin -> shutdown
7. CMakeLists.txt: add_executable + ament_target_dependencies + install
8. colcon build from ROOT -> source install -> ros2 run 

### What slipped
- Took too long to initiate this, and struggled with understanding what agent is, how to make it work, how to integrate with ros and how to write a node. 2.5 weeks went intothis
- I manually set ROS_DOMAIN_ID to some number. Even though I made sure that each termnal uses the same ID, the solution didn't really work until I removed it and set to 0. Much time was lost debugging this.
- Documentation says vehicle_local_position but when you list topics it has suffix v1. Trust what facts say sometimes. This was the correct name.

### Next steps
Time to embark on the perception node. WIll be fun.
