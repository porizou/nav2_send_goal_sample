# アクションメッセージ型


```text:NavigateToPose.action

#goal definition
geometry_msgs/PoseStamped pose
string behavior_tree
---
#result definition
std_msgs/Empty result
---
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
int16 number_of_recoveries
float32 distance_remaining
```

### ROS Navigationのメッセージ型

```MoveBase.action
geometry_msgs/PoseStamped target_pose
---
---
geometry_msgs/PoseStamped base_position

```


# 参考

https://daily-tech.hatenablog.com/entry/2017/02/11/214336


https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html


https://github.com/fmrico/nav2_test_utils



