# aubo_ros2_driver

遨博机器人ROS2驱动

# 在 rviz 中 查看 aubo 机器人模型（以 aubo_i5 为例）
```bash
ros2 launch aubo_description aubo_viewer.launch.py
```
# 驱动真实机械臂 aubo_i5 (修改机器人对应 robot_ip aubo_type)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5

```# aubo_ros2_driver

遨博机器人ROS2驱动

# 在 rviz 中 查看 aubo 机器人模型（以 aubo_i5 为例）
```bash
ros2 launch aubo_description aubo_viewer.launch.py
```
# 驱动真实机械臂 aubo_i5 (修改机器人对应 robot_ip aubo_type)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5

```
# 驱动真实机械臂 aubo_i5单点轨迹执行demo (修改机器人对应 robot_ip aubo_type)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```
# 服务节点驱动真实机械臂 (修改机器人对应 robot_ip)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_client.launch.py robot_ip:=127.0.0.1 log_level:=info
```
## 调用示例
```bash
source install/setup.bash 
ros2 service call /jsonrpc_service aubo_msgs/srv/JsonRpc \
"{cls: 'RobotState', func: 'getTcpPose', params: '[]'}"
```
## 响应示例
```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotState', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', error='None')
```
## 异常响应示例(输入错误类'RobotStat')
```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotStat', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='None', error='{"code": -32601, "message": "method not found: rob1.RobotStat.getTcpPose"}')
```
## aubo_sdk接口参考文档
[aubo_sdk developer](https://docs.aubo-robotics.cn/arcs_api/index.html)