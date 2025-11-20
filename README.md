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

```
# 驱动真实机械臂 aubo_i5单点轨迹执行demo (修改机器人对应 robot_ip aubo_type)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128  
 use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```
# jsonrpc服务节点驱动真实机械臂 (修改机器人对应 robot_ip)
```bash
source install/setup.bash 
ros2 launch aubo_ros2_driver aubo_client.launch.py
```
## 调用示例
```bash
source install/setup.bash 
ros2 service call /jsonrpc_service aubo_msgs/srv/JsonRpc "{jsonrpc_send: '{\"jsonrpc\":\"2.0\",\"method\":\"rob1.RobotState.getTcpPose\",\"params\":[],\"id\":1}'}"
```
## 响应示例
```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(jsonrpc_send='{"jsonrpc":"2.0","method":"rob1.RobotState.getTcpPose","params":[],"id":1}')

response:
aubo_msgs.srv.JsonRpc_Response(jsonrpc_response='{"id":1,"jsonrpc":"2.0","result":[0.27547126046613063,-0.005044242075764288,0.3927135572919476,3.118375599088596,-0.3941153573472987,-0.6932974021107962]}')
```
## aubo_sdk接口参考文档参考
[aubo_sdk developer](https://docs.aubo-robotics.cn/arcs_api/index.html)