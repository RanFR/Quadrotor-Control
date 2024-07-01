# Quadrotor-Control

用于提高mavros中的IMU发布消息频率（到100Hz），以及四旋翼无人机切换OFFBOARD模式与解锁。

## 使用方法

clone到本地。

```bash
git clone https://github.com/RanFR/Quadrotor-Control.git
```

`catkin`编译，需要安装python3-catkin-tools功能包。

```bash
catkin b
```

`roslaunch`运行

```bash
roslaunch quadrotor_control quadrotor_control.launch
```
