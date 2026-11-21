# px4_waypoint_sitl

PX4 + MAVROS + ROS1 + Gazebo Classic SITL 四点航点仿真：无人机解锁进入 OFFBOARD 后，按正方形四点 (1,0) -> (1,1) -> (0,1) -> (0,0) 依次飞行。

## 编译

```bash
cd ~/catkin_ws
catkin_make --pkg px4_waypoint_sitl
source ~/catkin_ws/devel/setup.bash
```

## 运行（四个 terminal）

| Terminal | 命令 | 说明 |
|----------|------|------|
| 1 | `cd ~/cwkj_PX4/PX4-Autopilot && make px4_sitl gazebo` | 启动 PX4 SITL + Gazebo |
| 2 | `roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557` | 启动 MAVROS 连接 |
| 3 | `source ~/catkin_ws/devel/setup.bash && rosrun px4_waypoint_sitl four_points_sitl` | 运行航点节点（自动切 OFFBOARD 并解锁） |
| 4 | `rosrun mavros mavsys mode -c OFFBOARD && rosrun mavros mavsafety arm` | 可选：手动切 OFFBOARD 模式并解锁 |

> 注意：节点现在会先持续发布第一个航点，再自动请求 OFFBOARD 模式并解锁，无需手动操作。Terminal 4 仅作备用手动方式。

## 遇到的问题与代码改动

### 问题 1：`rosrun` 找不到可执行文件

```
[rosrun] Couldn't find executable named four_points_sitl below /home/ian/catkin_ws/src/px4_waypoint_sitl
```

**原因**：`CMakeLists.txt` 中没有声明 `add_executable`，源码 `src/four_points.cpp` 从未被编译。

**修复**：在 `CMakeLists.txt` 中增加：

```cmake
add_executable(four_points_sitl src/four_points.cpp)
target_link_libraries(four_points_sitl
  ${catkin_LIBRARIES}
)
```

### 问题 2：编译报错 `pffset`

```cpp
p.pose.position.y += pffset[1];   // 拼写错误
```

**修复**：`pffset` -> `offset`。

### 问题 3：`break statement not within loop or switch`

原代码中航点跟随逻辑写在了主 `while` 循环**之外**，导致 `break` 不在循环内，且航点只执行一次。

**修复**：把航点发布、到达判定、切换航点、超时检测全部移入主 `while` 循环内，形成正确的航点跟随逻辑：

```cpp
while (ros::ok()){
    ros::spinOnce();

    if (!state.connected || !pose_received) break;
    if (!state.armed || state.mode != "OFFBOARD"){
        ROS_FATAL("Offboard failed");
        break;
    }

    auto target = points[index];
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    pub.publish(target);

    if (distanceTo(target) < 0.3){
        if (reached_since.isZero()) reached_since = ros::Time::now();
        if ((ros::Time::now() - reached_since).toSec() > 2.0){
            if (index + 1 < points.size()){
                ++index;
                reached_since = ros::Time();
                waypoint_start = ros::Time::now();
                ROS_INFO("Waypoint %zu", index + 1);
            } else {
                ROS_INFO("All waypoints reached.");
                break;
            }
        }
    } else {
        reached_since = ros::Time();
    }

    if ((ros::Time::now() - waypoint_start).toSec() > 30.0){
        ROS_ERROR("Waypoint timeout.");
        break;
    }
    rate.sleep();
}
```

### 问题 4：运行时报 `[FATAL] Offboard failed`

```
[FATAL] [...] : Offboard failed
```

**原因**：节点在无人机尚未解锁 / 未切 OFFBOARD 时直接 `break`。而 PX4 只有在持续收到 OFFBOARD 设定点后才会接受 OFFBOARD 模式，形成「先有鸡还是先有蛋」的循环。

**修复**：改为标准 MAVROS offboard 流程：

1. 先以 20 Hz 持续发布第一个航点（约 5 秒，100 次）；
2. 通过 `/mavros/set_mode` 服务请求 `OFFBOARD`；
3. 切成功后通过 `/mavros/cmd/arming` 服务请求解锁；
4. 解锁 + OFFBOARD 后开始跟随航点。

新增头文件 `mavros_msgs/SetMode.h`、`mavros_msgs/CommandBool.h`，并在 `main` 中加入两个 service client。

## TODO

- [ ] bringup
- [ ] 自动takeoff和land
- [ ] 待实测调试航点到达判定阈值 (0.3 m) 与保持时间 (2.0 s)
- [ ] 待实测调试航点超时时间 (30 s)
