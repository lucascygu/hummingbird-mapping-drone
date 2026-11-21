#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

# ======== 模型与场地参数配置 ========
UAV_MODEL_NAME = "iris"          # 替换为你Gazebo中无人机的实际model名称
UAV_LINK_NAME  = "base_link"     # 无人机主体link名称
DUCK_MODEL_NAME = "microduck"
DUCK_LINK_NAME  = "base_link"

TABLE_DROP_X = 1.75              # 桌面安全降落区X (桌子中心2.0，留出走到2.3餐巾纸的距离)
TABLE_DROP_Y = 0.0
TABLE_SURFACE_Z = 0.75           # 桌面高度
DUCK_HEIGHT = 0.24               # MicroDuck高度 + 无人机底部起落架偏置
CRUISE_ALT = 1.60                # 室内安全巡航高度 (高于0.75m桌面)
NAPKIN_X = 2.25                  # 餐巾纸拾取位置X
GROUND_HOME_X = 0.8              # 返回地面的投放位置X
GROUND_HOME_Y = -0.6

class AirGroundCoopFSM:
    def __init__(self):
        rospy.init_node("air_ground_coop_fsm")

        self.uav_state = State()
        self.uav_pose = PoseStamped()
        self.duck_odom = Odometry()

        # ROS 订阅与发布
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.uav_pose_cb)
        rospy.Subscriber("/microduck/odom", Odometry, self.duck_odom_cb)

        self.uav_sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.duck_vel_pub = rospy.Publisher("/microduck/cmd_vel", Twist, queue_size=10)

        # 服务客户端：解锁、切模式、电磁铁吸合/释放
        rospy.loginfo("等待 MAVROS 与 Gazebo 磁吸插件服务...")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/link_attacher_node/attach")
        rospy.wait_for_service("/link_attacher_node/detach")

        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    def state_cb(self, msg): self.uav_state = msg
    def uav_pose_cb(self, msg): self.uav_pose = msg
    def duck_odom_cb(self, msg): self.duck_odom = msg

    def set_magnet(self, enable=True, target_model=DUCK_MODEL_NAME, target_link=DUCK_LINK_NAME,
                   parent_model=UAV_MODEL_NAME, parent_link=UAV_LINK_NAME):
        """控制电磁铁（或夹爪）吸合与释放"""
        req = AttachRequest()
        req.model_name_1 = parent_model
        req.link_name_1 = parent_link
        req.model_name_2 = target_model
        req.link_name_2 = target_link
        if enable:
            self.attach_srv.call(req)
            rospy.loginfo(f"[磁吸/夹爪] 已锁死: {parent_model} <-> {target_model}")
        else:
            self.detach_srv.call(req)
            rospy.loginfo(f"[磁吸/夹爪] 已释放: {parent_model} <-> {target_model}")

    def uav_reached(self, tx, ty, tz, tol=0.10):
        dx = self.uav_pose.pose.position.x - tx
        dy = self.uav_pose.pose.position.y - ty
        dz = self.uav_pose.pose.position.z - tz
        return math.sqrt(dx*dx + dy*dy + dz*dz) < tol

    def drive_duck_to_x(self, target_x, speed=0.15):
        """控制 MicroDuck 沿 X 轴行走至指定位置"""
        cmd = Twist()
        err = target_x - self.duck_odom.pose.pose.position.x
        if abs(err) < 0.04:
            self.duck_vel_pub.publish(Twist()) # 停止
            return True
        cmd.linear.x = speed if err > 0 else -speed
        self.duck_vel_pub.publish(cmd)
        return False

    def run(self):
        rate = rospy.Rate(20)
        sp = PoseStamped()
        sp.pose.orientation.w = 1.0

        # PX4 Offboard 预热：发送100个初始设定点
        for _ in range(100):
            self.uav_sp_pub.publish(sp)
            rate.sleep()

        phase = "DUCK_APPROACH_TABLE"
        rospy.loginfo("=== 地空协同任务正式启动 ===")
        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            # 维持无人机 Offboard 与解锁状态 (当任务进入飞行阶段后)
            if phase not in ["DUCK_APPROACH_TABLE", "MISSION_COMPLETE"]:
                if self.uav_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(2.0)):
                    self.mode_srv(custom_mode="OFFBOARD")
                    last_req = rospy.Time.now()
                elif not self.uav_state.armed and (rospy.Time.now() - last_req > rospy.Duration(2.0)):
                    self.arm_srv(True)
                    last_req = rospy.Time.now()

            duck_x = self.duck_odom.pose.pose.position.x
            duck_y = self.duck_odom.pose.pose.position.y

            # 阶段 1：MicroDuck 走向桌子，遇到高差障碍呼叫无人机
            if phase == "DUCK_APPROACH_TABLE":
                if self.drive_duck_to_x(1.25):
                    rospy.loginfo("[MicroDuck] 检测到前方0.75m桌面高差！停止前进并呼叫无人机吊运！")
                    sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = 0.0, 0.0, CRUISE_ALT
                    phase = "UAV_TAKEOFF_AND_FETCH"

            # 阶段 2：无人机起飞并飞往 MicroDuck 正上方
            elif phase == "UAV_TAKEOFF_AND_FETCH":
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = duck_x, duck_y, CRUISE_ALT
                if self.uav_reached(duck_x, duck_y, CRUISE_ALT):
                    rospy.loginfo("[UAV] 到达 MicroDuck 正上方，开始下降对接...")
                    phase = "UAV_DESCEND_PICKUP_1"

            # 阶段 3：无人机下降贴合 MicroDuck 头顶，启动电磁铁
            elif phase == "UAV_DESCEND_PICKUP_1":
                pickup_z = DUCK_HEIGHT + 0.05
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = duck_x, duck_y, pickup_z
                if self.uav_reached(duck_x, duck_y, pickup_z, tol=0.08):
                    self.set_magnet(True) # 电磁铁通电吸合
                    rospy.sleep(0.5)
                    phase = "UAV_TRANSPORT_TO_TABLE"

            # 阶段 4：吊运升空并飞往桌面上方，再精准降落至桌面高度
            elif phase == "UAV_TRANSPORT_TO_TABLE":
                drop_z = TABLE_SURFACE_Z + DUCK_HEIGHT + 0.06
                # 先保持高度平移到桌面上方，到达XY后再下降
                xy_dist = math.hypot(self.uav_pose.pose.position.x - TABLE_DROP_X,
                                     self.uav_pose.pose.position.y - TABLE_DROP_Y)
                sp.pose.position.x = TABLE_DROP_X
                sp.pose.position.y = TABLE_DROP_Y
                sp.pose.position.z = CRUISE_ALT if xy_dist > 0.15 else drop_z

                if self.uav_reached(TABLE_DROP_X, TABLE_DROP_Y, drop_z, tol=0.08):
                    self.set_magnet(False) # 断电释放 MicroDuck 到桌面
                    rospy.loginfo("[UAV] MicroDuck 已投放至桌面！无人机升空悬停待命。")
                    phase = "DUCK_PICK_NAPKIN"

            # 阶段 5：无人机退避悬停，MicroDuck 在桌面上前进拾取餐巾纸
            elif phase == "DUCK_PICK_NAPKIN":
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = TABLE_DROP_X - 0.5, 0.0, CRUISE_ALT
                if self.drive_duck_to_x(NAPKIN_X, speed=0.10):
                    rospy.loginfo("[MicroDuck] 到达餐巾纸位置，执行夹取！")
                    # 复用插件模拟夹爪抓起餐巾纸
                    self.set_magnet(True, target_model="napkin", target_link="napkin_link",
                                    parent_model=DUCK_MODEL_NAME, parent_link=DUCK_LINK_NAME)
                    rospy.sleep(0.5)
                    phase = "DUCK_WAIT_EXTRACTION"

            # 阶段 6：MicroDuck 退回桌面开阔区，呼叫无人机接回地面
            elif phase == "DUCK_WAIT_EXTRACTION":
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = TABLE_DROP_X - 0.5, 0.0, CRUISE_ALT
                if self.drive_duck_to_x(TABLE_DROP_X, speed=0.10):
                    rospy.loginfo("[MicroDuck] 餐巾纸已拾取完毕，呼叫无人机接回地面！")
                    phase = "UAV_PICKUP_FROM_TABLE"

            # 阶段 7：无人机从桌面再次吸起携带着餐巾纸的 MicroDuck
            elif phase == "UAV_PICKUP_FROM_TABLE":
                pickup_z = TABLE_SURFACE_Z + DUCK_HEIGHT + 0.05
                xy_dist = math.hypot(self.uav_pose.pose.position.x - duck_x,
                                     self.uav_pose.pose.position.y - duck_y)
                sp.pose.position.x = duck_x
                sp.pose.position.y = duck_y
                sp.pose.position.z = CRUISE_ALT if xy_dist > 0.12 else pickup_z

                if self.uav_reached(duck_x, duck_y, pickup_z, tol=0.08):
                    self.set_magnet(True)
                    rospy.sleep(0.5)
                    phase = "UAV_RETURN_TO_GROUND"

            # 阶段 8：运送回地面目标点并释放
            elif phase == "UAV_RETURN_TO_GROUND":
                drop_z = DUCK_HEIGHT + 0.06
                xy_dist = math.hypot(self.uav_pose.pose.position.x - GROUND_HOME_X,
                                     self.uav_pose.pose.position.y - GROUND_HOME_Y)
                sp.pose.position.x = GROUND_HOME_X
                sp.pose.position.y = GROUND_HOME_Y
                sp.pose.position.z = CRUISE_ALT if xy_dist > 0.15 else drop_z

                if self.uav_reached(GROUND_HOME_X, GROUND_HOME_Y, drop_z, tol=0.08):
                    self.set_magnet(False)
                    rospy.loginfo("[UAV] MicroDuck 与餐巾纸已安全返回地面！无人机返航降落。")
                    phase = "UAV_RETURN_HOME"

            # 阶段 9：无人机返回起飞点降落
            elif phase == "UAV_RETURN_HOME":
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = 0.0, 0.0, CRUISE_ALT
                if self.uav_reached(0.0, 0.0, CRUISE_ALT, tol=0.15):
                    self.mode_srv(custom_mode="AUTO.LAND")
                    rospy.loginfo("=== 全流程地空协同任务圆满完成！ ===")
                    phase = "MISSION_COMPLETE"

            if phase != "MISSION_COMPLETE":
                self.uav_sp_pub.publish(sp)
            rate.sleep()

if __name__ == "__main__":
    fsm = AirGroundCoopFSM()
    fsm.run()
