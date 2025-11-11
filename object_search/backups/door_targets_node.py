#!/usr/bin/env python3
# rosrun object_search door_targets_node.py

import rospy
import yaml
import math
from dmce_msgs.srv import GetPlan, GetPlanRequest
from dmce_msgs.msg import RobotPosition
from geometry_msgs.msg import PoseStamped
import time
import os

class DoorTargetsNode:
    def __init__(self):
        rospy.init_node("door_targets_node")

        # ✅ 读取 YAML 文件（例如 apartment1_all.yaml）
        yaml_file = os.path.join(os.path.dirname(__file__), "..", "apartment1_all.yaml")
        yaml_file = os.path.abspath(yaml_file)
        if not os.path.exists(yaml_file):
            rospy.logerr(f"❌ 未找到 {yaml_file}")
            return

        with open(yaml_file, "r") as f:
            data = yaml.safe_load(f)

        # ✅ 提取门的 ROS 坐标
        self.targets = []
        for door in data.get("doors", []):
            center = door["ros"]["center"]
            self.targets.append((center["x"], center["y"]))

        if not self.targets:
            rospy.logerr("❌ YAML 文件中未找到 doors 信息！")
            return

        rospy.loginfo("✅ 读取到 %d 个门口坐标", len(self.targets))
        for i, (x, y) in enumerate(self.targets):
            rospy.loginfo(f"  门{i+1}: x={x:.2f}, y={y:.2f}")

        # ✅ 订阅机器人当前位置
        self.robot_pos = None
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)

        # ✅ 等待 GlobalPlannerService
        rospy.loginfo("等待 GlobalPlannerService 可用中...")
        rospy.wait_for_service("/robot1/GlobalPlannerService")
        self.plan_service = rospy.ServiceProxy("/robot1/GlobalPlannerService", GetPlan)
        rospy.loginfo("✅ GlobalPlannerService 已连接！")

    def pos_callback(self, msg):
        """订阅机器人当前位置"""
        self.robot_pos = (msg.x_position, msg.y_position)

    def wait_for_position(self):
        """等待首次位置更新"""
        while not rospy.is_shutdown() and self.robot_pos is None:
            rospy.loginfo_throttle(2.0, "等待 RobotPosition 更新...")
            rospy.sleep(0.1)

    def distance_to_target(self, target):
        """计算当前位置到目标点的距离"""
        if self.robot_pos is None:
            return float('inf')
        x, y = self.robot_pos
        tx, ty = target
        return math.sqrt((x - tx)**2 + (y - ty)**2)
    
    def wait_until_reached(self, target, threshold=0.5, timeout=60):
        """等待机器人到达目标点"""
        rospy.loginfo(f"⏳ 等待到达目标 (%.2f, %.2f) ..." % target)
        start_time = time.time()
        while not rospy.is_shutdown():
            dist = self.distance_to_target(target)
            if dist < threshold:
                rospy.loginfo(f"🎯 已到达目标点！距离={dist:.2f} m")
                return True
            if time.time() - start_time > timeout:
                rospy.logwarn(f"⚠️ 等待超时（>{timeout}s ，继续下一个目标")
                return False
            rospy.sleep(0.2)
        return False    

    def go_to_targets(self):
        """依次发送目标点"""
        self.wait_for_position()

        # ✅ 反转目标顺序
        self.targets = list(reversed(self.targets))        

        for i, (x, y) in enumerate(self.targets):
            if rospy.is_shutdown():
                break

            rospy.loginfo(f"🚩 发送目标 {i+1}: (x={x:.2f}, y={y:.2f})")

            # 当前机器人位置
            cur_x, cur_y = self.robot_pos

            req = GetPlanRequest()
            req.success = True
            req.currentPosition = RobotPosition(
                robotId=1,
                x_position=cur_x,
                y_position=cur_y
            )

            # 把目标点传给 objectsearch
            rospy.set_param("/objectsearch/next_target", {"x": x, "y": y})

            try:
                resp = self.plan_service(req)
                if len(resp.plan.poses) > 0:
                    rospy.loginfo(f"✅ 规划成功，共 {len(resp.plan.poses)} 个点")
                else:
                    rospy.logwarn("⚠️ 路径为空，可能目标在障碍物内！")
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {e}")
                continue

            # ✅ 等待真正到达目标（取代固定 sleep）
            self.wait_until_reached((x, y), threshold=0.4, timeout=90)

        rospy.loginfo("🎯 所有门口目标已完成。")


if __name__ == "__main__":
    try:
        node = DoorTargetsNode()
        node.go_to_targets()
    except rospy.ROSInterruptException:
        pass
