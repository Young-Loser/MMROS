#!/usr/bin/env python3
# rosrun object_search door_explorer_node.py


# 我需要让机器人执行一个任务，他现在在一个走廊中，需要完成的任务是去每一个门口看一眼，
# 并记录下来在每个门口看到了什么，但是他不知道每个门口的具体位置，需要自行探索，我能告诉他的所有信息就是走廊的起点与终点坐标，
# 他需要最终走到终点，并告诉我一共路过了几个门，在每个门口都看到了什么。 
        # self.start_point = tuple(rospy.get_param("~start_point", [-14, 1.4]))
        # self.end_point = tuple(rospy.get_param("~end_point", [14.0, 1.4]))

import rospy
import json
import time
from dmce_msgs.srv import GetPlan, GetPlanRequest
from dmce_msgs.msg import RobotPosition
from dmce_msgs.msg import NavigationFailureSignal
import rospy
from std_msgs.msg import String

class DoorExplorerNode:
    def __init__(self):
        rospy.init_node("door_explorer_node", anonymous=True)
        rospy.loginfo("🚪 启动 DoorExplorerNode：根据检测到的门坐标进行探索")

        # 参数
        self.robot_pos = (0.0, 0.0)
        self.detected_doors = {}   # {door_name: (x, y)}
        self.visited_doors = set()
        self.exploration_log = {}  # {door_name: [objects]}
        self.current_detected_objects = []  # 当前检测到的物体列表

        # ✅ 起点与终点坐标（走廊范围）
        self.start_point = tuple(rospy.get_param("~start_point", [-14.0, 1.7]))
        self.end_point = tuple(rospy.get_param("~end_point", [14.0, 1.7]))
        self.end_reach_threshold = rospy.get_param("~end_reach_threshold", 0.5)

        # 订阅机器人位置与检测结果
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        rospy.Subscriber("/object_search/detected_objects", String, self.objects_callback)

        # 连接全局规划服务
        rospy.wait_for_service("/robot1/GlobalPlannerService")
        self.plan_service = rospy.ServiceProxy("/robot1/GlobalPlannerService", GetPlan)
        rospy.loginfo("✅ 已连接 GlobalPlannerService")

        self.run()

    # ---------------- 回调 ----------------
    def pos_callback(self, msg):
        self.robot_pos = (msg.x_position, msg.y_position)
        print(f"[位置] x = {msg.x_position:6.2f} | y = {msg.y_position:6.2f}", end="\r")

    def objects_callback(self, msg):
        """解析 object_detector_node 发布的物体检测结果"""
        try:
            data = json.loads(msg.data)
            self.current_detected_objects = data.get("objects", [])

            for obj in self.current_detected_objects:
                name = obj.get("name", "")
                category = obj.get("category", "")
                x, y = obj.get("x", 0.0), obj.get("y", 0.0)

                # ✅ 判断是否为门口对象
                if "door" in name or category == "door":
                    if name not in self.detected_doors:
                        self.detected_doors[name] = (x, y)
                        rospy.loginfo(f"🚪 新发现门：{name} (x={x:.2f}, y={y:.2f})")

        except Exception as e:
            rospy.logwarn(f"⚠️ 无法解析检测结果: {e}")
        
    def clear_target(self):
        """彻底移除 /objectsearch/next_target 参数"""
        try:
            if rospy.has_param("/objectsearch/next_target"):
                rospy.delete_param("/objectsearch/next_target")
                rospy.loginfo("🧹 已删除参数 /objectsearch/next_target")
        except Exception as e:
            rospy.logwarn(f"⚠️ 删除参数失败: {e}")

    # ---------------- 移动函数 ----------------
    def go_to(self, target):
        """调用 GlobalPlannerService 让机器人移动到目标点"""
        try:
            cur_x, cur_y = self.robot_pos
            req = GetPlanRequest()
            req.success = True
            req.currentPosition = RobotPosition(robotId=1, x_position=cur_x, y_position=cur_y)

            rospy.set_param("/objectsearch/next_target", {"x": target[0], "y": target[1]})
            self.plan_service(req)
            # print(rospy.get_param("/objectsearch/next_target"))
            rospy.loginfo(f"🎯 正在前往目标点 ({target[0]:.2f}, {target[1]:.2f})")
        except Exception as e:
            rospy.logerr(f"❌ 调用 GlobalPlannerService 失败: {e}")
    
    def get_next_door(self):
        """返回离当前位置最近的、尚未访问的门"""
        candidates = {n: p for n, p in self.detected_doors.items() if n not in self.visited_doors}
        if not candidates:
            return None, None

        cur_x, cur_y = self.robot_pos
        name, pos = min(
            candidates.items(),
            key=lambda kv: (kv[1][0] - cur_x)**2 + (kv[1][1] - cur_y)**2
        )
        return name, pos

    # ---------------- 主逻辑 ----------------
    def run(self):
        rospy.loginfo("🕵️ 开始走廊探索任务...")
        state = "GO_TO_END"
        rate = rospy.Rate(1)

        self.go_to_segmented(self.end_point)
        current_target = ("end", self.end_point)

        while not rospy.is_shutdown():
            # ✅ 检查是否到达终点
            dist_to_end = ((self.robot_pos[0] - self.end_point[0])**2 +
                        (self.robot_pos[1] - self.end_point[1])**2)**0.5
            if dist_to_end < self.end_reach_threshold and state == "GO_TO_END":
                self.clear_target()
                rospy.loginfo("🏁 已到达走廊终点，探索任务结束！")
                break

            # === 状态：前往终点 ===
            if state == "GO_TO_END":
                next_name, next_pos = self.get_next_door()
                if next_name is not None:
                    rospy.loginfo(f"🚪 检测到新门 [{next_name}]，前往探索。")
                    self.go_to(next_pos)
                    current_target = (next_name, next_pos)
                    state = "VISIT_DOOR"
                else:
                    rospy.loginfo("➡️ 没有检测到新门，继续前往终点。")
                    self.go_to(self.end_point)
                    current_target = ("end", self.end_point)

            # === 状态：前往门 ===
            elif state == "VISIT_DOOR":
                name, pos = current_target
                dist = ((self.robot_pos[0] - pos[0])**2 + (self.robot_pos[1] - pos[1])**2)**0.5
                if dist < 0.1:
                    rospy.loginfo(f"⏸️ 已到达门 [{name}]，观察环境...")
                    self.clear_target()
                    state = "OBSERVE_DOOR"

            # === 状态：观察门 ===
            elif state == "OBSERVE_DOOR":
                name, pos = current_target
                seen = set()
                start = time.time()

                while time.time() - start < 3.0 and not rospy.is_shutdown():
                    for obj in self.current_detected_objects:
                        n = obj.get("name", "")
                        if "door" not in n:
                            seen.add(n)
                    rospy.sleep(0.5)

                self.visited_doors.add(name)
                self.exploration_log[name] = sorted(seen)
                rospy.loginfo(f"📋 门 [{name}] 观察完毕: {', '.join(seen) if seen else '无'}")

                # 判断是否还有别的门
                next_name, next_pos = self.get_next_door()
                if next_name is not None:
                    rospy.loginfo(f"🧭 还有未访问的门 [{next_name}]，继续前往。")
                    self.go_to(next_pos)
                    current_target = (next_name, next_pos)
                    state = "VISIT_DOOR"
                else:
                    # rospy.loginfo("➡️ 没有更多门，继续前往终点。")
                    self.go_to(self.end_point)
                    current_target = ("end", self.end_point)
                    state = "GO_TO_END"

            rate.sleep()

        # ✅ 输出探索总结
        rospy.loginfo("🎯 所有门探索完成！")
        print("\n====== 探索总结 ======")
        for door, objs in self.exploration_log.items():
            print(f"{door}: {', '.join(objs) if objs else '无'}")

if __name__ == "__main__":
    try:
        DoorExplorerNode()
    except rospy.ROSInterruptException:
        pass
