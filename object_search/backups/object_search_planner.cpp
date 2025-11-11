#include "dmce_core/object_search_planner.hpp"
#include "dmce_core/TypeDefs.hpp"
#include <cmath>

namespace dmce {

ObjectSearchPlanner::ObjectSearchPlanner(double robotDiameter)
    : Planner(robotDiameter), current_idx_(0), goal_tolerance_(0.5) {
    targets_.clear();
}

void ObjectSearchPlanner::setTargets(const std::vector<geometry_msgs::PoseStamped>& newTargets) {
    targets_ = newTargets;
    current_idx_ = 0;
}

bool ObjectSearchPlanner::reachedTarget_(const pos_t& current,
                                         const geometry_msgs::PoseStamped& goal) {
    double dx = current.x() - goal.pose.position.x;
    double dy = current.y() - goal.pose.position.y;
    return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

// void ObjectSearchPlanner::updatePlan_() {
//     // 从 ROS param 获取 Python 传入的目标点
//     double gx, gy;
//     if (ros::param::get("/objectsearch/next_target/x", gx) &&
//         ros::param::get("/objectsearch/next_target/y", gy)) {
//         targets_.clear();
//         geometry_msgs::PoseStamped goal;
//         goal.pose.position.x = gx;
//         goal.pose.position.y = gy;
//         targets_.push_back(goal);
//         ROS_INFO("[ObjectSearchPlanner] 收到新目标点: (%.2f, %.2f)", gx, gy);
//         ros::param::del("/objectsearch/next_target"); // 删除已读参数
//     }

//     if (targets_.empty() || current_idx_ >= targets_.size())
//         return;
// }

// std::pair<bool, plan_t> ObjectSearchPlanner::getLatestPlan_() {
//     if (targets_.empty() || current_idx_ >= targets_.size())
//         return {false, {}};

//     plan_t plan;
//     geometry_msgs::PoseStamped start;
//     pos_t current = getPosition();

//     start.header.frame_id = "map";
//     start.pose.position.x = current.x();
//     start.pose.position.y = current.y();
//     start.pose.orientation.w = 1.0;
//     plan.push_back(start);

//     geometry_msgs::PoseStamped goal = targets_[current_idx_];
//     goal.pose.orientation.w = 1.0;
//     plan.push_back(goal);

//     latest_plan_ = plan;
//     return {true, latest_plan_};
// }

void ObjectSearchPlanner::updatePlan_() {
    // 每次更新检查最新的目标点参数
    XmlRpc::XmlRpcValue target_param;
    if (ros::param::get("/objectsearch/next_target", target_param)) {
        double target_x = static_cast<double>(target_param["x"]);
        double target_y = static_cast<double>(target_param["y"]);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = target_x;
        goal.pose.position.y = target_y;
        goal.pose.orientation.w = 1.0;

        latest_plan_.clear();
        latest_plan_.push_back(goal);

        ROS_INFO_THROTTLE(2.0, "[ObjectSearchPlanner] 🔁 更新目标 (%.2f, %.2f)", target_x, target_y);
    }
}

// std::pair<bool, plan_t> ObjectSearchPlanner::getLatestPlan_() {
//     return { !latest_plan_.empty(), latest_plan_ };
// }

std::pair<bool, plan_t> ObjectSearchPlanner::getLatestPlan_() {
    // ✅ 1. 检查 /objectsearch/next_target 参数是否更新
    XmlRpc::XmlRpcValue target_param;
    if (ros::param::get("/objectsearch/next_target", target_param)) {
        double target_x = 0.0, target_y = 0.0;

        // 防御式检查：确保参数类型正确
        if (target_param.hasMember("x") && target_param.hasMember("y")) {
            try {
                target_x = static_cast<double>(target_param["x"]);
                target_y = static_cast<double>(target_param["y"]);
            } catch (...) {
                ROS_WARN("[ObjectSearchPlanner] ⚠️ 无法解析 /objectsearch/next_target 参数类型");
            }

            // ✅ 如果目标点和当前不同，则更新
            if (targets_.empty() ||
                std::fabs(target_x - targets_.back().pose.position.x) > 1e-3 ||
                std::fabs(target_y - targets_.back().pose.position.y) > 1e-3) {

                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.pose.position.x = target_x;
                goal.pose.position.y = target_y;
                goal.pose.orientation.w = 1.0;

                targets_.clear();
                targets_.push_back(goal);
                current_idx_ = 0;

                printf("[ObjectSearchPlanner] 🎯 更新目标点 (x=%.2f, y=%.2f)\n", target_x, target_y);
            }
        }
    } else {
        printf("[ObjectSearchPlanner] 未找到 /objectsearch/next_target 参数\n");
    }

    // ✅ 2. 正常规划逻辑
    if (targets_.empty() || current_idx_ >= targets_.size())
        return {false, {}};

    plan_t plan;
    geometry_msgs::PoseStamped start;
    pos_t current = getPosition();

    start.header.frame_id = "map";
    start.pose.position.x = current.x();
    start.pose.position.y = current.y();
    start.pose.orientation.w = 1.0;
    plan.push_back(start);

    geometry_msgs::PoseStamped goal = targets_[current_idx_];
    goal.pose.orientation.w = 1.0;
    plan.push_back(goal);

    latest_plan_ = plan;
    return {true, latest_plan_};
}


void ObjectSearchPlanner::signalNavigationFailure() {
    if (current_idx_ >= targets_.size())
        current_idx_ = 0;
}

}
