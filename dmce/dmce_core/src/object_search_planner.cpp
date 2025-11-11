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

void ObjectSearchPlanner::updatePlan_() {

    // 每次调用时重新读取最新目标参数
    XmlRpc::XmlRpcValue target_param;
    if (ros::param::get("/objectsearch/next_target", target_param)) {
        double target_x = static_cast<double>(target_param["x"]);
        double target_y = static_cast<double>(target_param["y"]);

        pos_t target(target_x, target_y);
        plan_t plan;
        plan.push_back(posToPose(target));
        latestPlan_ = plan;
        ROS_INFO_THROTTLE(1.0, "[ObjectSearchPlanner]  new target plan (%.2f, %.2f)", target_x, target_y);
    }

}

std::pair<bool, plan_t> ObjectSearchPlanner::getLatestPlan_() {

    // ⬇️ 与 ExternalPlanner 一样：返回并清空
    auto plan = latestPlan_;
    latestPlan_.clear();
    return { (plan.size() > 0), plan };
}


void ObjectSearchPlanner::signalNavigationFailure() {
    if (current_idx_ >= targets_.size())
        current_idx_ = 0;
    latestPlan_.clear();
}

} // namespace dmce


