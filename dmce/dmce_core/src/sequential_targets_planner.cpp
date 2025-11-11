#include "dmce_core/sequential_targets_planner.hpp"
#include "dmce_core/TypeDefs.hpp"
#include <cmath>

namespace dmce {

SequentialTargetsPlanner::SequentialTargetsPlanner(double robotDiameter)
    : Planner(robotDiameter), current_idx_(0), goal_tolerance_(0.5) {
    // 这里写死几个目标点，实际可以改成从 rosparam 加载
    geometry_msgs::PoseStamped p1, p2, p3;

    p1.pose.position.x = -5.00; p1.pose.position.y = 2.48;
    p2.pose.position.x = 7.40; p2.pose.position.y = 2.48;
    p3.pose.position.x = -10.40; p3.pose.position.y = -0.48;

    targets_ = {p1, p2, p3};
    // targets_.clear();

}

bool SequentialTargetsPlanner::reachedTarget_(const pos_t& current,
                                              const geometry_msgs::PoseStamped& goal) {
    double dx = current.x() - goal.pose.position.x;
    double dy = current.y() - goal.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist < goal_tolerance_;
}

void SequentialTargetsPlanner::updatePlan_() {
    if (targets_.empty()) return;
    if (current_idx_ >= targets_.size()) return;

    pos_t current = getPosition();   // ✅ 用基类提供的接口
    if (reachedTarget_(current, targets_[current_idx_])) {
        current_idx_++;
    }
}

std::pair<bool, plan_t> SequentialTargetsPlanner::getLatestPlan_() {
    if (targets_.empty() || current_idx_ >= targets_.size()) {
        return {false, {}};
    }

    plan_t plan;

    geometry_msgs::PoseStamped start;
    pos_t current = getPosition();   // ✅ 同样这里
    start.pose.position.x = current.x();
    start.pose.position.y = current.y();
    plan.push_back(start);

    plan.push_back(targets_[current_idx_]);

    latest_plan_ = plan;
    return {true, latest_plan_};
}


void SequentialTargetsPlanner::signalNavigationFailure() {
    if (current_idx_ >= targets_.size()) {
        current_idx_ = 0;
    }
}

} // namespace dmce
