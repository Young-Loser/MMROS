#pragma once
#include "dmce_core/Planner.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace dmce {

class SequentialTargetsPlanner : public Planner {
public:
    SequentialTargetsPlanner(double robotDiameter);

protected:
    void updatePlan_() override;
    std::pair<bool, plan_t> getLatestPlan_() override;
    void signalNavigationFailure() override;

private:
    std::vector<geometry_msgs::PoseStamped> targets_;
    size_t current_idx_;
    plan_t latest_plan_;
    double goal_tolerance_;

    bool reachedTarget_(const pos_t& current, const geometry_msgs::PoseStamped& goal);
};

}
