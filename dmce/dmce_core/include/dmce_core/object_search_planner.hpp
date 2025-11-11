#pragma once
#include "Planner.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <vector>


namespace dmce {

class ObjectSearchPlanner : public Planner {
public:

    plan_t latestPlan_;
    explicit ObjectSearchPlanner(double robotDiameter);

    // ✅ 外部接口：可手动设置新目标（例如调试用）
    void setTargets(const std::vector<geometry_msgs::PoseStamped>& newTargets);

protected:
    // ✅ 每次主循环自动执行，用于检测 /objectsearch/next_target 是否变化
    void updatePlan_() override;

    // ✅ GlobalPlannerService 调用时使用，返回最新路径
    std::pair<bool, plan_t> getLatestPlan_() override;

    // ✅ 处理导航失败
    void signalNavigationFailure() override;

private:
    // --- 内部状态 ---
    std::vector<geometry_msgs::PoseStamped> targets_;  // 所有目标点
    size_t current_idx_;                               // 当前目标索引
    double goal_tolerance_;                            // 目标容差

    // ✅ 工具函数
    bool reachedTarget_(const pos_t& current, const geometry_msgs::PoseStamped& goal);
};

}  // namespace dmce
